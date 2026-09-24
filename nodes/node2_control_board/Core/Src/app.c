/**
 * @file app.c
 * @brief Node 2 (Control Board) Application Layer Logic.
 *
 * Handles 8-channel ESC PWM outputs, 1 kHz slew-rate ramping, 10-ch SMC solenoids,
 * ST LSM6DSOXTR IMU (SPI), MS5837 Depth (I2C), and 100 Hz Navigation Telemetry (CAN FD).
 * Pure application logic with zero vendor ST HAL calls.
 * @organization Purdue ROV
 */

#include "app.h"
#include "bmi270.h"
#include "bsp.h"
#include "can_interface.h"
#include "ms5837.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_pwm_ramp.h"
#include "rov_safety.h"
#include "rov_timesync.h"
#include <math.h>
#include <string.h>

#define ESC_ARMING_TIME_MS 3000U

typedef enum { ESC_STATE_BOOT, ESC_STATE_ARMING, ESC_STATE_ACTIVE, ESC_STATE_DISARMED } esc_state_t;

static esc_state_t g_esc_state = ESC_STATE_BOOT;
static uint32_t g_esc_arming_start_ms = 0U;

static rov_safety_state_t g_safety_state;
static rov_timesync_state_t g_timesync;
static rov_thruster_cmd_t g_target_pwms;
static rov_thruster_cmd_t g_active_pwms;
static bmi270_dev_t g_imu_dev;
static ms5837_dev_t g_depth_dev;
static uint32_t g_last_nav_time = 0;
static uint32_t g_last_ramp_time = 0;
static bool g_can_ready = false;

static void node2_force_pwm_neutral(void) {
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_target_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        g_active_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        bsp_pwm_set_us((uint8_t)i, ROV_PWM_STOP_US);
    }
}

static void node2_force_neutral(void) {
    node2_force_pwm_neutral();
    bsp_solenoid_set(0);
}

void node2_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);
    rov_timesync_init(&g_timesync);

    g_can_ready = can_init();

    g_esc_state = ESC_STATE_BOOT;

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_target_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        g_active_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        bsp_pwm_set_us((uint8_t)i, ROV_PWM_STOP_US);
    }

    g_esc_arming_start_ms = time_get_ms();
    g_esc_state = ESC_STATE_ARMING;

    bsp_solenoid_set(0);

    if (!g_can_ready) {
        g_esc_state = ESC_STATE_DISARMED;
        g_safety_state.emergency_break_active = true;
        node2_force_neutral();
        bsp_emergency_brake_trip();
        return;
    }

    bmi270_init(&g_imu_dev);
    ms5837_init(&g_depth_dev);

    g_last_nav_time = 0;
    g_last_ramp_time = 0;
}

void node2_app_step(void) {
    uint32_t current_time = time_get_ms();

    if (!g_can_ready) {
        node2_force_neutral();
        bsp_emergency_brake_trip();
        delay_ms(1);
        return;
    }

    if (g_esc_state == ESC_STATE_ARMING && (uint32_t)(current_time - g_esc_arming_start_ms) >= ESC_ARMING_TIME_MS) {
        g_esc_state = ESC_STATE_ACTIVE;
        g_last_ramp_time = current_time;
    }

    /* Process all incoming CAN frames */
    uint32_t rx_id;
    uint8_t rx_data[64];
    uint8_t rx_len;

    for (uint32_t frame_budget = 0; frame_budget < 32U && can_receive(&rx_id, rx_data, &rx_len); frame_budget++) {
        if (rx_id == ROV_CAN_ID_EMERGENCY_BREAK) {
            /* Emergency break received: verify magic signature (0xAA, 0x55) for authorization */
            if (rx_len >= 2 && rx_data[0] == 0xAA && rx_data[1] == 0x55) {
                /* Instant hardware and software shutdown */
                rov_safety_trigger_emergency_break(&g_safety_state);
                bsp_emergency_brake_trip();
                g_esc_state = ESC_STATE_DISARMED;

                node2_force_neutral();
            }
        } else if (rx_id == ROV_CAN_ID_EFUSE_FAULT_ALERT) {
            node2_force_neutral();
            bsp_emergency_brake_trip();
            g_esc_state = ESC_STATE_DISARMED;
            rov_safety_trigger_emergency_break(&g_safety_state);
        } else if (rx_id == ROV_CAN_ID_THRUSTER_CMD) {
            /* Thruster commands are accepted only after ESC arming completes */
            if ((g_esc_state == ESC_STATE_ACTIVE) && (!g_safety_state.emergency_break_active)) {
                rov_thruster_cmd_t cmd;

                if (rov_can_unpack_thruster_cmd(rx_data, rx_len, &cmd) == ROV_OK) {
                    rov_safety_feed_heartbeat(&g_safety_state, current_time);

                    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                        uint16_t target_us = cmd.pwm_us[i];

                        if (target_us < 1000U) {
                            target_us = 1000U;
                        } else if (target_us > 2000U) {
                            target_us = 2000U;
                        }

                        g_target_pwms.pwm_us[i] = target_us;
                    }
                }
            }
        } else if (rx_id == ROV_CAN_ID_SOLENOID_CMD) {
            rov_solenoid_cmd_t sol;
            if (!g_safety_state.emergency_break_active &&
                rov_can_unpack_solenoid_cmd(rx_data, rx_len, &sol) == ROV_OK) {
                bsp_solenoid_set(sol.solenoid_mask);
            }
        } else if (rx_id == ROV_CAN_ID_TIME_SYNC_MASTER) {
            rov_time_sync_master_t sync_msg;
            if (rov_can_unpack_time_sync_master(rx_data, rx_len, &sync_msg) == ROV_OK) {
                rov_timesync_process_master(&g_timesync, &sync_msg, time_get_us());
            }
        } else if (rx_id == ROV_CAN_ID_TIME_SYNC_REQ) {
            rov_time_sync_req_t req;
            if (rov_can_unpack_time_sync_req(rx_data, rx_len, &req) == ROV_OK) {
                if (req.target_node_id == ROV_NODE_CONTROL_BOARD || req.target_node_id == ROV_NODE_BROADCAST) {
                    uint64_t rx_time_us = time_get_us();
                    rov_time_sync_resp_t resp;
                    resp.responder_node_id = ROV_NODE_CONTROL_BOARD;
                    resp.seq = req.seq;
                    resp.reserved = 0;
                    resp.status = rov_timesync_is_synchronized(&g_timesync, time_get_us()) ? 1 : 0;
                    resp.t1_us = req.t1_us;
                    resp.t2_us = rx_time_us;
                    resp.t3_us = time_get_us();
                    uint8_t resp_buf[64];
                    size_t resp_len = 0;
                    if (rov_can_pack_time_sync_resp(&resp, resp_buf, sizeof(resp_buf), &resp_len) == ROV_OK) {
                        can_send(ROV_CAN_ID_TIME_SYNC_RESP, resp_buf, (uint8_t)resp_len);
                    }
                }
            }
        }
    }

    /* Check heartbeat timeout: if no thruster command in 100 ms, drop to neutral */
    bool heartbeat_lost = rov_safety_is_heartbeat_lost(&g_safety_state, current_time);
    if (g_esc_state == ESC_STATE_ACTIVE && heartbeat_lost) {
        g_safety_state.watchdog_expired = true;
    }
    if ((g_esc_state == ESC_STATE_ACTIVE && heartbeat_lost) || g_safety_state.emergency_break_active) {
        node2_force_neutral();
    }

    /* Hold all ESCs at neutral throughout the mandatory arming period */
    if (g_esc_state == ESC_STATE_ARMING) {
        node2_force_pwm_neutral();
        g_last_ramp_time = current_time;
    } else if (g_esc_state == ESC_STATE_DISARMED) {
        node2_force_neutral();
        g_last_ramp_time = current_time;
    }
    /* 1 kHz Slew-Rate Ramping Step (executed every 1 ms or on step) */
    else if ((uint32_t)(current_time - g_last_ramp_time) > 0U) {
        uint32_t dt_ms = current_time - g_last_ramp_time;
        g_last_ramp_time = current_time;

        /* Hoist loop invariant: calculate max_step and branch on safety state once outside the loop */
        if (g_safety_state.emergency_break_active) {
            node2_force_neutral();
        } else {
            uint64_t max_step_u64 = (uint64_t)ROV_PWM_MAX_SLEW_RATE_US_PER_MS * dt_ms;
            uint16_t max_step = max_step_u64 > UINT16_MAX ? UINT16_MAX : (uint16_t)max_step_u64;
            for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                g_active_pwms.pwm_us[i] = rov_pwm_step_ramp(g_active_pwms.pwm_us[i], g_target_pwms.pwm_us[i], max_step);
                bsp_pwm_set_us((uint8_t)i, g_active_pwms.pwm_us[i]);
            }
        }
    }

    /* 100 Hz Navigation Telemetry Stream (0x200) */
    if (current_time - g_last_nav_time >= (1000 / ROV_NAV_TELEMETRY_FREQ_HZ)) {
        g_last_nav_time = current_time;

        rov_status_t imu_status = bmi270_read_raw(&g_imu_dev);
        rov_status_t depth_status = ms5837_read_pressure_depth(&g_depth_dev, 1000.0f);
        if (imu_status == ROV_OK && (!isfinite(g_imu_dev.q_w) || !isfinite(g_imu_dev.q_x) || !isfinite(g_imu_dev.q_y) ||
                                     !isfinite(g_imu_dev.q_z) || !isfinite(g_imu_dev.gyro_x_dps) ||
                                     !isfinite(g_imu_dev.gyro_y_dps) || !isfinite(g_imu_dev.gyro_z_dps))) {
            imu_status = ROV_ERROR;
        }
        if (depth_status == ROV_OK && (!isfinite(g_depth_dev.depth_meters) || g_depth_dev.depth_meters < 0.0f)) {
            depth_status = ROV_ERROR;
        }

        rov_nav_telemetry_t nav;
        nav.timestamp_us = rov_timesync_get_time_us(&g_timesync, time_get_us());
        if (imu_status == ROV_OK) {
            nav.q_w = g_imu_dev.q_w;
            nav.q_x = g_imu_dev.q_x;
            nav.q_y = g_imu_dev.q_y;
            nav.q_z = g_imu_dev.q_z;
            nav.gyro_x_rad_s = g_imu_dev.gyro_x_dps * (3.14159265f / 180.0f);
            nav.gyro_y_rad_s = g_imu_dev.gyro_y_dps * (3.14159265f / 180.0f);
            nav.gyro_z_rad_s = g_imu_dev.gyro_z_dps * (3.14159265f / 180.0f);
        } else {
            nav.q_w = 1.0f;
            nav.q_x = 0.0f;
            nav.q_y = 0.0f;
            nav.q_z = 0.0f;
            nav.gyro_x_rad_s = 0.0f;
            nav.gyro_y_rad_s = 0.0f;
            nav.gyro_z_rad_s = 0.0f;
        }
        nav.depth_meters = depth_status == ROV_OK ? g_depth_dev.depth_meters : 0.0f;
        nav.imu_status = imu_status == ROV_OK ? g_imu_dev.status_flags : 0U;

        uint8_t tx_buf[64];
        size_t packed_len = 0;
        if (rov_can_pack_nav_telemetry(&nav, tx_buf, sizeof(tx_buf), &packed_len) == ROV_OK) {
            can_send(ROV_CAN_ID_NAV_TELEMETRY, tx_buf, (uint8_t)packed_len);
        }

        led_toggle();
    }

    delay_ms(1);
}

#ifndef ROV_UNIT_TEST
void app_main(void) {
    node2_app_init();
    while (1) {
        node2_app_step();
    }
}
#endif
