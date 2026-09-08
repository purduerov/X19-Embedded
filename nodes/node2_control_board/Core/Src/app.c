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
#include "bsp.h"
#include "can_interface.h"
#include "lsm6dsoxtr.h"
#include "ms5837.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_pwm_ramp.h"
#include "rov_safety.h"
#include <string.h>

static rov_safety_state_t g_safety_state;
static rov_thruster_cmd_t g_target_pwms;
static rov_thruster_cmd_t g_active_pwms;
static lsm6dsoxtr_dev_t g_imu_dev;
static ms5837_dev_t g_depth_dev;
static uint32_t g_last_nav_time = 0;
static uint32_t g_last_ramp_time = 0;

void node2_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_target_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        g_active_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        bsp_pwm_set_us((uint8_t)i, ROV_PWM_STOP_US);
    }
    bsp_solenoid_set(0);

    lsm6dsoxtr_init(&g_imu_dev);
    ms5837_init(&g_depth_dev);

    g_last_nav_time = 0;
    g_last_ramp_time = 0;
}

void node2_app_step(void) {
    uint32_t current_time = time_get_ms();

    /* Process all incoming CAN frames */
    uint32_t rx_id;
    uint8_t rx_data[64];
    uint8_t rx_len;

    while (can_receive(&rx_id, rx_data, &rx_len)) {
        if (rx_id == ROV_CAN_ID_EMERGENCY_BREAK) {
            /* Emergency break received: verify magic signature (0xAA, 0x55) for authorization */
            if (rx_len >= 2 && rx_data[0] == 0xAA && rx_data[1] == 0x55) {
                /* Instant hardware and software shutdown */
                rov_safety_trigger_emergency_break(&g_safety_state);
                bsp_emergency_brake_trip();
                for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                    g_target_pwms.pwm_us[i] = ROV_PWM_STOP_US;
                    g_active_pwms.pwm_us[i] = ROV_PWM_STOP_US;
                    bsp_pwm_set_us((uint8_t)i, ROV_PWM_STOP_US);
                }
            }
        } else if (rx_id == ROV_CAN_ID_THRUSTER_CMD) {
            /* Only accept thruster commands if emergency break is not active */
            if (!g_safety_state.emergency_break_active) {
                rov_thruster_cmd_t cmd;
                if (rov_can_unpack_thruster_cmd(rx_data, rx_len, &cmd) == ROV_OK) {
                    rov_safety_feed_heartbeat(&g_safety_state, current_time);
                    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                        g_target_pwms.pwm_us[i] = cmd.pwm_us[i];
                    }
                }
            }
        } else if (rx_id == ROV_CAN_ID_SOLENOID_CMD) {
            rov_solenoid_cmd_t sol;
            if (rov_can_unpack_solenoid_cmd(rx_data, rx_len, &sol) == ROV_OK) {
                bsp_solenoid_set(sol.solenoid_mask);
            }
        }
    }

    /* Check heartbeat timeout: if no thruster command in 100 ms, drop to neutral */
    bool heartbeat_lost = rov_safety_is_heartbeat_lost(&g_safety_state, current_time);
    if (heartbeat_lost || g_safety_state.emergency_break_active) {
        for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
            g_target_pwms.pwm_us[i] = ROV_PWM_STOP_US;
        }
    }

    /* 1 kHz Slew-Rate Ramping Step (executed every 1 ms or on step) */
    if (current_time > g_last_ramp_time) {
        uint32_t dt_ms = current_time - g_last_ramp_time;
        g_last_ramp_time = current_time;

        /* Hoist loop invariant: calculate max_step once outside the loop */
        uint16_t max_step = 0;
        if (!g_safety_state.emergency_break_active) {
            max_step = (uint16_t)(ROV_PWM_MAX_SLEW_RATE_US_PER_MS * dt_ms);
        }

        if (g_safety_state.emergency_break_active) {
            for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                g_active_pwms.pwm_us[i] = ROV_PWM_STOP_US;
                bsp_pwm_set_us((uint8_t)i, g_active_pwms.pwm_us[i]);
            }
        } else {
            for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
                g_active_pwms.pwm_us[i] = rov_pwm_step_ramp(g_active_pwms.pwm_us[i], g_target_pwms.pwm_us[i], max_step);
                bsp_pwm_set_us((uint8_t)i, g_active_pwms.pwm_us[i]);
            }
        }
    }

    /* 100 Hz Navigation Telemetry Stream (0x200) */
    if (current_time - g_last_nav_time >= (1000 / ROV_NAV_TELEMETRY_FREQ_HZ)) {
        g_last_nav_time = current_time;

        lsm6dsoxtr_read_raw(&g_imu_dev);
        ms5837_read_pressure_depth(&g_depth_dev, 1000.0f);

        rov_nav_telemetry_t nav;
        nav.q_w = g_imu_dev.q_w;
        nav.q_x = g_imu_dev.q_x;
        nav.q_y = g_imu_dev.q_y;
        nav.q_z = g_imu_dev.q_z;
        nav.gyro_x_rad_s = g_imu_dev.gyro_x_dps * (3.14159265f / 180.0f);
        nav.gyro_y_rad_s = g_imu_dev.gyro_y_dps * (3.14159265f / 180.0f);
        nav.gyro_z_rad_s = g_imu_dev.gyro_z_dps * (3.14159265f / 180.0f);
        nav.depth_meters = g_depth_dev.depth_meters;
        nav.imu_status = 3;

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
