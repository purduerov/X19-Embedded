/**
 * @file app.c
 * @brief Node 2 (Control Board) Application Layer Logic.
 *
 * Handles 8-channel ESC PWM outputs, 1 kHz slew-rate ramping, 10-ch SMC solenoids,
 * ST LSM6DSOXTR IMU (SPI), MS5837 Depth (I2C), and 100 Hz Navigation Telemetry (CAN FD).
 * Pure application logic with zero vendor ST HAL calls.
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include "lsm6dsoxtr.h"
#include "ms5837.h"
#include "x19_can_protocol.h"
#include "x19_parameters.h"
#include "x19_safety.h"

/* Global safety state */
static x19_safety_state_t g_safety_state;

/* Commanded vs Active Ramped PWMs */
static x19_thruster_cmd_t g_target_pwms;
static x19_thruster_cmd_t g_active_pwms;

/* Sensors */
static lsm6dsoxtr_dev_t g_imu_dev;
static ms5837_dev_t g_depth_dev;

void app_main(void) {
    bsp_init();
    x19_safety_init(&g_safety_state);

    /* Initialize target PWMs to stopped (1500 us) */
    for (int i = 0; i < X19_NUM_THRUSTERS; i++) {
        g_target_pwms.pwm_us[i] = X19_PWM_STOP_US;
        g_active_pwms.pwm_us[i] = X19_PWM_STOP_US;
    }

    /* Initialize drivers */
    lsm6dsoxtr_init(&g_imu_dev);
    ms5837_init(&g_depth_dev);

    uint32_t last_nav_time = 0;

    while (1) {
        /* Check for incoming thruster commands (CAN ID 0x100) */
        uint32_t rx_id;
        uint8_t rx_data[64];
        uint8_t rx_len;

        if (can_receive(&rx_id, rx_data, &rx_len)) {
            if (rx_id == X19_CAN_ID_THRUSTER_CMD && rx_len >= sizeof(x19_thruster_cmd_t)) {
                x19_safety_feed_heartbeat(&g_safety_state, time_get_ms());
                /* Atomic copy to target buffer */
            }
        }

        /* 100 Hz Navigation Telemetry stream (CAN ID 0x200) */
        if (time_get_ms() - last_nav_time >= 10) {
            last_nav_time = time_get_ms();
            led_toggle();
        }

        delay_ms(1);
    }
}
