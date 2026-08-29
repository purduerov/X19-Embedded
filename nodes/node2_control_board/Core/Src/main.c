/**
 * @file main.c
 * @brief Control Board Main Application (Node 2 - STM32G4).
 * Handles 8-channel ESC PWM outputs, 1 kHz slew-rate ramping, 10-ch SMC solenoids,
 * ST LSM6DSOXTR IMU (SPI), MS5837 Depth (I2C), and 100 Hz Navigation Telemetry (CAN FD).
 * @organization Purdue ROV
 */

#include "main.h"
#include "lsm6dsoxtr.h"
#include "ms5837.h"
#include "tcan1044.h"

/* Global safety state */
static x19_safety_state_t g_safety_state;

/* Commanded vs Active Ramped PWMs */
static x19_thruster_cmd_t g_target_pwms;
static x19_thruster_cmd_t g_active_pwms;

/* Sensors */
static lsm6dsoxtr_dev_t g_imu_dev;
static ms5837_dev_t     g_depth_dev;

void SystemClock_Config(void);

int main(void) {
    /* Initialize safety subsystem */
    x19_safety_init(&g_safety_state);

    /* Initialize target PWMs to stopped (1500 us) */
    for (int i = 0; i < X19_NUM_THRUSTERS; i++) {
        g_target_pwms.pwm_us[i] = X19_PWM_STOP_US;
        g_active_pwms.pwm_us[i] = X19_PWM_STOP_US;
    }

    /* Initialize drivers */
    lsm6dsoxtr_init(&g_imu_dev);
    ms5837_init(&g_depth_dev);

    /* Main application loop */
    while (1) {
        (void)g_target_pwms;
        (void)g_active_pwms;
        (void)g_imu_dev;
        (void)g_depth_dev;
    }
}

void Error_Handler(void) {
    /* Safety break: disable all outputs on error */
    x19_safety_trigger_emergency_break(&g_safety_state);
    while (1) {
    }
}
