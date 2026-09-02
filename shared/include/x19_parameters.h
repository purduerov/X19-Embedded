/**
 * @file x19_parameters.h
 * @brief Authoritative system parameters, timing intervals, physical bounds, and safety limits.
 * @organization Purdue ROV
 */

#ifndef X19_PARAMETERS_H
#define X19_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* VEHICLE ELECTRICAL & POWER ENVELOPE PARAMETERS                             */
/* ========================================================================== */
#define X19_TETHER_NOMINAL_VOLTAGE_V (48.0f)
#define X19_TETHER_MAX_CURRENT_A     (25.0f)
#define X19_TETHER_MAX_POWER_W       (1200.0f)

#define X19_LOGIC_RAIL_VOLTAGE_V     (5.2f)
#define X19_LOGIC_RAIL_MAX_POWER_W   (50.0f)
#define X19_LOGIC_RAIL_MAX_CURRENT_A (9.6f)

#define X19_NUM_12V_BRICKS      (4)
#define X19_BRICK_MAX_POWER_W   (300.0f)
#define X19_BRICK_MAX_CURRENT_A (25.0f)
#define X19_THRUSTERS_PER_BRICK (2)

/* Thruster current software cap (12.5A per thruster = 150W per thruster) */
#define X19_THRUSTER_MAX_CURRENT_A (12.5f)
#define X19_THRUSTER_MAX_POWER_W   (150.0f)

/* ========================================================================== */
/* THRUSTER PWM BOUNDS & TIMING PARAMETERS                                    */
/* ========================================================================== */
#define X19_NUM_THRUSTERS    (8)
#define X19_PWM_STOP_US      (1500)
#define X19_PWM_MIN_US       (1000)
#define X19_PWM_MAX_US       (2000)
#define X19_PWM_DEADBAND_US  (25)
#define X19_PWM_FREQUENCY_HZ (400)

/* 1 kHz Slew-Rate Ramping Rate (us per millisecond) */
#define X19_PWM_MAX_SLEW_RATE_US_PER_MS (2)

/* Exponential response curve coefficient (0.0 = linear, 1.0 = full cubic) */
#define X19_PWM_EXPO_FACTOR (0.65f)

/* ========================================================================== */
/* PNEUMATIC ACTUATION PARAMETERS                                             */
/* ========================================================================== */
#define X19_NUM_SOLENOIDS         (5)  /* SMC SY3400-6U1-NA */
#define X19_NUM_SOLENOID_CHANNELS (10) /* 2 discrete MOSFET channels per valve */

/* ========================================================================== */
/* CAN FD BUS TIMING & BITRATES                                               */
/* ========================================================================== */
#define X19_CAN_ARBITRATION_BITRATE_BPS (1000000) /* 1 Mbps */
#define X19_CAN_DATA_BITRATE_BPS        (5000000) /* 5 Mbps */
#define X19_CAN_MAX_PAYLOAD_BYTES       (64)

/* ========================================================================== */
/* TELEMETRY STREAMING FREQUENCIES & TIMEOUTS                                 */
/* ========================================================================== */
#define X19_NAV_TELEMETRY_FREQ_HZ     (100)
#define X19_ENV_TELEMETRY_FREQ_HZ     (10)
#define X19_POWER_TELEMETRY_FREQ_HZ   (20)
#define X19_USB_HUB_TELEMETRY_FREQ_HZ (5)

#define X19_HEARTBEAT_TIMEOUT_MS    (100)
#define X19_WATCHDOG_TIMEOUT_MS     (150)
#define X19_I2C_RECOVERY_TIMEOUT_MS (10)

/* ========================================================================== */
/* FLASH MEMORY & BOOTLOADER PARTITIONS                                       */
/* ========================================================================== */
#define X19_FLASH_BASE_ADDR             (0x08000000)
#define X19_FLASH_BOOTLOADER_SIZE_BYTES (0x4000)     /* 16 KB Sector 0 */
#define X19_FLASH_APP_START_ADDR        (0x08004000) /* Sector 1 Vector Table */

#ifdef __cplusplus
}
#endif

#endif /* X19_PARAMETERS_H */
