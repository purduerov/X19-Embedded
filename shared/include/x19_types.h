/**
 * @file x19_types.h
 * @brief Common types, telemetry data structures, and status enums for X19-Embedded.
 * @organization Purdue ROV
 */

#ifndef X19_TYPES_H
#define X19_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Standard return codes across all X19 embedded drivers and libraries.
 */
typedef enum {
    X19_OK                  = 0x00,
    X19_ERROR               = 0x01,
    X19_BUSY                = 0x02,
    X19_TIMEOUT             = 0x03,
    X19_ERR_INVALID_ARG     = 0x04,
    X19_ERR_CRC_MISMATCH    = 0x05,
    X19_ERR_BUS_LOCKED      = 0x06,
    X19_ERR_OVERCURRENT     = 0x07,
    X19_ERR_OVERTEMP        = 0x08,
    X19_ERR_LEAK_DETECTED   = 0x09,
    X19_ERR_SAFETY_TRIP     = 0x0A
} x19_status_t;

/**
 * @brief Vehicle Node Identifiers.
 */
typedef enum {
    X19_NODE_PI_CORE        = 0x00,
    X19_NODE_PI_SHIELD      = 0x01,
    X19_NODE_CONTROL_BOARD  = 0x02,
    X19_NODE_POWER_SLAB     = 0x03,
    X19_NODE_USB_HUB        = 0x04,
    X19_NODE_BROADCAST      = 0xFF
} x19_node_id_t;

/**
 * @brief 8-Channel Thruster PWM Command Payload (Packed, 16 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t pwm_us[8];         /**< Pulse width in microseconds (1000 to 2000 us, 1500 = Stop) */
} x19_thruster_cmd_t;

/**
 * @brief 10-Channel Pneumatic Solenoid Command Payload (Packed, 2 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t solenoid_mask;     /**< Bitmask of 10 solenoid channels (1 = Energized/12V, 0 = Off) */
} x19_solenoid_cmd_t;

/**
 * @brief 100 Hz Navigation Telemetry Payload from Control Board (Packed, 33 bytes).
 */
typedef struct __attribute__((packed)) {
    float q_w;                  /**< Orientation Quaternion W */
    float q_x;                  /**< Orientation Quaternion X */
    float q_y;                  /**< Orientation Quaternion Y */
    float q_z;                  /**< Orientation Quaternion Z */
    float gyro_x_rad_s;         /**< Angular Velocity X (rad/s) */
    float gyro_y_rad_s;         /**< Angular Velocity Y (rad/s) */
    float gyro_z_rad_s;         /**< Angular Velocity Z (rad/s) */
    float depth_meters;         /**< Hydrostatic depth in meters (from MS5837) */
    uint8_t imu_status;         /**< IMU calibration & health flags (0 = Uncalibrated, 3 = High Precision) */
} x19_nav_telemetry_t;

/**
 * @brief 10 Hz Environmental & Leak Telemetry from Pi Shield (Packed, 13 bytes).
 */
typedef struct __attribute__((packed)) {
    float pressure_hpa;         /**< Enclosure internal pressure in hPa (BME280) */
    float humidity_pct;         /**< Enclosure relative humidity in % (BME280) */
    float temperature_c;        /**< Enclosure internal temperature in deg C (BME280) */
    uint8_t leak_flags;         /**< Bit 0: BME280 trigger, Bit 1: Floor leak trace 1, Bit 2: Floor leak trace 2 */
} x19_env_telemetry_t;

/**
 * @brief 20 Hz Power Slab Telemetry Payload (Packed, 24 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t tether_voltage_mv; /**< 48V Tether Voltage in millivolts */
    uint16_t tether_current_ma; /**< 48V Tether Current in milliamperes */
    uint16_t v5_voltage_mv;     /**< 5.2V Main Logic Rail Voltage in mV */
    uint16_t v5_current_ma;     /**< 5.2V Main Logic Rail Current in mA */
    uint16_t v12_current_ma[4]; /**< Current draw per 12V 300W Brick (mA) */
    int16_t  pcb_temp_c;        /**< Power Slab Copper Temperature (0.1 deg C) */
    uint16_t status_flags;      /**< eFuse status, ideal diode status, fault bits */
} x19_power_telemetry_t;

/**
 * @brief 5 Hz USB Camera Hub Telemetry Payload (Packed, 20 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t vbus_voltage_mv;   /**< 5.0V VBUS Line Voltage (mV) */
    uint16_t port_current_ma[8];/**< Current per camera port (mA) */
    int16_t  controller_temp_c; /**< USB Host Controller Temperature (0.1 deg C) */
    uint8_t  port_power_mask;   /**< Active VBUS power bitmask (8 ports) */
    uint8_t  fault_flags;       /**< Overcurrent, thermal warning flags */
} x19_usb_hub_telemetry_t;

/**
 * @brief Bootloader Control Command Identifiers.
 */
typedef enum {
    X19_BOOT_CMD_PING           = 0x01,
    X19_BOOT_CMD_ERASE_APP      = 0x02,
    X19_BOOT_CMD_START_FLASH    = 0x03,
    X19_BOOT_CMD_VERIFY_APP     = 0x04,
    X19_BOOT_CMD_JUMP_APP       = 0x05,
    X19_BOOT_ACK                = 0x06,
    X19_BOOT_NACK               = 0x07
} x19_boot_cmd_t;

#ifdef __cplusplus
}
#endif

#endif /* X19_TYPES_H */
