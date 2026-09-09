/**
 * @file rov_types.h
 * @brief Common types, telemetry data structures, and status enums for Purdue ROV Embedded Systems.
 * @organization Purdue ROV
 */

#ifndef ROV_TYPES_H
#define ROV_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Standard return codes across all ROV embedded drivers and libraries.
 */
typedef enum {
    ROV_OK = 0x00,
    ROV_ERROR = 0x01,
    ROV_BUSY = 0x02,
    ROV_TIMEOUT = 0x03,
    ROV_ERR_INVALID_ARG = 0x04,
    ROV_ERR_CRC_MISMATCH = 0x05,
    ROV_ERR_BUS_LOCKED = 0x06,
    ROV_ERR_OVERCURRENT = 0x07,
    ROV_ERR_OVERTEMP = 0x08,
    ROV_ERR_LEAK_DETECTED = 0x09,
    ROV_ERR_SAFETY_TRIP = 0x0A
} rov_status_t;

/**
 * @brief Vehicle Node Identifiers.
 */
typedef enum {
    ROV_NODE_PI_CORE = 0x00,
    ROV_NODE_PI_SHIELD = 0x01,
    ROV_NODE_CONTROL_BOARD = 0x02,
    ROV_NODE_POWER_SLAB = 0x03,
    ROV_NODE_USB_HUB = 0x04,
    ROV_NODE_BROADCAST = 0xFF
} rov_node_id_t;

/**
 * @brief 8-Channel Thruster PWM Command Payload (Packed, 16 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t pwm_us[8]; /**< Pulse width in microseconds (1000 to 2000 us, 1500 = Stop) */
} rov_thruster_cmd_t;

/**
 * @brief 10-Channel Pneumatic Solenoid Command Payload (Packed, 2 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t solenoid_mask; /**< Bitmask of 10 solenoid channels (1 = Energized/12V, 0 = Off) */
} rov_solenoid_cmd_t;

/**
 * @brief IMU Sensor Data Payload.
 */
typedef struct __attribute__((packed)) {
    float q_w;          /**< Orientation Quaternion W */
    float q_x;          /**< Orientation Quaternion X */
    float q_y;          /**< Orientation Quaternion Y */
    float q_z;          /**< Orientation Quaternion Z */
    float gyro_x_dps;   /**< Angular Velocity X (dps) */
    float gyro_y_dps;   /**< Angular Velocity Y (dps) */
    float gyro_z_dps;   /**< Angular Velocity Z (dps) */
    uint8_t status;     /**< IMU status */
} imu_data_t;

/**
 * @brief 100 Hz Navigation Telemetry Payload from Control Board (Packed, 33 bytes).
 */
typedef struct __attribute__((packed)) {
    float q_w;          /**< Orientation Quaternion W */
    float q_x;          /**< Orientation Quaternion X */
    float q_y;          /**< Orientation Quaternion Y */
    float q_z;          /**< Orientation Quaternion Z */
    float gyro_x_rad_s; /**< Angular Velocity X (rad/s) */
    float gyro_y_rad_s; /**< Angular Velocity Y (rad/s) */
    float gyro_z_rad_s; /**< Angular Velocity Z (rad/s) */
    float depth_meters; /**< Hydrostatic depth in meters (from MS5837) */
    uint8_t imu_status; /**< IMU calibration & health flags (0 = Uncalibrated, 3 = High Precision) */
} rov_nav_telemetry_t;

/**
 * @brief 10 Hz Environmental & Leak Telemetry from Pi Shield (Packed, 13 bytes).
 */
typedef struct __attribute__((packed)) {
    float pressure_hpa;  /**< Enclosure internal pressure in hPa (BME280) */
    float humidity_pct;  /**< Enclosure relative humidity in % (BME280) */
    float temperature_c; /**< Enclosure internal temperature in deg C (BME280) */
    uint8_t leak_flags;  /**< Bit 0: BME280 trigger, Bit 1: Floor leak trace 1, Bit 2: Floor leak trace 2 */
} rov_env_telemetry_t;

/**
 * @brief 20 Hz Power Slab Telemetry Payload (Packed, 24 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t tether_voltage_mv; /**< Tether Voltage in millivolts */
    uint16_t tether_current_ma; /**< Tether Current in milliamperes */
    uint16_t v5_voltage_mv;     /**< 5.2V Main Logic Rail Voltage in mV */
    uint16_t v5_current_ma;     /**< 5.2V Main Logic Rail Current in mA */
    uint16_t v12_current_ma[4]; /**< Current draw per 12V 300W Brick (mA) */
    int16_t pcb_temp_c;         /**< Power Slab Copper Temperature (0.1 deg C) */
    uint16_t status_flags;      /**< eFuse status, ideal diode status, fault bits */
} rov_power_telemetry_t;

/**
 * @brief 5 Hz USB Camera Hub Telemetry Payload (Packed, 20 bytes).
 */
typedef struct __attribute__((packed)) {
    uint16_t vbus_voltage_mv;    /**< 5.0V VBUS Line Voltage (mV) */
    uint16_t port_current_ma[8]; /**< Current per camera port (mA) */
    int16_t controller_temp_c;   /**< USB Host Controller Temperature (0.1 deg C) */
    uint8_t port_power_mask;     /**< Active VBUS power bitmask (8 ports) */
    uint8_t fault_flags;         /**< Overcurrent, thermal warning flags */
} rov_usb_hub_telemetry_t;

/**
 * @brief Bootloader Control Command Identifiers.
 */
typedef enum {
    ROV_BOOT_CMD_PING = 0x01,
    ROV_BOOT_CMD_ERASE_APP = 0x02,
    ROV_BOOT_CMD_START_FLASH = 0x03,
    ROV_BOOT_CMD_VERIFY_APP = 0x04,
    ROV_BOOT_CMD_JUMP_APP = 0x05,
    ROV_BOOT_ACK = 0x06,
    ROV_BOOT_NACK = 0x07
} rov_boot_cmd_t;

/* ========================================================================== */
/* BACKWARD COMPATIBILITY ALIASES (X19 Vehicle Profile)                       */
/* ========================================================================== */
typedef rov_status_t x19_status_t;
#define X19_OK                ROV_OK
#define X19_ERROR             ROV_ERROR
#define X19_BUSY              ROV_BUSY
#define X19_TIMEOUT           ROV_TIMEOUT
#define X19_ERR_INVALID_ARG   ROV_ERR_INVALID_ARG
#define X19_ERR_CRC_MISMATCH  ROV_ERR_CRC_MISMATCH
#define X19_ERR_BUS_LOCKED    ROV_ERR_BUS_LOCKED
#define X19_ERR_OVERCURRENT   ROV_ERR_OVERCURRENT
#define X19_ERR_OVERTEMP      ROV_ERR_OVERTEMP
#define X19_ERR_LEAK_DETECTED ROV_ERR_LEAK_DETECTED
#define X19_ERR_SAFETY_TRIP   ROV_ERR_SAFETY_TRIP

typedef rov_node_id_t x19_node_id_t;
#define X19_NODE_PI_CORE       ROV_NODE_PI_CORE
#define X19_NODE_PI_SHIELD     ROV_NODE_PI_SHIELD
#define X19_NODE_CONTROL_BOARD ROV_NODE_CONTROL_BOARD
#define X19_NODE_POWER_SLAB    ROV_NODE_POWER_SLAB
#define X19_NODE_USB_HUB       ROV_NODE_USB_HUB
#define X19_NODE_BROADCAST     ROV_NODE_BROADCAST

typedef rov_thruster_cmd_t x19_thruster_cmd_t;
typedef rov_solenoid_cmd_t x19_solenoid_cmd_t;
typedef rov_nav_telemetry_t x19_nav_telemetry_t;
typedef rov_env_telemetry_t x19_env_telemetry_t;
typedef rov_power_telemetry_t x19_power_telemetry_t;
typedef rov_usb_hub_telemetry_t x19_usb_hub_telemetry_t;
typedef rov_boot_cmd_t x19_boot_cmd_t;

#define X19_BOOT_CMD_PING        ROV_BOOT_CMD_PING
#define X19_BOOT_CMD_ERASE_APP   ROV_BOOT_CMD_ERASE_APP
#define X19_BOOT_CMD_START_FLASH ROV_BOOT_CMD_START_FLASH
#define X19_BOOT_CMD_VERIFY_APP  ROV_BOOT_CMD_VERIFY_APP
#define X19_BOOT_CMD_JUMP_APP    ROV_BOOT_CMD_JUMP_APP
#define X19_BOOT_ACK             ROV_BOOT_ACK
#define X19_BOOT_NACK            ROV_BOOT_NACK

#ifdef __cplusplus
}
#endif

#endif /* ROV_TYPES_H */
