/**
 * @file pmbus_brick.h
 * @brief PMBus / I2C Driver Interface for Power Slab DC-DC Converter Bricks.
 * @organization Purdue ROV
 */

#ifndef PMBUS_BRICK_H
#define PMBUS_BRICK_H

#include "rov_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Power Slab PMBus addresses */
#define PMBUS_BRICK_ADDR_12V_0 0x40U
#define PMBUS_BRICK_ADDR_12V_1 0x41U
#define PMBUS_BRICK_ADDR_12V_2 0x42U
#define PMBUS_BRICK_ADDR_12V_3 0x43U
#define PMBUS_BRICK_ADDR_5V2   0x44U

#define PMBUS_BRICK_COUNT 5U

/* Standard PMBus commands */
#define PMBUS_CMD_STATUS_WORD       0x79U
#define PMBUS_CMD_READ_VIN          0x88U
#define PMBUS_CMD_READ_VOUT         0x8BU
#define PMBUS_CMD_READ_IOUT         0x8CU
#define PMBUS_CMD_READ_TEMPERATURE1 0x8DU

/*
 * STATUS_WORD low-byte fault bits.
 */
#define PMBUS_STATUS_VOUT_OV 0x0020U
#define PMBUS_STATUS_IOUT_OC 0x0010U
#define PMBUS_STATUS_VIN_UV  0x0008U
#define PMBUS_STATUS_TEMP    0x0004U

typedef struct {
    uint8_t pmbus_addr;

    float input_voltage_v;
    float output_voltage_v;
    float output_current_a;
    float temperature_c;

    uint16_t status_word;
} pmbus_brick_dev_t;

/**
 * @brief Initialize a PMBus converter brick device.
 * @param dev Device structure.
 * @param pmbus_addr PMBus address from 0x40 through 0x44.
 * @return ROV_OK on success or an error status.
 */
rov_status_t pmbus_brick_init(pmbus_brick_dev_t *dev, uint8_t pmbus_addr);

/**
 * @brief Read telemetry from a PMBus converter brick.
 * @param dev Device structure.
 * @return ROV_OK on success or an error status.
 */
rov_status_t pmbus_brick_read_telemetry(pmbus_brick_dev_t *dev);

/**
 * @brief Convert a PMBus LINEAR11 raw value to floating point.
 * @param raw_value Raw 16-bit LINEAR11 value.
 * @return Converted floating-point value.
 */
float pmbus_linear11_to_float(uint16_t raw_value);

/**
 * @brief Check whether an address belongs to a Power Slab PMBus brick.
 * @param pmbus_addr PMBus address.
 * @return true for addresses 0x40 through 0x44.
 */
bool pmbus_brick_address_valid(uint8_t pmbus_addr);

/**
 * @brief Return the next brick address in round-robin order.
 *
 * Sequence:
 * 0x40 -> 0x41 -> 0x42 -> 0x43 -> 0x44 -> 0x40
 *
 * @param current_addr Current brick address.
 * @return Next PMBus brick address.
 */
uint8_t pmbus_brick_next_address(uint8_t current_addr);

/**
 * @brief Check STATUS_WORD for output overcurrent.
 */
bool pmbus_status_has_overcurrent(uint16_t status_word);

/**
 * @brief Check STATUS_WORD for output overvoltage.
 */
bool pmbus_status_has_overvoltage(uint16_t status_word);

/**
 * @brief Check STATUS_WORD for input undervoltage / UVLO.
 */
bool pmbus_status_has_uvlo(uint16_t status_word);

/**
 * @brief Check STATUS_WORD for a temperature fault.
 */
bool pmbus_status_has_thermal_fault(uint16_t status_word);

/**
 * @brief Convert an unsigned PMBus Linear16 value using a fixed exponent.
 * @param raw_value Raw 16-bit mantissa.
 * @param exponent Signed base-2 exponent.
 * @return Converted floating-point value.
 */
float pmbus_linear16_to_float(uint16_t raw_value, int8_t exponent);

#ifdef __cplusplus
}
#endif

#endif /* PMBUS_BRICK_H */
