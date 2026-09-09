/**
 * @file pmbus_brick.h
 * @brief Mornsun Isolated DC-DC Converter Brick PMBus / I2C Driver Interface.
 * @organization Purdue ROV
 */

#ifndef PMBUS_BRICK_H
#define PMBUS_BRICK_H

#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t pmbus_addr;
    float input_voltage_v;
    float output_voltage_v;
    float output_current_a;
    float temperature_c;
    uint16_t status_word;
} pmbus_brick_dev_t;

rov_status_t pmbus_brick_init(pmbus_brick_dev_t *dev, uint8_t pmbus_addr);
rov_status_t pmbus_brick_read_telemetry(pmbus_brick_dev_t *dev);
float pmbus_linear11_to_float(uint16_t raw_value);

#ifdef __cplusplus
}
#endif

#endif /* PMBUS_BRICK_H */
