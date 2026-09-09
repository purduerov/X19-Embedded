/**
 * @file ina237.h
 * @brief TI INA237AIDGSR High-Precision Power Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA237_H
#define INA237_H

#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float bus_voltage_v;
    float shunt_current_a;
    float power_w;
    float die_temp_c;
} ina237_dev_t;

rov_status_t ina237_init(ina237_dev_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms);
rov_status_t ina237_read_power(ina237_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA237_H */
