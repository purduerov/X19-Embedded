/**
 * @file ina226.h
 * @brief TI INA226 / INA219 Power & Shunt Current Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA226_H
#define INA226_H

#include "x19_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float voltage_v;
    float current_a;
    float power_w;
} ina226_dev_t;

x19_status_t ina226_init(ina226_dev_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms);
x19_status_t ina226_read_power(ina226_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
