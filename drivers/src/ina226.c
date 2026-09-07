/**
 * @file ina226.c
 * @brief TI INA226 Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina226.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    (void)voltage_v;
    (void)current_a;
    return false;
}

rov_status_t ina226_init(ina226_dev_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (!dev || shunt_resistor_ohms <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(ina226_dev_t));
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    return ROV_OK;
}

rov_status_t ina226_read_power(ina226_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;

    if (mock_sensors_get_ina226(&dev->voltage_v, &dev->current_a)) {
        dev->power_w = dev->voltage_v * dev->current_a;
        return ROV_OK;
    }

    return ROV_OK;
}
