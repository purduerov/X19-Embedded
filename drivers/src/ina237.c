/**
 * @file ina237.c
 * @brief TI INA237AIDGSR Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina237.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    (void)voltage_v;
    (void)current_a;
    return false;
}

rov_status_t ina237_init(ina237_dev_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (!dev || shunt_resistor_ohms <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(ina237_dev_t));
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    return ROV_OK;
}

rov_status_t ina237_read_power(ina237_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    if (mock_sensors_get_ina226(&dev->bus_voltage_v, &dev->shunt_current_a)) {
        dev->power_w = dev->bus_voltage_v * dev->shunt_current_a;
        return ROV_OK;
    }
    return ROV_OK;
}
