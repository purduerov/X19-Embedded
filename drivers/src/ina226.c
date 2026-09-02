/**
 * @file ina226.c
 * @brief TI INA226 Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina226.h"
#include <string.h>

x19_status_t ina226_init(ina226_dev_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (!dev || shunt_resistor_ohms <= 0.0f)
        return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(ina226_dev_t));
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    return X19_OK;
}

x19_status_t ina226_read_power(ina226_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    return X19_OK;
}
