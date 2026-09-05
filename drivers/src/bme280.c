/**
 * @file bme280.c
 * @brief Bosch BME280 Environmental Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include <string.h>

x19_status_t bme280_init(bme280_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(bme280_dev_t));
    return X19_OK;
}

x19_status_t bme280_read_all(bme280_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    return X19_OK;
}
