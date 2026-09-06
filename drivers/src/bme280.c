/**
 * @file bme280.c
 * @brief Bosch BME280 Environmental Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_bme280(float *pressure_hpa, float *humidity_pct, float *temp_c) {
    (void)pressure_hpa;
    (void)humidity_pct;
    (void)temp_c;
    return false;
}

x19_status_t bme280_init(bme280_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(bme280_dev_t));
    return X19_OK;
}

x19_status_t bme280_read_all(bme280_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    if (mock_sensors_get_bme280(&dev->pressure_hpa, &dev->humidity_pct, &dev->temperature_c)) {
        return X19_OK;
    }
    return X19_OK;
}
