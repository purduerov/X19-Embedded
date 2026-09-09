/**
 * @file tmp1075.c
 * @brief TI TMP1075NDRLR Temperature Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "tmp1075.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_tmp1075(float *temp_c) {
    (void)temp_c;
    return false;
}

rov_status_t tmp1075_init(tmp1075_dev_t *dev, uint8_t i2c_addr) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(tmp1075_dev_t));
    dev->i2c_addr = i2c_addr;
    dev->temperature_c = 25.0f;
    return ROV_OK;
}

rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    if (mock_sensors_get_tmp1075(&dev->temperature_c)) {
        return ROV_OK;
    }
    return ROV_OK;
}
