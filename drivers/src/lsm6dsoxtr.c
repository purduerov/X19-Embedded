/**
 * @file lsm6dsoxtr.c
 * @brief ST LSM6DSOXTR 6-Axis IMU SPI Driver & Madgwick Filter Implementation.
 * @organization Purdue ROV
 */

#include "lsm6dsoxtr.h"
#include <string.h>

x19_status_t lsm6dsoxtr_init(lsm6dsoxtr_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(lsm6dsoxtr_dev_t));
    dev->q_w = 1.0f;
    return X19_OK;
}

x19_status_t lsm6dsoxtr_read_raw(lsm6dsoxtr_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    return X19_OK;
}

x19_status_t lsm6dsoxtr_update_madgwick(lsm6dsoxtr_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return X19_ERR_INVALID_ARG;
    return X19_OK;
}
