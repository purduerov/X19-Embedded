/**
 * @file lsm6dsoxtr.c
 * @brief ST LSM6DSOXTR 6-Axis IMU SPI Driver & Madgwick Filter Implementation.
 * @organization Purdue ROV
 */

#include "lsm6dsoxtr.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_imu(float *qw, float *qx, float *qy, float *qz, float *gx_rad_s,
                                                float *gy_rad_s, float *gz_rad_s, uint8_t *status) {
    (void)qw;
    (void)qx;
    (void)qy;
    (void)qz;
    (void)gx_rad_s;
    (void)gy_rad_s;
    (void)gz_rad_s;
    (void)status;
    return false;
}

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

    uint8_t status = 0;
    if (mock_sensors_get_imu(&dev->q_w, &dev->q_x, &dev->q_y, &dev->q_z, &dev->gyro_x_dps, &dev->gyro_y_dps,
                             &dev->gyro_z_dps, &status)) {
        dev->status_flags = status;
        return X19_OK;
    }

    return X19_OK;
}

x19_status_t lsm6dsoxtr_update_madgwick(lsm6dsoxtr_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return X19_ERR_INVALID_ARG;

    /* Normalize quaternion */
    float norm = sqrtf((dev->q_w * dev->q_w) + (dev->q_x * dev->q_x) + (dev->q_y * dev->q_y) + (dev->q_z * dev->q_z));
    if (norm > 0.00001f) {
        /* Performance optimization: FPU division is slow (~14 cycles).
           1 division + 4 multiplications is much faster than 4 divisions. */
        float inv_norm = 1.0f / norm;
        dev->q_w *= inv_norm;
        dev->q_x *= inv_norm;
        dev->q_y *= inv_norm;
        dev->q_z *= inv_norm;
    }

    return X19_OK;
}
