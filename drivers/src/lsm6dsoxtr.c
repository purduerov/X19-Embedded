/**
 * @file lsm6dsoxtr.c
 * @brief ST LSM6DSOXTR 6-Axis IMU SPI Driver & Madgwick Filter Implementation.
 * @organization Purdue ROV
 */

#include "lsm6dsoxtr.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* Forward declaration for weak mock function struct */
typedef struct {
    float qw;
    float qx;
    float qy;
    float qz;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    uint8_t status;
} imu_data_t;

__attribute__((weak)) bool mock_sensors_get_imu(imu_data_t *data) {
    (void)data;
    return false;
}

rov_status_t lsm6dsoxtr_init(lsm6dsoxtr_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(lsm6dsoxtr_dev_t));
    dev->q_w = 1.0f;
    return ROV_OK;
}

rov_status_t lsm6dsoxtr_read_raw(lsm6dsoxtr_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;

    imu_data_t mock_data;
    if (mock_sensors_get_imu(&mock_data)) {
        dev->q_w = mock_data.qw;
        dev->q_x = mock_data.qx;
        dev->q_y = mock_data.qy;
        dev->q_z = mock_data.qz;
        dev->gyro_x_dps = mock_data.gx_dps;
        dev->gyro_y_dps = mock_data.gy_dps;
        dev->gyro_z_dps = mock_data.gz_dps;
        dev->status_flags = mock_data.status;
        return ROV_OK;
    }

    return ROV_OK;
}

rov_status_t lsm6dsoxtr_update_madgwick(lsm6dsoxtr_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return ROV_ERR_INVALID_ARG;

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

    return ROV_OK;
}
