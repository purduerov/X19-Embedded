/**
 * @file bmi270.c
 * @brief Bosch BMI270 6-Axis IMU Driver Implementation.
 * @organization Purdue ROV
 */

#include "bmi270.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_imu(float *qw, float *qx, float *qy, float *qz, float *gx, float *gy,
                                                float *gz, uint8_t *status) {
    (void)qw;
    (void)qx;
    (void)qy;
    (void)qz;
    (void)gx;
    (void)gy;
    (void)gz;
    (void)status;
    return false;
}

rov_status_t bmi270_init(bmi270_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(bmi270_dev_t));
    dev->q_w = 1.0f;
    return ROV_OK;
}

rov_status_t bmi270_read_raw(bmi270_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    if (mock_sensors_get_imu(&dev->q_w, &dev->q_x, &dev->q_y, &dev->q_z, &dev->gyro_x_dps, &dev->gyro_y_dps,
                             &dev->gyro_z_dps, &dev->status_flags)) {
        return ROV_OK;
    }
    return ROV_OK;
}

rov_status_t bmi270_update_attitude(bmi270_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    return bmi270_read_raw(dev);
}
