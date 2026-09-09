/**
 * @file bmi270.c
 * @brief Bosch BMI270 6-Axis IMU Driver Implementation.
 * @organization Purdue ROV
 */

#include "bmi270.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_imu(imu_data_t *data) {
    (void)data;
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

    imu_data_t data;
    if (mock_sensors_get_imu(&data)) {
        dev->q_w = data.q_w;
        dev->q_x = data.q_x;
        dev->q_y = data.q_y;
        dev->q_z = data.q_z;
        dev->gyro_x_dps = data.gyro_x_dps;
        dev->gyro_y_dps = data.gyro_y_dps;
        dev->gyro_z_dps = data.gyro_z_dps;
        dev->status_flags = data.status;
        return ROV_OK;
    }
    return ROV_OK;
}

rov_status_t bmi270_update_attitude(bmi270_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    return bmi270_read_raw(dev);
}

