/**
 * @file bmi270.c
 * @brief Bosch BMI270 6-Axis IMU Driver Implementation.
 * @organization Purdue ROV
 */

#include "bmi270.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_imu(imu_data_t *imu_data) {
    (void)imu_data;
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

    imu_data_t imu_data = {0};
    if (mock_sensors_get_imu(&imu_data)) {
        dev->q_w = imu_data.qw;
        dev->q_x = imu_data.qx;
        dev->q_y = imu_data.qy;
        dev->q_z = imu_data.qz;
        dev->gyro_x_dps = imu_data.gx_dps;
        dev->gyro_y_dps = imu_data.gy_dps;
        dev->gyro_z_dps = imu_data.gz_dps;
        dev->status_flags = imu_data.status;
        return ROV_OK;
    }
    return ROV_OK;
}

rov_status_t bmi270_update_attitude(bmi270_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    return bmi270_read_raw(dev);
}
