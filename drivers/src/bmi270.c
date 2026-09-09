/**
 * @file bmi270.c
 * @brief Bosch BMI270 6-Axis IMU Driver Implementation.
 * @organization Purdue ROV
 */

#include "bmi270.h"
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

rov_status_t bmi270_update_attitude(bmi270_dev_t *dev, float dt_sec) {
    if (!dev || dt_sec <= 0.0f)
        return ROV_ERR_INVALID_ARG;
    return bmi270_read_raw(dev);
}
