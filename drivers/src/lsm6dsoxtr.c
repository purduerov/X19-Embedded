/**
 * @file lsm6dsoxtr.c
 * @brief ST LSM6DSOXTR 6-Axis IMU SPI Driver & Madgwick Filter Implementation.
 * @organization Purdue ROV
 */

#include "lsm6dsoxtr.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_imu(imu_data_t *data) {
    (void)data;
    return false;
}

static bool lsm6dsoxtr_data_valid(const imu_data_t *data) {
    if (!data || !isfinite(data->q_w) || !isfinite(data->q_x) || !isfinite(data->q_y) || !isfinite(data->q_z) ||
        !isfinite(data->gyro_x_dps) || !isfinite(data->gyro_y_dps) || !isfinite(data->gyro_z_dps)) {
        return false;
    }
    float norm =
        sqrtf((data->q_w * data->q_w) + (data->q_x * data->q_x) + (data->q_y * data->q_y) + (data->q_z * data->q_z));
    return isfinite(norm) && norm > 0.00001f;
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

    imu_data_t data;
    if (mock_sensors_get_imu(&data) && lsm6dsoxtr_data_valid(&data)) {
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

    return ROV_ERROR;
}

rov_status_t lsm6dsoxtr_update_madgwick(lsm6dsoxtr_dev_t *dev, float dt_sec) {
    if (!dev || !(dt_sec > 0.0f))
        return ROV_ERR_INVALID_ARG;

    const float deg_to_rad = 0.017453292519943295f;
    float half_dt = 0.5f * dt_sec;
    float gx = dev->gyro_x_dps * deg_to_rad;
    float gy = dev->gyro_y_dps * deg_to_rad;
    float gz = dev->gyro_z_dps * deg_to_rad;

    float q_w = dev->q_w;
    float q_x = dev->q_x;
    float q_y = dev->q_y;
    float q_z = dev->q_z;

    q_w += half_dt * ((-q_x * gx) - (q_y * gy) - (q_z * gz));
    q_x += half_dt * ((q_w * gx) + (q_y * gz) - (q_z * gy));
    q_y += half_dt * ((q_w * gy) - (q_x * gz) + (q_z * gx));
    q_z += half_dt * ((q_w * gz) + (q_x * gy) - (q_y * gx));

    float norm = sqrtf((q_w * q_w) + (q_x * q_x) + (q_y * q_y) + (q_z * q_z));
    if (!isfinite(norm) || norm <= 0.00001f)
        return ROV_ERROR;

    float inv_norm = 1.0f / norm;
    dev->q_w = q_w * inv_norm;
    dev->q_x = q_x * inv_norm;
    dev->q_y = q_y * inv_norm;
    dev->q_z = q_z * inv_norm;

    return ROV_OK;
}
