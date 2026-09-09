/**
 * @file test_driver_bmi270.c
 * @brief Unit tests for Bosch BMI270 6-Axis IMU Driver.
 * @organization Purdue ROV
 */

#include "bmi270.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

void test_bmi270_driver(void) {
    bmi270_dev_t dev;

    /* Negative test: NULL pointer */
    assert(bmi270_init(NULL) == ROV_ERR_INVALID_ARG);
    assert(bmi270_read_raw(NULL) == ROV_ERR_INVALID_ARG);
    assert(bmi270_update_attitude(NULL, 0.01f) == ROV_ERR_INVALID_ARG);
    assert(bmi270_init(&dev) == ROV_OK);
    assert(bmi270_update_attitude(&dev, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(bmi270_update_attitude(&dev, -0.01f) == ROV_ERR_INVALID_ARG);

    /* Test mock IMU rate injection */
    mock_sensors_reset();
    imu_data_t imu_mock = {
        .q_w = 0.7071f,
        .q_x = 0.0f,
        .q_y = 0.7071f,
        .q_z = 0.0f,
        .gyro_x_dps = 10.0f,
        .gyro_y_dps = -5.0f,
        .gyro_z_dps = 45.0f,
        .status = 3
    };
    mock_sensors_set_imu(&imu_mock);

    assert(bmi270_read_raw(&dev) == ROV_OK);
    assert(dev.gyro_x_dps > 9.9f && dev.gyro_x_dps < 10.1f);
    assert(dev.gyro_y_dps > -5.1f && dev.gyro_y_dps < -4.9f);
    assert(dev.gyro_z_dps > 44.9f && dev.gyro_z_dps < 45.1f);
    assert(dev.status_flags == 3);

    /* Test attitude quaternion integration & unit normalization */
    assert(bmi270_update_attitude(&dev, 0.01f) == ROV_OK);
    float norm = sqrtf(dev.q_w * dev.q_w + dev.q_x * dev.q_x + dev.q_y * dev.q_y + dev.q_z * dev.q_z);
    assert(fabsf(norm - 1.0f) < 1e-4f);

    printf("[PASS] test_bmi270_driver\n");
}

int main(void) {
    printf("Running BMI270 Driver Unit Tests...\n");
    test_bmi270_driver();
    printf("All BMI270 Driver Tests Passed Successfully!\n");
    return 0;
}
