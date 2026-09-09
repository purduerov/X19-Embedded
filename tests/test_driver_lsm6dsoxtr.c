/**
 * @file test_driver_lsm6dsoxtr.c
 * @brief Unit tests for ST LSM6DSOXTR 6-Axis IMU Driver.
 * @organization Purdue ROV
 */

#include "lsm6dsoxtr.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

void test_lsm6dsoxtr_driver(void) {
    lsm6dsoxtr_dev_t dev;

    /* Negative tests */
    assert(lsm6dsoxtr_init(NULL) == ROV_ERR_INVALID_ARG);
    assert(lsm6dsoxtr_read_raw(NULL) == ROV_ERR_INVALID_ARG);
    assert(lsm6dsoxtr_update_madgwick(NULL, 0.01f) == ROV_ERR_INVALID_ARG);
    assert(lsm6dsoxtr_init(&dev) == ROV_OK);
    assert(lsm6dsoxtr_update_madgwick(&dev, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(lsm6dsoxtr_update_madgwick(&dev, -0.01f) == ROV_ERR_INVALID_ARG);

    /* Initialization */
    assert(dev.q_w == 1.0f);
    assert(dev.q_x == 0.0f);

    /* Test mock sensor reading */
    mock_sensors_reset();

    imu_data_t mock_imu = {.qw = 0.7071f,
                           .qx = 0.0f,
                           .qy = 0.7071f,
                           .qz = 0.0f,
                           .gx_dps = 0.05f,
                           .gy_dps = -0.02f,
                           .gz_dps = 0.12f,
                           .status = 3};
    mock_sensors_set_imu(&mock_imu);

    assert(lsm6dsoxtr_read_raw(&dev) == ROV_OK);
    assert(dev.q_w > 0.70f && dev.q_w < 0.71f);
    assert(dev.gyro_z_dps > 0.11f && dev.gyro_z_dps < 0.13f);

    /* Test quaternion normalization */
    dev.q_w = 2.0f;
    dev.q_x = 0.0f;
    dev.q_y = 2.0f;
    dev.q_z = 0.0f;
    assert(lsm6dsoxtr_update_madgwick(&dev, 0.01f) == ROV_OK);
    float norm = sqrtf((dev.q_w * dev.q_w) + (dev.q_x * dev.q_x) + (dev.q_y * dev.q_y) + (dev.q_z * dev.q_z));
    assert(norm > 0.999f && norm < 1.001f);

    printf("[PASS] test_lsm6dsoxtr_driver\n");
}

int main(void) {
    printf("Running LSM6DSOXTR Driver Unit Tests...\n");
    test_lsm6dsoxtr_driver();
    printf("All LSM6DSOXTR Driver Tests Passed Successfully!\n");
    return 0;
}
