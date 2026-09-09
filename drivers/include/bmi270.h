/**
 * @file bmi270.h
 * @brief Bosch BMI270 6-Axis IMU SPI Driver & Orientation Estimator Interface.
 * @organization Purdue ROV
 */

#ifndef BMI270_H
#define BMI270_H

#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float q_w;
    float q_x;
    float q_y;
    float q_z;
    uint8_t status_flags;
} bmi270_dev_t;

rov_status_t bmi270_init(bmi270_dev_t *dev);
rov_status_t bmi270_read_raw(bmi270_dev_t *dev);
rov_status_t bmi270_update_attitude(bmi270_dev_t *dev, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_H */
