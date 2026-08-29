/**
 * @file lsm6dsoxtr.h
 * @brief ST LSM6DSOXTR 6-Axis IMU SPI Driver & Madgwick Filter Interface.
 * @organization Purdue ROV
 */

#ifndef LSM6DSOXTR_H
#define LSM6DSOXTR_H

#include "x19_types.h"
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
} lsm6dsoxtr_dev_t;

x19_status_t lsm6dsoxtr_init(lsm6dsoxtr_dev_t *dev);
x19_status_t lsm6dsoxtr_read_raw(lsm6dsoxtr_dev_t *dev);
x19_status_t lsm6dsoxtr_update_madgwick(lsm6dsoxtr_dev_t *dev, float dt_sec);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSOXTR_H */
