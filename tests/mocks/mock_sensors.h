/**
 * @file mock_sensors.h
 * @brief Synthetic Sensor Injection for Software-in-the-Loop Host Testing.
 * @organization Purdue ROV
 */

#ifndef MOCK_SENSORS_H
#define MOCK_SENSORS_H

#include "bme280.h"
#include "bmi270.h"
#include "ina226.h"
#include "ina237.h"
#include "lsm6dsoxtr.h"
#include "ms5837.h"
#include "pmbus_brick.h"
#include "tmp1075.h"
#include "tps25990.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset all simulated sensor registers to nominal atmospheric values.
 */
void mock_sensors_reset(void);

/**
 * @brief Inject Bosch BME280 enclosure pressure, humidity, and temperature.
 */
void mock_sensors_set_bme280(float pressure_hpa, float humidity_pct, float temp_c);
bool mock_sensors_get_bme280(float *pressure_hpa, float *humidity_pct, float *temp_c);

/**
 * @brief Inject MS5837-30BA hydrostatic pressure and temperature.
 */
void mock_sensors_set_ms5837(float pressure_mbar, float temp_c);
bool mock_sensors_get_ms5837(float *pressure_mbar, float *temp_c);

/**
 * @brief IMU telemetry data structure for mock sensors.
 */
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

/**
 * @brief Inject LSM6DSOXTR / BMI270 6-axis IMU quaternion, gyro rates, and status.
 */
void mock_sensors_set_imu(const imu_data_t *data);
bool mock_sensors_get_imu(imu_data_t *data);

/**
 * @brief Inject INA226 / INA237 bus voltage and current.
 */
void mock_sensors_set_ina226(float voltage_v, float current_a);
bool mock_sensors_get_ina226(float *voltage_v, float *current_a);

/**
 * @brief Inject TPS25990 PMBus converter brick telemetry.
 */
void mock_sensors_set_tps25990(uint8_t brick_idx, float v_in, float v_out, float i_out, float temp_c, uint16_t status);
bool mock_sensors_get_tps25990(uint8_t brick_idx, float *v_in, float *v_out, float *i_out, float *temp_c,
                               uint16_t *status);

/**
 * @brief Inject TMP1075 PCB temperature.
 */
void mock_sensors_set_tmp1075(float temp_c);
bool mock_sensors_get_tmp1075(float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_SENSORS_H */
