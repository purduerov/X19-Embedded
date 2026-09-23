/**
 * @file bme280.h
 * @brief Bosch BME280 Enclosure Pressure, Humidity & Temperature Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef BME280_H
#define BME280_H

#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_calib_t; // facotry coefficients 

typedef struct {
    uint8_t i2c_addr; // 0x76 or 0x77 
    bme280_calib_t calib; 
    uint32_t t_fine; // temperature compensation produces an intermediate value 

    float pressure_hpa;
    float humidity_pct;
    float temperature_c;

    bool initialized; 
} bme280_dev_t;

rov_status_t bme280_init(bme280_dev_t *dev);
rov_status_t bme280_read_all(bme280_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H */
