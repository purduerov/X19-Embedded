/**
 * @file bme280.h
 * @brief Bosch BME280 Enclosure Pressure, Humidity & Temperature Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef BME280_H
#define BME280_H

#include "x19_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure_hpa;
    float humidity_pct;
    float temperature_c;
} bme280_dev_t;

x19_status_t bme280_init(bme280_dev_t *dev);
x19_status_t bme280_read_all(bme280_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H */
