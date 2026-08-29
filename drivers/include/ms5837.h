/**
 * @file ms5837.h
 * @brief TE Connectivity MS5837-30BA Hydrostatic Pressure & Depth Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef MS5837_H
#define MS5837_H

#include "x19_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure_mbar;
    float temperature_c;
    float depth_meters;
    uint16_t cal_coeffs[8];
} ms5837_dev_t;

x19_status_t ms5837_init(ms5837_dev_t *dev);
x19_status_t ms5837_read_pressure_depth(ms5837_dev_t *dev, float fluid_density_kg_m3);

#ifdef __cplusplus
}
#endif

#endif /* MS5837_H */
