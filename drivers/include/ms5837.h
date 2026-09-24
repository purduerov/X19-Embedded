/**
 * @file ms5837.h
 * @brief TE Connectivity MS5837-30BA Hydrostatic Pressure & Depth Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef MS5837_H
#define MS5837_H

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { MS5837_STATE_IDLE = 0, MS5837_STATE_WAIT_D1, MS5837_STATE_WAIT_D2 } ms5837_state_t;

typedef struct {
    float pressure_mbar;
    float temperature_c;
    float depth_meters;

    uint16_t cal_coeffs[8];

    uint32_t raw_pressure;
    uint32_t raw_temperature;
    uint32_t conversion_start_ms;

    ms5837_state_t state;
    bool initialized;

    float cached_fluid_density;
    float inv_rho_g;
} ms5837_dev_t;

rov_status_t ms5837_init(ms5837_dev_t *dev);
rov_status_t ms5837_read_pressure_depth(ms5837_dev_t *dev, float fluid_density_kg_m3);

#ifdef __cplusplus
}
#endif

#endif /* MS5837_H */