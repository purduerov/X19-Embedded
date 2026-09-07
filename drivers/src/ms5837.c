/**
 * @file ms5837.c
 * @brief TE Connectivity MS5837-30BA Hydrostatic Pressure & Depth Driver Implementation.
 * @organization Purdue ROV
 */

#include "ms5837.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_ms5837(float *pressure_mbar, float *temp_c) {
    (void)pressure_mbar;
    (void)temp_c;
    return false;
}

rov_status_t ms5837_init(ms5837_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(ms5837_dev_t));
    dev->cached_fluid_density = -1.0f; /* Force calculation on first run */
    return ROV_OK;
}

rov_status_t ms5837_read_pressure_depth(ms5837_dev_t *dev, float fluid_density_kg_m3) {
    if (!dev || fluid_density_kg_m3 <= 0.0f)
        return ROV_ERR_INVALID_ARG;

    if (mock_sensors_get_ms5837(&dev->pressure_mbar, &dev->temperature_c)) {
        /* Atmospheric baseline: 1013.25 mbar (101325 Pa) */
        float delta_p_pa = (dev->pressure_mbar * 100.0f) - 101325.0f;
        if (delta_p_pa <= 0.0f) {
            dev->depth_meters = 0.0f;
        } else {
            /* Performance optimization: FPU division is slow (~14 cycles).
               Cache and precompute the inverse of the divisor to use multiplication instead. */
            if (dev->cached_fluid_density != fluid_density_kg_m3) {
                dev->cached_fluid_density = fluid_density_kg_m3;
                dev->inv_rho_g = 1.0f / (fluid_density_kg_m3 * 9.80665f);
            }
            /* Hydrostatic formula: depth = Delta_P * (1 / (rho * g)) */
            dev->depth_meters = delta_p_pa * dev->inv_rho_g;
        }
        return ROV_OK;
    }

    return ROV_OK;
}
