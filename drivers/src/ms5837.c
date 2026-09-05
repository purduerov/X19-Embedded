/**
 * @file ms5837.c
 * @brief TE Connectivity MS5837-30BA Hydrostatic Pressure & Depth Driver Implementation.
 * @organization Purdue ROV
 */

#include "ms5837.h"
#include <string.h>

x19_status_t ms5837_init(ms5837_dev_t *dev) {
    if (!dev) return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(ms5837_dev_t));
    return X19_OK;
}

x19_status_t ms5837_read_pressure_depth(ms5837_dev_t *dev, float fluid_density_kg_m3) {
    if (!dev || fluid_density_kg_m3 <= 0.0f) return X19_ERR_INVALID_ARG;
    return X19_OK;
}
