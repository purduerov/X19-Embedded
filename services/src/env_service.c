/**
 * @file env_service.c
 * @brief Implementation of environmental sensor service.
 */

#include "env_service.h"
#include "bme280.h"
#include <string.h>

static bme280_dev_t g_bme_dev;
static bool g_bme_initialized = false;
static float g_cached_temp_c = 0.0f;
static float g_cached_press_hpa = 0.0f;
static float g_cached_hum_pct = 0.0f;

rov_status_t env_service_init(void) {
    memset(&g_bme_dev, 0, sizeof(g_bme_dev));
    rov_status_t status = bme280_init(&g_bme_dev);
    if (status == ROV_OK) {
        g_bme_initialized = true;
    }
    return status;
}

rov_status_t env_read_all(float *temp_c, float *press_hpa, float *hum_pct) {
    if (!g_bme_initialized) {
        rov_status_t init_status = env_service_init();
        if (init_status != ROV_OK) {
            return init_status;
        }
    }

    rov_status_t status = bme280_read_all(&g_bme_dev);
    if (status == ROV_OK) {
        g_cached_temp_c = g_bme_dev.temperature_c;
        g_cached_press_hpa = g_bme_dev.pressure_hpa;
        g_cached_hum_pct = g_bme_dev.humidity_pct;

        if (temp_c != NULL) {
            *temp_c = g_cached_temp_c;
        }
        if (press_hpa != NULL) {
            *press_hpa = g_cached_press_hpa;
        }
        if (hum_pct != NULL) {
            *hum_pct = g_cached_hum_pct;
        }
    }

    return status;
}

float env_get_temperature(void) {
    return g_cached_temp_c;
}

float env_get_pressure(void) {
    return g_cached_press_hpa;
}

float env_get_humidity(void) {
    return g_cached_hum_pct;
}
