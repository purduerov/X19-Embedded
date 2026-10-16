/**
 * @file env_service.c
 * @brief Implementation of environmental sensor service.
 */

#include "env_service.h"
#include "bme280.h"
#include "bsp.h"
#include <string.h>

static bme280_dev_t g_bme_dev;
static bool g_bme_initialized = false;

rov_status_t env_service_init(void) {
    memset(&g_bme_dev, 0, sizeof(g_bme_dev));
    rov_status_t status = bme280_init(&g_bme_dev, bsp_i2c_bus());
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
        if (temp_c != NULL) {
            *temp_c = g_bme_dev.temperature_c;
        }
        if (press_hpa != NULL) {
            *press_hpa = g_bme_dev.pressure_hpa;
        }
        if (hum_pct != NULL) {
            *hum_pct = g_bme_dev.humidity_pct;
        }
    }

    return status;
}

bool env_probe_i2c(uint8_t addr) {
    return bsp_i2c_probe(addr);
}
