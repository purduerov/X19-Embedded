/**
 * @file power_service.c
 * @brief Implementation of power conversion and monitoring domain service.
 * @author Purdue ROV Embedded Team
 */

#include "power_service.h"
#include "bsp.h"
#include "ina237.h"
#include <string.h>

/*
 * Node 1 Pi Shield 5V rail monitor. The service owns the device config so the
 * application never names a driver, an address, or a bus. Per-board rail
 * configuration is deferred to the node 3 power sub-project.
 */
#define POWER_RAIL_INA237_ADDR (0x40u)
#define POWER_RAIL_SHUNT_OHMS  (0.001f)

static ina237_dev_t g_rail_dev;
static bool g_rail_initialized = false;

rov_status_t power_service_init(void) {
    memset(&g_rail_dev, 0, sizeof(g_rail_dev));

    rov_status_t status = ina237_init(&g_rail_dev, bsp_i2c_bus(), POWER_RAIL_INA237_ADDR, POWER_RAIL_SHUNT_OHMS);
    g_rail_initialized = (status == ROV_OK);
    return status;
}

rov_status_t power_read_rail(float *bus_voltage_v, float *current_a) {
    if (!g_rail_initialized) {
        rov_status_t init_status = power_service_init();
        if (init_status != ROV_OK) {
            return init_status;
        }
    }

    rov_status_t status = ina237_read_power(&g_rail_dev);
    if (status == ROV_OK) {
        if (bus_voltage_v != NULL) {
            *bus_voltage_v = g_rail_dev.bus_voltage_v;
        }
        if (current_a != NULL) {
            *current_a = g_rail_dev.shunt_current_a;
        }
    }

    return status;
}

void power_bricks_disable_all(void) {
    bsp_power_brick_disable_all();
}

float power_get_pcb_temperature_c(void) {
    return bsp_get_pcb_temperature_c();
}

bool power_lm74700_is_ok(void) {
    return bsp_lm74700_status_ok();
}

uint16_t power_get_logic_voltage_mv(void) {
    return bsp_get_logic_voltage_mv();
}
