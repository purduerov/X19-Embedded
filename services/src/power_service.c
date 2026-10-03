/**
 * @file power_service.c
 * @brief Implementation of power conversion and monitoring domain service.
 * @author Purdue ROV Embedded Team
 */

#include "power_service.h"
#include "bsp.h"

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
