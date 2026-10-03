/**
 * @file safety_service.c
 * @brief Implementation of vehicle safety, leak detection, and emergency trip service.
 */

#include "safety_service.h"
#include "actuator_service.h"
#include "bsp.h"

static bool g_emergency_tripped = false;

void safety_service_init(void) {
    g_emergency_tripped = false;
}

bool leak_probe_is_wet(uint8_t probe_idx) {
    return bsp_leak_probe_read(probe_idx);
}

void safety_emergency_trip(void) {
    g_emergency_tripped = true;
    bsp_emergency_brake_trip();
    actuators_stop_all();
}

bool safety_is_tripped(void) {
    return g_emergency_tripped || bsp_is_emergency_brake_tripped();
}

bool safety_emergency_reset(void) {
    /* If either physical probe is still wet, refuse to reset */
    if (leak_probe_is_wet(0) || leak_probe_is_wet(1)) {
        return false;
    }
    g_emergency_tripped = false;
    return true;
}
