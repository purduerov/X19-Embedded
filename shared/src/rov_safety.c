/**
 * @file rov_safety.c
 * @brief Safety monitor, watchdog tracking, and emergency break trigger logic.
 * @organization Purdue ROV
 */

#include "rov_safety.h"
#include "rov_parameters.h"

void rov_safety_init(rov_safety_state_t *state) {
    if (!state)
        return;
    state->emergency_break_active = false;
    state->leak_detected = false;
    state->watchdog_expired = false;
    state->overtemperature_tripped = false;
    state->last_heartbeat_timestamp_ms = 0;
}

void rov_safety_feed_heartbeat(rov_safety_state_t *state, uint32_t current_time_ms) {
    if (!state)
        return;
    state->last_heartbeat_timestamp_ms = current_time_ms;
}

bool rov_safety_is_heartbeat_lost(const rov_safety_state_t *state, uint32_t current_time_ms) {
    if (!state)
        return true;
    if (state->last_heartbeat_timestamp_ms == 0)
        return false; /* Uninitialized */
    return (current_time_ms - state->last_heartbeat_timestamp_ms) > ROV_HEARTBEAT_TIMEOUT_MS;
}

void rov_safety_trigger_emergency_break(rov_safety_state_t *state) {
    if (!state)
        return;
    state->emergency_break_active = true;
}

/* ========================================================================== */
/* Backward Compatibility Export Symbols                                      */
/* ========================================================================== */
#undef x19_safety_init
#undef x19_safety_feed_heartbeat
#undef x19_safety_is_heartbeat_lost
#undef x19_safety_trigger_emergency_break

void x19_safety_init(rov_safety_state_t *state) {
    rov_safety_init(state);
}

void x19_safety_feed_heartbeat(rov_safety_state_t *state, uint32_t current_time_ms) {
    rov_safety_feed_heartbeat(state, current_time_ms);
}

bool x19_safety_is_heartbeat_lost(const rov_safety_state_t *state, uint32_t current_time_ms) {
    return rov_safety_is_heartbeat_lost(state, current_time_ms);
}

void x19_safety_trigger_emergency_break(rov_safety_state_t *state) {
    rov_safety_trigger_emergency_break(state);
}
