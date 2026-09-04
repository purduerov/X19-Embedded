/**
 * @file x19_safety.c
 * @brief Safety monitor, watchdog tracking, and emergency break trigger logic.
 * @organization Purdue ROV
 */

#include "x19_safety.h"
#include "x19_parameters.h"

void x19_safety_init(x19_safety_state_t *state) {
    if (!state)
        return;
    state->emergency_break_active = false;
    state->leak_detected = false;
    state->watchdog_expired = false;
    state->overtemperature_tripped = false;
    state->last_heartbeat_timestamp_ms = 0;
}

void x19_safety_feed_heartbeat(x19_safety_state_t *state, uint32_t current_time_ms) {
    if (!state)
        return;
    state->last_heartbeat_timestamp_ms = current_time_ms;
}

bool x19_safety_is_heartbeat_lost(const x19_safety_state_t *state, uint32_t current_time_ms) {
    if (!state)
        return true;
    if (state->last_heartbeat_timestamp_ms == 0)
        return false; /* Uninitialized */
    return (current_time_ms - state->last_heartbeat_timestamp_ms) > X19_HEARTBEAT_TIMEOUT_MS;
}

void x19_safety_trigger_emergency_break(x19_safety_state_t *state) {
    if (!state)
        return;
    state->emergency_break_active = true;
}
