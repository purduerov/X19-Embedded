/**
 * @file x19_safety.h
 * @brief Safety monitor macros, watchdog limits, and emergency break trigger interfaces.
 * @organization Purdue ROV
 */

#ifndef X19_SAFETY_H
#define X19_SAFETY_H

#include "x19_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define X19_LEAK_PRESSURE_DROP_THRESHOLD_HPA (15.0f)
#define X19_LEAK_HUMIDITY_MAX_PCT            (80.0f)
#define X19_PCB_MAX_SAFE_TEMP_C              (85.0f)

typedef struct {
    bool emergency_break_active;
    bool leak_detected;
    bool watchdog_expired;
    bool overtemperature_tripped;
    uint32_t last_heartbeat_timestamp_ms;
} x19_safety_state_t;

void x19_safety_init(x19_safety_state_t *state);
void x19_safety_feed_heartbeat(x19_safety_state_t *state, uint32_t current_time_ms);
bool x19_safety_is_heartbeat_lost(const x19_safety_state_t *state, uint32_t current_time_ms);
void x19_safety_trigger_emergency_break(x19_safety_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* X19_SAFETY_H */
