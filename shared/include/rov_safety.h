/**
 * @file rov_safety.h
 * @brief Safety monitor macros, watchdog limits, and emergency break trigger interfaces.
 * @organization Purdue ROV
 */

#ifndef ROV_SAFETY_H
#define ROV_SAFETY_H

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROV_LEAK_PRESSURE_DROP_THRESHOLD_HPA (15.0f)
#define ROV_LEAK_HUMIDITY_MAX_PCT            (80.0f)
#define ROV_PCB_MAX_SAFE_TEMP_C              (85.0f)

typedef struct {
    bool emergency_break_active;
    bool leak_detected;
    bool watchdog_expired;
    bool overtemperature_tripped;
    uint32_t last_heartbeat_timestamp_ms;
} rov_safety_state_t;

void rov_safety_init(rov_safety_state_t *state);
void rov_safety_feed_heartbeat(rov_safety_state_t *state, uint32_t current_time_ms);
bool rov_safety_is_heartbeat_lost(const rov_safety_state_t *state, uint32_t current_time_ms);
void rov_safety_trigger_emergency_break(rov_safety_state_t *state);

/* ========================================================================== */
/* BACKWARD COMPATIBILITY ALIASES (X19 Vehicle Profile)                       */
/* ========================================================================== */
#define X19_LEAK_PRESSURE_DROP_THRESHOLD_HPA ROV_LEAK_PRESSURE_DROP_THRESHOLD_HPA
#define X19_LEAK_HUMIDITY_MAX_PCT            ROV_LEAK_HUMIDITY_MAX_PCT
#define X19_PCB_MAX_SAFE_TEMP_C              ROV_PCB_MAX_SAFE_TEMP_C

typedef rov_safety_state_t x19_safety_state_t;

#define x19_safety_init                    rov_safety_init
#define x19_safety_feed_heartbeat          rov_safety_feed_heartbeat
#define x19_safety_is_heartbeat_lost       rov_safety_is_heartbeat_lost
#define x19_safety_trigger_emergency_break rov_safety_trigger_emergency_break

#ifdef __cplusplus
}
#endif

#endif /* ROV_SAFETY_H */
