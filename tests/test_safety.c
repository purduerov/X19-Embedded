#include "x19_safety.h"
#include "x19_parameters.h"
#include <stdio.h>
#include <assert.h>

void test_safety_initialization(void) {
    x19_safety_state_t state;
    x19_safety_init(&state);

    assert(state.emergency_break_active == false);
    assert(state.leak_detected == false);
    assert(state.watchdog_expired == false);
    assert(state.overtemperature_tripped == false);
    printf("[PASS] test_safety_initialization\n");
}

void test_heartbeat_watchdog(void) {
    x19_safety_state_t state;
    x19_safety_init(&state);

    uint32_t current_time_ms = 1000;
    x19_safety_feed_heartbeat(&state, current_time_ms);

    // 50ms later -> heartbeat valid
    assert(x19_safety_is_heartbeat_lost(&state, 1050) == false);

    // 150ms later -> heartbeat expired (> 100ms)
    assert(x19_safety_is_heartbeat_lost(&state, 1150) == true);
    printf("[PASS] test_heartbeat_watchdog\n");
}

void test_emergency_break_trigger(void) {
    x19_safety_state_t state;
    x19_safety_init(&state);

    x19_safety_trigger_emergency_break(&state);
    assert(state.emergency_break_active == true);
    printf("[PASS] test_emergency_break_trigger\n");
}

int main(void) {
    printf("Running Safety Subsystem Unit Tests...\n");
    test_safety_initialization();
    test_heartbeat_watchdog();
    test_emergency_break_trigger();
    printf("All Safety Tests Passed Successfully!\n");
    return 0;
}
