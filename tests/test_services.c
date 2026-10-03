/**
 * @file test_services.c
 * @brief Unit tests for domain services (env_service, safety_service, actuator_service).
 */

#include "actuator_service.h"
#include "env_service.h"
#include "mock_bsp.h"
#include "safety_service.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_actuator_service_clamping(void) {
    actuator_service_init();

    /* Verify initial neutral state */
    for (uint8_t ch = 0; ch < 8; ch++) {
        assert(pwm_get_pulse_us(ch) == 1500);
    }

    /* Verify clamping below 1000 us */
    pwm_set_pulse_us(0, 800);
    assert(pwm_get_pulse_us(0) == 1000);

    /* Verify clamping above 2000 us */
    pwm_set_pulse_us(1, 2500);
    assert(pwm_get_pulse_us(1) == 2000);

    /* Verify nominal command */
    pwm_set_pulse_us(2, 1650);
    assert(pwm_get_pulse_us(2) == 1650);

    /* Verify stop all resets to 1500 */
    actuators_stop_all();
    assert(pwm_get_pulse_us(2) == 1500);

    /* Solenoids */
    solenoid_set_mask(0x01F);
    assert(solenoid_get_mask() == 0x01F);

    printf("  [PASS] Actuator service clamping and neutral safety\n");
}

static void test_safety_service_tripping(void) {
    safety_service_init();
    assert(!safety_is_tripped());

    safety_emergency_trip();
    assert(safety_is_tripped());

    /* While tripped, actuators should be neutralized */
    pwm_set_pulse_us(0, 1800);
    assert(pwm_get_pulse_us(0) == 1500);

    mock_bsp_reset();
    safety_emergency_reset();
    assert(!safety_is_tripped());

    printf("  [PASS] Safety service emergency trip and latch\n");
}

int main(void) {
    printf("=== Running Domain Services Tests ===\n");
    test_actuator_service_clamping();
    test_safety_service_tripping();
    printf("=== All Domain Services Tests Passed ===\n");
    return 0;
}
