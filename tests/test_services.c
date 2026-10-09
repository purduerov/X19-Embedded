/**
 * @file test_services.c
 * @brief Unit tests for domain services (env_service, safety_service, actuator_service).
 */

#include "actuator_service.h"
#include "env_service.h"
#include "mock_bsp.h"
#include "power_service.h"
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

static void test_power_service(void) {
    mock_bsp_reset();
    mock_bsp_set_pcb_temperature_c(42.5f);
    assert(fabsf(power_get_pcb_temperature_c() - 42.5f) < 0.01f);

    mock_bsp_set_lm74700_ok(true);
    assert(power_lm74700_is_ok());

    mock_bsp_set_logic_voltage_mv(5150);
    assert(power_get_logic_voltage_mv() == 5150);

    power_bricks_disable_all();
    for (uint8_t i = 0; i < 4; i++) {
        assert(!mock_bsp_is_power_brick_enabled(i));
    }

    printf("  [PASS] Power service monitoring and brick shutdown\n");
}

static void test_env_service(void) {
    /* Test env probe returns false when mock I2C has no device */
    assert(!env_probe_i2c(0x76));
    printf("  [PASS] Environmental service initialization and probing\n");
}

int main(void) {
    printf("=== Running Domain Services Tests ===\n");
    test_actuator_service_clamping();
    test_safety_service_tripping();
    test_power_service();
    test_env_service();
    printf("=== All Domain Services Tests Passed ===\n");
    return 0;
}
