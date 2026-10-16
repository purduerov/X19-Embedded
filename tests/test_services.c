/**
 * @file test_services.c
 * @brief Unit tests for domain services (env_service, safety_service, actuator_service).
 */

#include "actuator_service.h"
#include "env_service.h"
#include "mock_bme280.h"
#include "mock_bsp.h"
#include "mock_ina237.h"
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

/*
 * The service owns the BME280: it initializes the driver and returns the
 * compensated engineering units. Exercise the real register-image path (device
 * model -> driver -> service), not a value-level hook, so a wrong branch in
 * init, a wrong register, or wrong compensation all surface here.
 */
static void test_env_service(void) {
    float temp_c = 0.0f;
    float press_hpa = 0.0f;
    float hum_pct = 0.0f;

    /* No device on the bus: the probe is false and a read fails. */
    mock_bsp_reset();
    assert(!env_probe_i2c(0x76));
    assert(env_read_all(NULL, NULL, NULL) != ROV_OK);

    /* Put the sensor on the bus at a known reading. */
    mock_bme280_present(0x76u);
    mock_bme280_set_reading(0x76u, 1013.25f, 42.0f, 26.5f);

    assert(env_service_init() == ROV_OK);
    assert(env_read_all(&temp_c, &press_hpa, &hum_pct) == ROV_OK);

    /* Expectations are hand-written literals, independent of the driver. */
    assert(fabsf(temp_c - 26.5f) < 0.5f);
    assert(fabsf(press_hpa - 1013.25f) < 1.0f);
    assert(fabsf(hum_pct - 42.0f) < 1.0f);

    mock_bsp_reset();
    printf("  [PASS] Environmental service probing and engineering-unit reads\n");
}

/*
 * The service owns the rail INA237: power_service_init() binds it to the mock
 * bus and power_read_rail() returns engineering units. Going through the
 * register image means a wrong LSB, a wrong register or a wrong shunt all
 * surface here instead of being masked by a value-level hook.
 */
static void test_power_service_rail(void) {
    float voltage_v = 0.0f;
    float current_a = 0.0f;

    mock_bsp_reset();
    mock_ina237_set_reading(0x40u, 5.2f, 1.5f, 0.001f);

    assert(power_service_init() == ROV_OK);
    assert(power_read_rail(&voltage_v, &current_a) == ROV_OK);
    assert(fabsf(voltage_v - 5.2f) < 0.01f);
    assert(fabsf(current_a - 1.5f) < 0.01f);

    /* A bus fault fails the read and leaves the caller's outputs untouched. */
    mock_bsp_reset();
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    voltage_v = 7.0f;
    current_a = 7.0f;
    assert(power_read_rail(&voltage_v, &current_a) == ROV_TIMEOUT);
    assert(voltage_v == 7.0f);
    assert(current_a == 7.0f);

    mock_bsp_reset();
    printf("  [PASS] Power service rail ownership and fault handling\n");
}

int main(void) {
    printf("=== Running Domain Services Tests ===\n");
    test_actuator_service_clamping();
    test_safety_service_tripping();
    test_power_service();
    test_power_service_rail();
    test_env_service();
    printf("=== All Domain Services Tests Passed ===\n");
    return 0;
}
