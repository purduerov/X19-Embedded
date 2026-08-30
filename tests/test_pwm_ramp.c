#include "x19_parameters.h"
#include "x19_types.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

extern float x19_pwm_apply_expo(float raw_norm);
extern uint16_t x19_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us);

void test_pwm_slew_rate_limiting(void) {
    uint16_t current_pwm = 1500;
    uint16_t target_pwm = 1900;
    uint16_t max_step = X19_PWM_MAX_SLEW_RATE_US_PER_MS; // 2 us/ms

    // Step 1: should increase by exactly 2 us
    current_pwm = x19_pwm_step_ramp(current_pwm, target_pwm, max_step);
    assert(current_pwm == 1502);

    // Step 2: simulate 50 ms loop
    for (int i = 0; i < 49; i++) {
        current_pwm = x19_pwm_step_ramp(current_pwm, target_pwm, max_step);
    }
    assert(current_pwm == 1600); // 1500 + 50*2 = 1600
    printf("[PASS] test_pwm_slew_rate_limiting\n");
}

void test_pwm_clamps(void) {
    uint16_t current_pwm = 1500;
    // Test upper clamp (2000 us)
    uint16_t result = x19_pwm_step_ramp(current_pwm, 2500, 1000);
    assert(result <= X19_PWM_MAX_US);

    // Test lower clamp (1000 us)
    result = x19_pwm_step_ramp(current_pwm, 500, 1000);
    assert(result >= X19_PWM_MIN_US);
    printf("[PASS] test_pwm_clamps\n");
}

void test_cubic_expo_mapping(void) {
    // 0.0 stick should output 0.0
    float out_neutral = x19_pwm_apply_expo(0.0f);
    assert(fabsf(out_neutral) < 0.0001f);

    // 1.0 full stick should output 1.0
    float out_full = x19_pwm_apply_expo(1.0f);
    assert(fabsf(out_full - 1.0f) < 0.0001f);

    // 0.5 mid-stick should be softened by cubic curve (< 0.5)
    float out_mid = x19_pwm_apply_expo(0.5f);
    assert(out_mid < 0.5f && out_mid > 0.0f);
    printf("[PASS] test_cubic_expo_mapping\n");
}

int main(void) {
    printf("Running PWM Ramp & Math Unit Tests...\n");
    test_pwm_slew_rate_limiting();
    test_pwm_clamps();
    test_cubic_expo_mapping();
    printf("All PWM Tests Passed Successfully!\n");
    return 0;
}
