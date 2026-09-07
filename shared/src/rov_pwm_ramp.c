/**
 * @file rov_pwm_ramp.c
 * @brief 1 kHz Slew-Rate Ramping & Parametric Cubic Exponential Mapping.
 * @organization Purdue ROV
 */

#include "rov_pwm_ramp.h"
#include "rov_parameters.h"
#include "rov_types.h"
#include <math.h>
#include <stdint.h>

float rov_pwm_apply_expo(float raw_norm) {
    if (raw_norm > 1.0f)
        raw_norm = 1.0f;
    if (raw_norm < -1.0f)
        raw_norm = -1.0f;

    /* Performance optimization: Factor polynomial to utilize FMA
     * Original: (A * x^3) + ((1 - A) * x) => 3 multiplies, 1 add, 1 sub
     * Factored: x * (A * x^2 + (1 - A)) => 2 multiplies, 1 add, 1 sub (allows FMA)
     */
    float squared = raw_norm * raw_norm;
    return raw_norm * ((ROV_PWM_EXPO_FACTOR * squared) + (1.0f - ROV_PWM_EXPO_FACTOR));
}

uint16_t rov_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us) {
    if (target_us > ROV_PWM_MAX_US)
        target_us = ROV_PWM_MAX_US;
    if (target_us < ROV_PWM_MIN_US)
        target_us = ROV_PWM_MIN_US;

    if (current_us < target_us) {
        if ((target_us - current_us) <= max_step_us) {
            return target_us;
        }
        return current_us + max_step_us;
    } else if (current_us > target_us) {
        if ((current_us - target_us) <= max_step_us) {
            return target_us;
        }
        return current_us - max_step_us;
    }
    return current_us;
}

/* ========================================================================== */
/* Backward Compatibility Export Symbols                                      */
/* ========================================================================== */
#undef x19_pwm_apply_expo
#undef x19_pwm_step_ramp

float x19_pwm_apply_expo(float raw_norm) {
    return rov_pwm_apply_expo(raw_norm);
}

uint16_t x19_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us) {
    return rov_pwm_step_ramp(current_us, target_us, max_step_us);
}
