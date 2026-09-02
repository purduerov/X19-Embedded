/**
 * @file x19_pwm_ramp.c
 * @brief 1 kHz Slew-Rate Ramping & Parametric Cubic Exponential Mapping.
 * @organization Purdue ROV
 */

#include "x19_parameters.h"
#include "x19_types.h"
#include <math.h>
#include <stdint.h>

float x19_pwm_apply_expo(float raw_norm) {
    if (raw_norm > 1.0f)
        raw_norm = 1.0f;
    else if (raw_norm < -1.0f)
        raw_norm = -1.0f;

    // Optimization: Factored polynomial (a*x^3 + (1-a)*x) into x * (a*x^2 + (1-a))
    // Reduces multiplications and helps Cortex-M4 FPU utilize FMA instructions
    return raw_norm * (X19_PWM_EXPO_FACTOR * raw_norm * raw_norm + (1.0f - X19_PWM_EXPO_FACTOR));
}

uint16_t x19_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us) {
    if (target_us > X19_PWM_MAX_US)
        target_us = X19_PWM_MAX_US;
    if (target_us < X19_PWM_MIN_US)
        target_us = X19_PWM_MIN_US;

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
