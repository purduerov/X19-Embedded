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
    /* Factorized: raw_norm * (FACTOR * raw_norm^2 + (1 - FACTOR)) to reduce by 1 floating-point multiplication */
    return raw_norm * ((X19_PWM_EXPO_FACTOR * raw_norm * raw_norm) + (1.0f - X19_PWM_EXPO_FACTOR));
}

uint16_t x19_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us) {
    if (target_us > X19_PWM_MAX_US)
        target_us = X19_PWM_MAX_US;
    else if (target_us < X19_PWM_MIN_US)
        target_us = X19_PWM_MIN_US;

    /* Rewritten using int32_t differences to allow compiler to emit branchless cmov
       instructions, improving pipeline prediction for this 1 kHz hot loop. */
    int32_t diff = (int32_t)target_us - (int32_t)current_us;
    int32_t step = diff;

    if (step > max_step_us)
        step = max_step_us;
    else if (step < -max_step_us)
        step = -max_step_us;

    return current_us + step;
}
