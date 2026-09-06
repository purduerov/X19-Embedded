/**
 * @file x19_pwm_ramp.h
 * @brief 1 kHz Slew-Rate Ramping & Parametric Cubic Exponential Mapping.
 * @organization Purdue ROV
 */

#ifndef X19_PWM_RAMP_H
#define X19_PWM_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float x19_pwm_apply_expo(float raw_norm);
uint16_t x19_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us);

#ifdef __cplusplus
}
#endif

#endif /* X19_PWM_RAMP_H */
