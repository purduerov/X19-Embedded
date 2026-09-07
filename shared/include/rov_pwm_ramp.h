/**
 * @file rov_pwm_ramp.h
 * @brief 1 kHz Slew-Rate Ramping & Parametric Cubic Exponential Mapping.
 * @organization Purdue ROV
 */

#ifndef ROV_PWM_RAMP_H
#define ROV_PWM_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

float rov_pwm_apply_expo(float raw_norm);
uint16_t rov_pwm_step_ramp(uint16_t current_us, uint16_t target_us, uint16_t max_step_us);

/* ========================================================================== */
/* BACKWARD COMPATIBILITY ALIASES (X19 Vehicle Profile)                       */
/* ========================================================================== */
#define x19_pwm_apply_expo rov_pwm_apply_expo
#define x19_pwm_step_ramp  rov_pwm_step_ramp

#ifdef __cplusplus
}
#endif

#endif /* ROV_PWM_RAMP_H */
