/**
 * @file actuator_service.c
 * @brief Implementation of actuator service (ESC PWM & pneumatic solenoids).
 */

#include "actuator_service.h"
#include "bsp.h"
#include "safety_service.h"

#define ESC_PULSE_MIN_US     1000U
#define ESC_PULSE_MAX_US     2000U
#define ESC_PULSE_NEUTRAL_US 1500U

static uint16_t g_pwm_pulses[8] = {ESC_PULSE_NEUTRAL_US, ESC_PULSE_NEUTRAL_US, ESC_PULSE_NEUTRAL_US,
                                   ESC_PULSE_NEUTRAL_US, ESC_PULSE_NEUTRAL_US, ESC_PULSE_NEUTRAL_US,
                                   ESC_PULSE_NEUTRAL_US, ESC_PULSE_NEUTRAL_US};
static uint16_t g_solenoid_mask = 0;

void actuator_service_init(void) {
    actuators_stop_all();
}

void pwm_set_pulse_us(uint8_t channel, uint16_t pulse_us) {
    if (channel >= 8U) {
        return;
    }

    /* Interlock: if safety is tripped, force neutral */
    if (safety_is_tripped()) {
        g_pwm_pulses[channel] = ESC_PULSE_NEUTRAL_US;
        bsp_pwm_set_us(channel, ESC_PULSE_NEUTRAL_US);
        return;
    }

    /* Enforce strict physical clamp */
    if (pulse_us < ESC_PULSE_MIN_US) {
        pulse_us = ESC_PULSE_MIN_US;
    } else if (pulse_us > ESC_PULSE_MAX_US) {
        pulse_us = ESC_PULSE_MAX_US;
    }

    g_pwm_pulses[channel] = pulse_us;
    bsp_pwm_set_us(channel, pulse_us);
}

uint16_t pwm_get_pulse_us(uint8_t channel) {
    if (channel >= 8U) {
        return ESC_PULSE_NEUTRAL_US;
    }
    return g_pwm_pulses[channel];
}

void solenoid_set_mask(uint16_t mask) {
    if (safety_is_tripped()) {
        g_solenoid_mask = 0;
        bsp_solenoid_set(0);
        return;
    }
    g_solenoid_mask = (mask & 0x03FFU);
    bsp_solenoid_set(g_solenoid_mask);
}

uint16_t solenoid_get_mask(void) {
    return g_solenoid_mask;
}

void actuators_stop_all(void) {
    for (uint8_t ch = 0; ch < 8U; ch++) {
        g_pwm_pulses[ch] = ESC_PULSE_NEUTRAL_US;
        bsp_pwm_set_us(ch, ESC_PULSE_NEUTRAL_US);
    }
    g_solenoid_mask = 0;
    bsp_solenoid_set(0);
}
