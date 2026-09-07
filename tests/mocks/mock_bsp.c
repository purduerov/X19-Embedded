/**
 * @file mock_bsp.c
 * @brief Mock Board Support Package (BSP) Implementation.
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "x19_parameters.h"
#include <string.h>

static uint32_t g_mock_time_ms = 0;
static bool g_mock_led_state = false;
static uint32_t g_mock_led_toggle_count = 0;
static uint16_t g_mock_pwm_us[X19_NUM_THRUSTERS];
static uint16_t g_mock_solenoid_mask = 0;
static bool g_mock_leak_probes[2] = {false, false};
static bool g_mock_emergency_brake_tripped = false;
static bool g_auto_advance_delay = true;

void mock_bsp_reset(void) {
    g_mock_time_ms = 0;
    g_mock_led_state = false;
    g_mock_led_toggle_count = 0;
    for (int i = 0; i < X19_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = X19_PWM_STOP_US;
    }
    g_mock_solenoid_mask = 0;
    g_mock_leak_probes[0] = false;
    g_mock_leak_probes[1] = false;
    g_mock_emergency_brake_tripped = false;
    g_auto_advance_delay = true;
}

void mock_bsp_set_time_ms(uint32_t ms) {
    g_mock_time_ms = ms;
}

void mock_bsp_advance_time_ms(uint32_t delta_ms) {
    g_mock_time_ms += delta_ms;
}

bool mock_bsp_get_led_state(void) {
    return g_mock_led_state;
}

uint32_t mock_bsp_get_led_toggle_count(void) {
    return g_mock_led_toggle_count;
}

uint16_t mock_bsp_get_pwm_us(uint8_t channel) {
    if (channel < X19_NUM_THRUSTERS) {
        return g_mock_pwm_us[channel];
    }
    return X19_PWM_STOP_US;
}

uint16_t mock_bsp_get_solenoid_mask(void) {
    return g_mock_solenoid_mask;
}

void mock_bsp_set_leak_probe(uint8_t probe_idx, bool wet) {
    if (probe_idx < 2) {
        g_mock_leak_probes[probe_idx] = wet;
    }
}

void mock_bsp_set_auto_advance_delay(bool enable) {
    g_auto_advance_delay = enable;
}

/* ========================================================================== */
/* BSP Public Interface Implementations                                      */
/* ========================================================================== */

void bsp_init(void) {
    /* Initialize to safe neutral state */
    for (int i = 0; i < X19_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = X19_PWM_STOP_US;
    }
    g_mock_solenoid_mask = 0;
    g_mock_emergency_brake_tripped = false;
}

uint32_t time_get_ms(void) {
    return g_mock_time_ms;
}

void delay_ms(uint32_t ms) {
    if (g_auto_advance_delay) {
        g_mock_time_ms += ms;
    }
}

void led_toggle(void) {
    g_mock_led_state = !g_mock_led_state;
    g_mock_led_toggle_count++;
}

void led_set(bool state) {
    g_mock_led_state = state;
}

void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us) {
    if (channel < X19_NUM_THRUSTERS) {
        if (g_mock_emergency_brake_tripped) {
            g_mock_pwm_us[channel] = X19_PWM_STOP_US;
        } else {
            g_mock_pwm_us[channel] = pulse_us;
        }
    }
}

uint16_t bsp_pwm_get_us(uint8_t channel) {
    if (channel < X19_NUM_THRUSTERS) {
        return g_mock_pwm_us[channel];
    }
    return X19_PWM_STOP_US;
}

void bsp_solenoid_set(uint16_t mask) {
    g_mock_solenoid_mask = mask & 0x03FF;
}

uint16_t bsp_solenoid_get(void) {
    return g_mock_solenoid_mask;
}

bool bsp_leak_probe_read(uint8_t probe_idx) {
    if (probe_idx < 2) {
        return g_mock_leak_probes[probe_idx];
    }
    return false;
}

void bsp_emergency_brake_trip(void) {
    g_mock_emergency_brake_tripped = true;
    for (int i = 0; i < X19_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = X19_PWM_STOP_US;
    }
}

bool bsp_is_emergency_brake_tripped(void) {
    return g_mock_emergency_brake_tripped;
}

bool mock_bsp_is_emergency_brake_tripped(void) {
    return g_mock_emergency_brake_tripped;
}
