/**
 * @file mock_bsp.c
 * @brief Mock Board Support Package (BSP) Implementation.
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "mock_physics.h"
#include "rov_parameters.h"
#include <string.h>

static uint64_t g_mock_time_us = 0;
static bool g_mock_led_state = false;
static uint32_t g_mock_led_toggle_count = 0;
static uint16_t g_mock_pwm_us[ROV_NUM_THRUSTERS];
static uint16_t g_mock_solenoid_mask = 0;
static bool g_mock_leak_probes[2] = {false, false};
static bool g_mock_emergency_brake_tripped = false;
static bool g_auto_advance_delay = true;
static bool g_mock_brick_enabled[4] = {false, false, false, false};
static uint32_t g_mock_logic_voltage_mv = 5200;
static bool g_mock_lm74700_ok = true;
static float g_mock_pcb_temperature_c = 25.0f;

void mock_bsp_reset(void) {
    g_mock_time_us = 0;
    g_mock_led_state = false;
    g_mock_led_toggle_count = 0;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = ROV_PWM_STOP_US;
    }
    g_mock_solenoid_mask = 0;
    g_mock_leak_probes[0] = false;
    g_mock_leak_probes[1] = false;
    g_mock_emergency_brake_tripped = false;
    g_auto_advance_delay = true;
    for (int i = 0; i < 4; i++) {
        g_mock_brick_enabled[i] = false;
    }
    g_mock_logic_voltage_mv = 5200;
    g_mock_lm74700_ok = true;
    g_mock_pcb_temperature_c = 25.0f;
    mock_physics_reset();
}

void mock_bsp_set_time_ms(uint32_t ms) {
    g_mock_time_us = (uint64_t)ms * 1000ULL;
}

void mock_bsp_advance_time_ms(uint32_t delta_ms) {
    g_mock_time_us += (uint64_t)delta_ms * 1000ULL;
    if (mock_physics_is_enabled() && delta_ms > 0) {
        mock_physics_step((float)delta_ms / 1000.0f);
    }
}

void mock_bsp_set_time_us(uint64_t us) {
    g_mock_time_us = us;
}

void mock_bsp_advance_time_us(uint64_t delta_us) {
    g_mock_time_us += delta_us;
    if (mock_physics_is_enabled() && delta_us > 0) {
        mock_physics_step((float)delta_us / 1000000.0f);
    }
}

bool mock_bsp_get_led_state(void) {
    return g_mock_led_state;
}

uint32_t mock_bsp_get_led_toggle_count(void) {
    return g_mock_led_toggle_count;
}

uint16_t mock_bsp_get_pwm_us(uint8_t channel) {
    if (channel < ROV_NUM_THRUSTERS) {
        return g_mock_pwm_us[channel];
    }
    return ROV_PWM_STOP_US;
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
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = ROV_PWM_STOP_US;
    }
    g_mock_solenoid_mask = 0;
    g_mock_emergency_brake_tripped = false;
}

uint32_t time_get_ms(void) {
    return (uint32_t)(g_mock_time_us / 1000ULL);
}

uint64_t time_get_us(void) {
    return g_mock_time_us;
}

void delay_ms(uint32_t ms) {
    if (g_auto_advance_delay) {
        g_mock_time_us += (uint64_t)ms * 1000ULL;
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
    if (channel < ROV_NUM_THRUSTERS) {
        if (g_mock_emergency_brake_tripped) {
            g_mock_pwm_us[channel] = ROV_PWM_STOP_US;
        } else {
            if (pulse_us < ROV_PWM_MIN_US) {
                pulse_us = ROV_PWM_MIN_US;
            } else if (pulse_us > ROV_PWM_MAX_US) {
                pulse_us = ROV_PWM_MAX_US;
            }
            g_mock_pwm_us[channel] = pulse_us;
        }
    }
}

uint16_t bsp_pwm_get_us(uint8_t channel) {
    if (channel < ROV_NUM_THRUSTERS) {
        return g_mock_pwm_us[channel];
    }
    return ROV_PWM_STOP_US;
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
    g_mock_solenoid_mask = 0;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        g_mock_pwm_us[i] = ROV_PWM_STOP_US;
    }
}

bool bsp_is_emergency_brake_tripped(void) {
    return g_mock_emergency_brake_tripped;
}

bool mock_bsp_is_emergency_brake_tripped(void) {
    return g_mock_emergency_brake_tripped;
}

void bsp_power_brick_enable(uint8_t brick_idx) {
    if (brick_idx < 4) {
        g_mock_brick_enabled[brick_idx] = true;
    }
}

void bsp_power_brick_disable_all(void) {
    for (int i = 0; i < 4; i++) {
        g_mock_brick_enabled[i] = false;
    }
}

uint32_t bsp_get_logic_voltage_mv(void) {
    return g_mock_logic_voltage_mv;
}

bool bsp_lm74700_status_ok(void) {
    return g_mock_lm74700_ok;
}

float bsp_get_pcb_temperature_c(void) {
    return g_mock_pcb_temperature_c;
}

bool mock_bsp_is_brick_enabled(uint8_t brick_idx) {
    if (brick_idx < 4) {
        return g_mock_brick_enabled[brick_idx];
    }
    return false;
}

void mock_bsp_set_logic_voltage_mv(uint32_t mv) {
    g_mock_logic_voltage_mv = mv;
}

void mock_bsp_set_lm74700_ok(bool ok) {
    g_mock_lm74700_ok = ok;
}

void mock_bsp_set_pcb_temperature_c(float temp_c) {
    g_mock_pcb_temperature_c = temp_c;
}

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)addr;
    (void)data;
    (void)len;
    return true;
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    (void)addr;
    if (!data || len == 0U) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        data[i] = 0U;
    }
    return true;
}
