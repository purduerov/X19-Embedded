/**
 * @file platform_pwm.c
 * @brief NUCLEO-F411 platform PWM & solenoid implementation.
 */

#include "bsp.h"

static uint16_t g_bench_pwm[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
static uint16_t g_bench_solenoids = 0;
static uint8_t g_bench_brick_mask = 0;

void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us) {
    if (channel < 8) {
        g_bench_pwm[channel] = pulse_us;
    }
}

uint16_t bsp_pwm_get_us(uint8_t channel) {
    return (channel < 8) ? g_bench_pwm[channel] : 1500U;
}

void bsp_solenoid_set(uint16_t mask) {
    g_bench_solenoids = mask;
}

uint16_t bsp_solenoid_get(void) {
    return g_bench_solenoids;
}

void bsp_power_brick_enable(uint8_t brick_idx) {
    if (brick_idx < 4) {
        g_bench_brick_mask |= (uint8_t)(1 << brick_idx);
    }
}

void bsp_power_brick_disable_all(void) {
    g_bench_brick_mask = 0;
}

uint32_t bsp_get_logic_voltage_mv(void) {
    return 5000U;
}

bool bsp_lm74700_status_ok(void) {
    return true;
}

float bsp_get_pcb_temperature_c(void) {
    return 24.5f;
}

bool bsp_pmbus_read_word(uint8_t pmbus_addr, uint8_t command, uint16_t *raw_word) {
    (void)pmbus_addr;
    (void)command;
    if (raw_word) {
        *raw_word = 0x0000;
    }
    return false;
}
