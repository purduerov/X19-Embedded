/**
 * @file x19_bsp_stub.c
 * @brief Default board support package fallback routines for nodes.
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "can_interface.h"

__attribute__((weak)) void bsp_init(void) {}

__attribute__((weak)) uint32_t time_get_ms(void) {
    return 0;
}

__attribute__((weak)) uint64_t time_get_us(void) {
    return (uint64_t)time_get_ms() * 1000ULL;
}

__attribute__((weak)) void delay_ms(uint32_t ms) {
    (void)ms;
}

__attribute__((weak)) bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)addr;
    (void)data;
    (void)len;

    return false;
}

__attribute__((weak)) bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    (void)addr;
    (void)data;
    (void)len;

    return false;
}

__attribute__((weak)) void led_toggle(void) {}

__attribute__((weak)) void led_set(bool state) {
    (void)state;
}

/* A target without a real FDCAN implementation must fail closed. */
__attribute__((weak)) bool can_init(void) {
    return false;
}

__attribute__((weak)) bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;

    return false;
}

/**
 * @brief Weak fallback for safety-critical CAN transmission.
 *
 * Hardware targets should override this function with a direct FDCAN
 * hardware transmission implementation.
 *
 * A target must provide a strong implementation; the weak fallback fails
 * closed instead of claiming that a safety frame was transmitted.
 */
__attribute__((weak)) bool can_send_emergency(uint32_t id, const uint8_t *data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;
    return false;
}

__attribute__((weak)) bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    (void)id;
    (void)data;
    (void)len;

    return false;
}

__attribute__((weak)) bool can_is_bus_off(void) {
    return false;
}

__attribute__((weak)) void can_recover(void) {}

__attribute__((weak)) void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us) {
    (void)channel;
    (void)pulse_us;
}

__attribute__((weak)) uint16_t bsp_pwm_get_us(uint8_t channel) {
    (void)channel;

    return 1500;
}

__attribute__((weak)) void bsp_solenoid_set(uint16_t mask) {
    (void)mask;
}

__attribute__((weak)) uint16_t bsp_solenoid_get(void) {
    return 0;
}

__attribute__((weak)) bool bsp_leak_probe_read(uint8_t probe_idx) {
    (void)probe_idx;

    return false;
}

__attribute__((weak)) void bsp_emergency_brake_trip(void) {}

__attribute__((weak)) void bsp_power_brick_enable(uint8_t brick_idx) {
    (void)brick_idx;
}

__attribute__((weak)) void bsp_power_brick_disable_all(void) {}

__attribute__((weak)) uint32_t bsp_get_logic_voltage_mv(void) {
    return 5200;
}

__attribute__((weak)) bool bsp_lm74700_status_ok(void) {
    return true;
}

__attribute__((weak)) float bsp_get_pcb_temperature_c(void) {
    return 25.0f;
}
