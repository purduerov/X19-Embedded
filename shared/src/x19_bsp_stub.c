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

__attribute__((weak)) void delay_ms(uint32_t ms) {
    (void)ms;
}

__attribute__((weak)) void led_toggle(void) {}

__attribute__((weak)) void led_set(bool state) {
    (void)state;
}

__attribute__((weak)) bool can_init(void) {
    return true;
}

__attribute__((weak)) bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;
    return true;
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
