/**
 * @file platform_gpio.c
 * @brief NUCLEO-F411 platform GPIO & safety brake implementation.
 */

#include "bsp.h"
#include "stm32f4xx_hal.h"

static bool g_emergency_tripped = false;

void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void led_set(bool state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool bsp_leak_probe_read(uint8_t probe_idx) {
    (void)probe_idx;
    /* NUCLEO dev bench leak probe stub - dry by default */
    return false;
}

void bsp_emergency_brake_trip(void) {
    g_emergency_tripped = true;
    for (uint8_t i = 0; i < 8; i++) {
        bsp_pwm_set_us(i, 1500);
    }
}

bool bsp_is_emergency_brake_tripped(void) {
    return g_emergency_tripped;
}
