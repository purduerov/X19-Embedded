/**
 * @file platform_gpio.c
 * @brief NUCLEO-G474 platform GPIO and solenoid driver.
 */

#include "bsp.h"
#include "main.h"

static uint16_t g_solenoid_state_mask = 0;

void led_toggle(void) {
#if defined(LED_STATUS_GPIO_Port) && defined(LED_STATUS_Pin)
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
#else
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
#endif
}

void led_set(bool state) {
#if defined(LED_STATUS_GPIO_Port) && defined(LED_STATUS_Pin)
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

void bsp_solenoid_set(uint16_t mask) {
    g_solenoid_state_mask = (mask & 0x03FF);
#if defined(SOL_0_GPIO_Port) && defined(SOL_0_Pin)
    HAL_GPIO_WritePin(SOL_0_GPIO_Port, SOL_0_Pin, (mask & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint16_t bsp_solenoid_get(void) {
    return g_solenoid_state_mask;
}
