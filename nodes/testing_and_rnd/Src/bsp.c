/**
 * @file bsp.c
 * @brief Board Support Package Implementation for NUCLEO-F446RE.
 * Encapsulates STM32 HAL peripheral calls away from application logic.
 */

#include "bsp.h"
#include "can_interface.h"
#include "main.h"
#include <stdio.h>

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately to UART */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize and start CAN hardware and filters */
    can_init();
}

uint32_t time_get_ms(void) {
    return HAL_GetTick();
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void led_set(bool state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
