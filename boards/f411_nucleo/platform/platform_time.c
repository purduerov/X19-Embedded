/**
 * @file platform_time.c
 * @brief NUCLEO-F411 platform timing implementation.
 */

#include "bsp.h"
#include "stm32f4xx_hal.h"

uint32_t time_get_ms(void) {
    return HAL_GetTick();
}

uint64_t time_get_us(void) {
    return (uint64_t)HAL_GetTick() * 1000U;
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
