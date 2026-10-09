/**
 * @file platform_time.c
 * @brief NUCLEO-G474 platform timekeeping implementation.
 */

#include "bsp.h"
#include "main.h"

uint32_t time_get_ms(void) {
    return HAL_GetTick();
}

uint64_t time_get_us(void) {
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
