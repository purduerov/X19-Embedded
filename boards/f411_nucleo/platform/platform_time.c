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
#if defined(SysTick)
    uint32_t ms1, ms2, val;
    do {
        ms1 = HAL_GetTick();
        val = SysTick->VAL;
        ms2 = HAL_GetTick();
    } while (ms1 != ms2);

    uint32_t load = SysTick->LOAD;
    if (load > 0) {
        uint64_t sub_us = ((uint64_t)(load - val) * 1000ULL) / (load + 1);
        return (uint64_t)ms1 * 1000ULL + sub_us;
    }
    return (uint64_t)ms1 * 1000ULL;
#else
    return (uint64_t)HAL_GetTick() * 1000U;
#endif
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
