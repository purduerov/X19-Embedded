/**
 * @file bsp.c
 * @brief Board Support Package Implementation for Node 1 (Pi Shield).
 * @author Purdue ROV Embedded Team
 *
 * Implements low-level hardware routines for Node 1:
 * - Floor leak GPIO input reading and EXTI interrupt callback
 * - Hardware emergency cutoff trigger
 * - Microsecond/millisecond system timers
 * - Status LED toggling
 *
 * Note: Under host unit testing (-DROV_UNIT_TEST), mock_bsp.c provides the
 * implementation. This file is compiled when targeting bare-metal hardware.
 */

#ifndef ROV_UNIT_TEST

#include "bsp.h"
#include "can_interface.h"

/*
 * When compiling against STM32CubeMX generated drivers, include main.h.
 * Provide conditional fallbacks if compiling in minimal/stub environments.
 */
#if __has_include("main.h")
#include "main.h"
#endif

void bsp_init(void) {
    /* Peripheral init is performed in main.c by STM32CubeMX generated code */
}

uint32_t time_get_ms(void) {
#if defined(HAL_GetTick) || defined(STM32C5) || defined(STM32G4)
    return HAL_GetTick();
#else
    return 0;
#endif
}

uint64_t time_get_us(void) {
    /*
     * For microsecond resolution, read DWT->CYCCNT or a high-resolution timer.
     * Fallback to millisecond scaling when cycle counter is not configured.
     */
    return (uint64_t)time_get_ms() * 1000ULL;
}

void delay_ms(uint32_t ms) {
#if defined(HAL_Delay) || defined(STM32C5) || defined(STM32G4)
    HAL_Delay(ms);
#else
    (void)ms;
#endif
}

void led_toggle(void) {
#if defined(LED_HEARTBEAT_GPIO_Port) && defined(LED_HEARTBEAT_Pin)
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
#endif
}

void led_set(bool state) {
#if defined(LED_HEARTBEAT_GPIO_Port) && defined(LED_HEARTBEAT_Pin)
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)state;
#endif
}

bool bsp_leak_probe_read(uint8_t probe_idx) {
    /*
     * Floor leak probe 0 (PA4) and probe 1 (PA5).
     * Active LOW with pull-up: contact with water grounds the trace (wet = LOW).
     */
#if defined(LEAK_PROBE0_GPIO_Port) && defined(LEAK_PROBE0_Pin) && defined(LEAK_PROBE1_Pin)
    if (probe_idx == 0) {
        return (HAL_GPIO_ReadPin(LEAK_PROBE0_GPIO_Port, LEAK_PROBE0_Pin) == GPIO_PIN_RESET);
    } else if (probe_idx == 1) {
        return (HAL_GPIO_ReadPin(LEAK_PROBE1_GPIO_Port, LEAK_PROBE1_Pin) == GPIO_PIN_RESET);
    }
#else
    (void)probe_idx;
#endif
    return false;
}

void bsp_emergency_brake_trip(void) {
    /*
     * Assert local emergency cutoff GPIO signal to latch hardware break.
     */
#if defined(EMERGENCY_CUTOFF_GPIO_Port) && defined(EMERGENCY_CUTOFF_Pin)
    HAL_GPIO_WritePin(EMERGENCY_CUTOFF_GPIO_Port, EMERGENCY_CUTOFF_Pin, GPIO_PIN_SET);
#endif
}

bool bsp_is_emergency_brake_tripped(void) {
#if defined(EMERGENCY_CUTOFF_GPIO_Port) && defined(EMERGENCY_CUTOFF_Pin)
    return (HAL_GPIO_ReadPin(EMERGENCY_CUTOFF_GPIO_Port, EMERGENCY_CUTOFF_Pin) == GPIO_PIN_SET);
#else
    return false;
#endif
}

#endif /* ROV_UNIT_TEST */
