/**
 * @file bsp_f411.c
 * @brief Board Support Package initialization entry point for NUCLEO-F411RE.
 */

#include "bsp.h"
#include "can_interface.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately to UART */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize and start CAN hardware and filters */
    can_init();

    /* Initialize bit-bang I2C pins and run bus recovery if needed */
    bsp_i2c_init();

    bsp_watchdog_init(2000U);
}

void bsp_watchdog_init(uint32_t timeout_ms) {
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
#if defined(__HAL_DBGMCU_FREEZE_TIM1)
    __HAL_DBGMCU_FREEZE_TIM1();
#endif
#if defined(__HAL_DBGMCU_FREEZE_TIM8)
    __HAL_DBGMCU_FREEZE_TIM8();
#endif
#if defined(IWDG)
    uint32_t reload = (timeout_ms > 4095U) ? 4095U : ((timeout_ms > 0U) ? (timeout_ms - 1U) : 1U);
    IWDG->KR = 0x5555U; /* Unlock PR and RLR */
    IWDG->PR = 3U;      /* Prescaler /32 (~1 kHz from ~32 kHz LSI => ~1 ms/tick) */
    IWDG->RLR = reload;
    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (guard-- > 0U)) {
    }
    IWDG->KR = 0xAAAAU; /* Reload counter */
    IWDG->KR = 0xCCCCU; /* Start IWDG */
#else
    (void)timeout_ms;
#endif
}

void bsp_watchdog_kick(void) {
#if defined(IWDG)
    IWDG->KR = 0xAAAAU;
#endif
}
