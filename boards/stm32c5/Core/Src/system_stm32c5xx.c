/**
 * @file system_stm32c5xx.c
 * @brief Device system configuration for STM32C5xx.
 * @organization Purdue ROV
 */

#include "stm32c5xx.h"

uint32_t SystemCoreClock = 144000000U;

void SystemCoreClockUpdate(void);

void SystemInit(void) {
    /* Set CP10 and CP11 Full Access for hardware FPU */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
#endif

    SystemCoreClockUpdate();
}

void SystemCoreClockUpdate(void) {
    SystemCoreClock = 144000000U;
}
