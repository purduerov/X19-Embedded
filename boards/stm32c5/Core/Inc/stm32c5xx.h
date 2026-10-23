/**
 * @file stm32c5xx.h
 * @brief Top-level CMSIS device header for STM32C5 Series.
 * @organization Purdue ROV
 */

#ifndef STM32C5XX_H
#define STM32C5XX_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(STM32C542xx)
#define STM32C542xx
#endif

#if defined(STM32C542xx)
#include "stm32c542xx.h"
#endif

extern uint32_t SystemCoreClock;
void SystemInit(void);
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_H */
