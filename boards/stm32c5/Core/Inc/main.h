/**
 * @file main.h
 * @brief Main board configuration header for STM32C542 target.
 * @organization Purdue ROV
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32c5xx_hal.h"

/* Pin definitions for vehicle nodes on STM32C542 */
#define LED_HEARTBEAT_Pin       GPIO_PIN_5
#define LED_HEARTBEAT_GPIO_Port GPIOA

#define LEAK_PROBE0_Pin       GPIO_PIN_4
#define LEAK_PROBE0_GPIO_Port GPIOA

#define LEAK_PROBE1_Pin       GPIO_PIN_5
#define LEAK_PROBE1_GPIO_Port GPIOA

#define EMERGENCY_CUTOFF_Pin       GPIO_PIN_8
#define EMERGENCY_CUTOFF_GPIO_Port GPIOA

#define LED_STATUS_Pin       GPIO_PIN_5
#define LED_STATUS_GPIO_Port GPIOA

#define SOL_0_Pin       GPIO_PIN_0
#define SOL_0_GPIO_Port GPIOB

/* CubeMX Peripheral initializations */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2C1_Init(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
