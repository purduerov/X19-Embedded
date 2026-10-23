/**
 * @file stm32c5xx_hal_conf.h
 * @brief HAL configuration template for STM32C5xx.
 * @organization Purdue ROV
 */

#ifndef STM32C5XX_HAL_CONF_H
#define STM32C5XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

#define TICK_INT_PRIORITY 0x00U

#include "stm32c5xx_hal.h"

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_CONF_H */
