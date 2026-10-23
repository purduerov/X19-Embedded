/**
 * @file main.c
 * @brief STM32C5 entry point. Hands over execution to app_main().
 * @organization Purdue ROV
 */

#include "main.h"
#include "app.h"
#include "bsp.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_I2C1_Init();
    bsp_init();

    /* USER CODE BEGIN 2 */
    app_main();
    /* USER CODE END 2 */

    while (1) {
    }
}
