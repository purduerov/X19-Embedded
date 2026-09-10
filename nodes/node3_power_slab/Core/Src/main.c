/**
 * @file main.c
 * @brief Power Slab Hardware Entry Point (Node 3 - STM32G4).
 * Generated/Managed by STM32CubeMX. Hands over execution to app_main().
 */

#include "main.h"
#include "app.h"

int main(void) {
    /* USER CODE BEGIN 2 */
    /* Hand over execution to Application Layer (Src/app.c).
     * CubeMX code generation preserves this call, while all
     * application logic, state machines, and telemetry stay safe in app.c */

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I3C1_Init();
    MX_FDCAN1_Init();
    app_main();
    /* USER CODE END 2 */

    /* Infinite loop fallback */
    while (1) {
    }
}

void Error_Handler(void) {
    while (1) {
    }
}
