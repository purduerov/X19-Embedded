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
