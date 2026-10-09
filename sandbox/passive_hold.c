/**
 * @file passive_hold.c
 * @brief Minimal firmware: no printf, no DShot traffic, no PA8 manipulation.
 * Intended only to let the ESC's UART/TX-RX interface own ST-Link VCP
 * without our code driving USART or PA8.
 */
#include "stm32g4xx_hal.h"

void app_main(void) {
    while (1) {
        HAL_Delay(1000);
    }
}
