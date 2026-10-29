/**
 * @file test_transceiver.c
 * @brief Diagnostic firmware to test CAN transceiver wiring and loopback.
 *
 * Configures PB9 (TX) as GPIO output and PB8 (RX) as GPIO input.
 * Drives PB9 dominant (LOW) and recessive (HIGH), verifying that the
 * transceiver properly mirrors the signal back to PB8 and drives the bus.
 *
 * Build & Run:
 *   python rov.py run sandbox/test_transceiver.c -b g474
 */

#include "app.h"
#include "bsp.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>

void app_main(void) {
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("  CAN Transceiver Electrical & Loopback Diagnostic\r\n");
    printf("  Testing STM32G474: PB9 (TXD) <---> PB8 (RXD)\r\n");
    printf("======================================================\r\n\r\n");

    /* De-init FDCAN so pins are freed */
    extern FDCAN_HandleTypeDef hfdcan1;
    HAL_FDCAN_DeInit(&hfdcan1);

    /* Configure PB9 as GPIO Push-Pull Output (CAN TXD) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio_tx = {0};
    gpio_tx.Pin = GPIO_PIN_9;
    gpio_tx.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_tx.Pull = GPIO_PULLUP;
    gpio_tx.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_tx);

    /* Configure PB8 as GPIO Input with internal pullup (CAN RXD) */
    GPIO_InitTypeDef gpio_rx = {0};
    gpio_rx.Pin = GPIO_PIN_8;
    gpio_rx.Mode = GPIO_MODE_INPUT;
    gpio_rx.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio_rx);

    printf("[STEP 1] Testing Idle State (Recessive: PB9 = HIGH)...\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(10);
    GPIO_PinState rx_high = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    printf("  PB9 = HIGH (Recessive) --> PB8 reads: %s\r\n",
           (rx_high == GPIO_PIN_SET) ? "HIGH (OK)" : "LOW (FAIL: Line pulled down!)");

    printf("\r\n[STEP 2] Testing Drive State (Dominant: PB9 = LOW)...\r\n");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(10);
    GPIO_PinState rx_low = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    printf("  PB9 = LOW  (Dominant)  --> PB8 reads: %s\r\n",
           (rx_low == GPIO_PIN_RESET) ? "LOW (OK - Transceiver Loopback Active!)"
                                      : "HIGH (FAIL: Transceiver NOT driving or STBY active!)");

    if (rx_high == GPIO_PIN_SET && rx_low == GPIO_PIN_RESET) {
        printf(
            "\r\n[RESULT] SUCCESS! The CAN transceiver is powered and echoing bits between PB9 and PB8 properly.\r\n");
    } else {
        printf("\r\n[RESULT] FAILURE! Loopback did NOT work.\r\n");
        printf("  Probable causes:\r\n");
        printf("  1. Transceiver STBY / S pin is not tied to GND (in silent sleep mode).\r\n");
        printf("  2. PB9 (TX) and PB8 (RX) wires are swapped or on wrong header pins.\r\n");
        printf("  3. Transceiver VCC (5V) or GND is disconnected.\r\n");
    }

    printf("\r\n[CONTINUOUS TEST] Testing loopback every 1 second...\r\n");
    while (1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); /* Dominant */
        HAL_Delay(10);
        GPIO_PinState read_low = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); /* Recessive */
        HAL_Delay(10);
        GPIO_PinState read_high = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

        printf("Drive PB9=LOW -> PB8=%s | Drive PB9=HIGH -> PB8=%s [%s]\r\n",
               (read_low == GPIO_PIN_RESET) ? "LOW" : "HIGH", (read_high == GPIO_PIN_SET) ? "HIGH" : "LOW",
               (read_low == GPIO_PIN_RESET && read_high == GPIO_PIN_SET) ? "LOOPBACK OK" : "NO LOOPBACK");

        led_set(read_low == GPIO_PIN_RESET);
        HAL_Delay(980);
    }
}
