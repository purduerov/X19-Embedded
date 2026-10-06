/**
 * @file esc_passthrough.c
 * @brief UART bridge for esc-configurator on Nucleo-G474RE.
 *
 * WHAT THIS IS:
 *   PC <-> ST-Link VCP (COM6) <-> STM32 LPUART1 <-> USART1 on PA9/PA10 <-> ESC
 *   Any byte received from the PC is forwarded to the ESC, and vice-versa.
 *
 * WIRING REQUIRED:
 *   PC connects to the ST-Link USB on the NUCLEO BOARD. On the target pins:
 *     PA9  (TX of PC-bridge) -> ESC RX
 *     PA10 (RX of PC-bridge) <- ESC TX
 *     GND <- common ground with ESC
 *
 * Then open the ESC configurator on the ST-Link VCP (COM6, 115200, no monitor).
 */

#include "stm32g4xx_hal.h"

static UART_HandleTypeDef hlpuart1; /* to ST-Link VCP */
static UART_HandleTypeDef husart1;  /* to ESC RX/TX on PA9/PA10 */

static void uart_init(UART_HandleTypeDef *h) {
    HAL_UART_Init(h);
}

void app_main(void) {
    /* LPUART1 -> ST-Link VCP (PA2/PA3, AF12) */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&hlpuart1);

    /* USART1 -> ESC RX/TX (PA9/PA10, AF7) */
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    husart1.Instance = USART1;
    husart1.Init.BaudRate = 115200;
    husart1.Init.WordLength = UART_WORDLENGTH_8B;
    husart1.Init.StopBits = UART_STOPBITS_1;
    husart1.Init.Parity = UART_PARITY_NONE;
    husart1.Init.Mode = UART_MODE_TX_RX;
    husart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    husart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    husart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    husart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&husart1);

    for (;;) {
        /* LPUART1 (VCP) -> USART1 (ESC) */
        while (__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(LPUART1->RDR & 0xFF);
            while (!__HAL_UART_GET_FLAG(&husart1, UART_FLAG_TXE)) {}
            USART1->TDR = ch;
        }
        /* USART1 (ESC) -> LPUART1 (VCP) */
        while (__HAL_UART_GET_FLAG(&husart1, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(USART1->RDR & 0xFF);
            while (!__HAL_UART_GET_FLAG(&hlpuart1, UART_FLAG_TXE)) {}
            LPUART1->TDR = ch;
        }
    }
}
