/**
 * @file bsp_g474.c
 * @brief Board Support Package setup and safety coordinator for NUCLEO-G474.
 */

#include "bsp.h"
#include "main.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

UART_HandleTypeDef hlpuart1;

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately to UART */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* 1. Configure clock and GPIO for LPUART1 (ST-Link Virtual COM Port) */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_HSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure User LED LD2 (PA5) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

    /* 2. Configure 400 Hz standard vehicle PWM (2500 us period) on TIM1 and TIM8 */
    __HAL_TIM_SET_AUTORELOAD(&htim1, 2499U);
    __HAL_TIM_SET_AUTORELOAD(&htim8, 2499U);

    static const uint32_t tim_channels[4] = {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4,
    };

    for (uint8_t i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, tim_channels[i], 1500U);
        __HAL_TIM_SET_COMPARE(&htim8, tim_channels[i], 1500U);

        HAL_TIM_PWM_Start(&htim1, tim_channels[i]);
        HAL_TIM_PWM_Start(&htim8, tim_channels[i]);
    }

    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);

    for (uint8_t i = 0; i < 8; i++) {
        bsp_pwm_set_us(i, 1500U);
    }

    bsp_watchdog_init(2000U);
}

int __io_putchar(int ch) {
    uint32_t timeout = 5000;
    while (!(LPUART1->ISR & USART_ISR_TXE) && --timeout) {
    }
    if (timeout > 0) {
        LPUART1->TDR = (uint8_t)ch;
    }
    return ch;
}

void bsp_emergency_brake_trip(void) {
    for (uint8_t ch = 0; ch < 8; ch++) {
        bsp_pwm_set_us(ch, 1500U);
    }
    bsp_solenoid_set(0);
}

bool bsp_is_emergency_brake_tripped(void) {
    return false;
}

void bsp_watchdog_init(uint32_t timeout_ms) {
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
#if defined(__HAL_DBGMCU_FREEZE_TIM1)
    __HAL_DBGMCU_FREEZE_TIM1();
#endif
#if defined(__HAL_DBGMCU_FREEZE_TIM8)
    __HAL_DBGMCU_FREEZE_TIM8();
#endif
#if defined(IWDG)
    uint32_t reload = (timeout_ms > 4095U) ? 4095U : ((timeout_ms > 0U) ? (timeout_ms - 1U) : 1U);
    IWDG->KR = 0x5555U; /* Unlock PR and RLR */
    IWDG->PR = 3U;      /* Prescaler /32 (~1 kHz from ~32 kHz LSI => ~1 ms/tick) */
    IWDG->RLR = reload;
    uint32_t guard = 100000U;
    while ((IWDG->SR != 0U) && (guard-- > 0U)) {
    }
    IWDG->KR = 0xAAAAU; /* Reload counter */
    IWDG->KR = 0xCCCCU; /* Start IWDG */
#else
    (void)timeout_ms;
#endif
}

void bsp_watchdog_kick(void) {
#if defined(IWDG)
    IWDG->KR = 0xAAAAU;
#endif
}
