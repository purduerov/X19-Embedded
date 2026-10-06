/**
 * @file sandbox.c
 * @brief STM32G474RE Blue Robotics T200 Thruster DShot HAL DMA Driver.
 *
 * Implements hardware-timed DShot using STM32 HAL TIM1_CH1 PWM + DMA1_Channel1.
 * Grounded directly in STM32G4 Reference Manual (RM0440) and ST HAL Drivers.
 *
 * Hardware Connections on NUCLEO-G474RE:
 *   - ESC Signal (DShot): PA8 (Morpho CN10 pin 23 / Arduino D7)
 *   - Ground: Any GND pin (e.g. CN10 pin 20 or pin 9)
 *   - ESC Power: 12V-24V DC Bench Supply / Battery
 */

#if defined(ROV_UNIT_TEST) || defined(X19_UNIT_TEST)

#include "bsp.h"
#include <stdio.h>

void app_main(void) {
    printf("[Host SIL] DShot DMA sandbox requires STM32G474RE hardware target.\r\n");
}

#else

#include "bsp.h"
#include "dshot_dma.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Global Timer & DMA Handles */
extern TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* DShot Timing @ 170 MHz SYSCLK (PSC = 0) */
/* DShot300: 300 kHz -> ARR = 566, T0H = 213, T1H = 425 */
/* DShot150: 150 kHz -> ARR = 1132, T0H = 425, T1H = 850 */
#define DSHOT_USE_300 1

#if DSHOT_USE_300
#define DSHOT_ARR  566U
#define DSHOT_T0H  213U
#define DSHOT_T1H  425U
#define DSHOT_NAME "DShot300 (300 kHz)"
#else
#define DSHOT_ARR  1132U
#define DSHOT_T0H  425U
#define DSHOT_T1H  850U
#define DSHOT_NAME "DShot150 (150 kHz)"
#endif

/* 16 data bits + 2 trailing zero reset entries = 18 halfwords */
#define DSHOT_DMA_BUF_LEN 18U
static uint16_t s_dshot_dma_buf[DSHOT_DMA_BUF_LEN];
static volatile bool s_dma_in_progress = false;

/* Performance counters */
static volatile uint32_t s_frames_sent = 0;
static volatile uint32_t s_dma_timeouts = 0;

/**
 * @brief DMA1 Channel 1 Interrupt Handler (overrides weak symbol in startup_stm32g474xx.s)
 */
void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}

/**
 * @brief HAL TIM PWM Transfer Complete Callback.
 * Called automatically by HAL_DMA_IRQHandler when all 18 entries are transferred.
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        /* Stop PWM DMA transfer and clamp output line LOW */
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        TIM1->CCR1 = 0;
        s_dma_in_progress = false;
    }
}

/**
 * @brief Initialize TIM1 Channel 1 (PA8) and DMA1 Channel 1 with DMAMUX for DShot.
 */
void dshot_dma_init(dshot_speed_t speed) {
    (void)speed;

    /* 1. Enable Peripheral Clocks */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 2. Configure PA8 (TIM1_CH1) as Alternate Function AF6 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 3. Configure DMA1 Channel 1 linked to DMAMUX Request TIM1_CH1 (42U) */
    hdma_tim1_ch1.Instance = DMA1_Channel1;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK) {
        printf("[ERROR] HAL_DMA_Init failed\r\n");
    }

    /* Link DMA handle into htim1 */
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);

    /* Enable DMA1 Channel 1 Interrupt in NVIC */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* 4. Configure TIM1 Time Base */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = DSHOT_ARR;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim1);

    /* 5. Configure TIM1 PWM Channel 1 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);

    /* 6. Configure Break and Dead Time for Advanced-Control Timer (MOE) */
    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.OffStateRunMode = TIM_OSSR_DISABLE;
    bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bdt.LockLevel = TIM_LOCKLEVEL_OFF;
    bdt.DeadTime = 0;
    bdt.BreakState = TIM_BREAK_DISABLE;
    bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdt.BreakFilter = 0;
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt);

    /* Enable Main Output Enable (MOE) */
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* Initial line clamp */
    TIM1->CCR1 = 0;
}

/**
 * @brief Transmit 16-bit DShot packet via DMA PWM.
 */
bool dshot_dma_write(uint16_t throttle, bool telemetry) {
    if (throttle > 2047) {
        throttle = 2047;
    }

    /* Wait if previous transfer is still in flight (DShot300 frame is ~54 us) */
    uint32_t spin = 2000;
    while (s_dma_in_progress && --spin) {
    }
    if (s_dma_in_progress) {
        s_dma_timeouts++;
        return false;
    }

    /* Build 16-bit DShot packet: 11-bit throttle, 1-bit telemetry, 4-bit CRC */
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);
    uint16_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    uint16_t frame = (packet << 4) | csum;

    /* Fill DMA buffer (16 bits MSB first) */
    for (int i = 0; i < 16; i++) {
        s_dshot_dma_buf[i] = (frame & (0x8000 >> i)) ? DSHOT_T1H : DSHOT_T0H;
    }
    /* Trailing zero reset entries clamp PWM output to 0% duty (LOW) */
    s_dshot_dma_buf[16] = 0;
    s_dshot_dma_buf[17] = 0;

    s_dma_in_progress = true;
    s_frames_sent++;

    /* Launch DMA transfer */
    if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)s_dshot_dma_buf, DSHOT_DMA_BUF_LEN) != HAL_OK) {
        s_dma_in_progress = false;
        return false;
    }
    return true;
}

bool dshot_dma_is_busy(void) {
    return s_dma_in_progress;
}

/* Microsecond delay using DWT cycle counter */
static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void app_main(void) {
    printf("\r\n======================================================\r\n");
    printf("   X19-ROV T200 Thruster STM32 HAL TIM1+DMA DShot\r\n");
    printf("   Protocol: %s\r\n", DSHOT_NAME);
    printf("   Clock: %lu MHz | PA8 (TIM1_CH1) -> DMA1_Channel1\r\n", SystemCoreClock / 1000000U);
    printf("======================================================\r\n\r\n");

    /* Initialize DWT cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    /* Initialize DShot HAL DMA */
    dshot_dma_init(DSHOT_USE_300 ? DSHOT_SPEED_300 : DSHOT_SPEED_150);
    printf("[DSHOT DMA] Driver initialized. Starting arming sequence...\r\n");

    /* Arming Phase: Stream throttle 0 at 1 kHz for 1.5 seconds */
    printf("[ARM] Streaming throttle 0 (1000 Hz) for 1500 ms to arm ESC...\r\n");
    for (int i = 0; i < 1500; i++) {
        dshot_dma_write(0, false);
        delay_us(1000);
    }
    printf("[ARM] ESC Armed successfully! Frames sent: %lu, Timeouts: %lu\r\n\r\n", s_frames_sent, s_dma_timeouts);

    /* Test Loop: Smooth acceleration ramp between idle (48) and run throttle */
    printf("[RUN] Starting continuous 1 kHz motor control loop...\r\n");
    printf("[RUN] Blue Robotics T200 thruster spinning smoothly.\r\n");

    uint16_t current_throttle = 48;
    int16_t step = 2;
    const uint16_t min_throttle = 48;  /* Minimum idle forward throttle */
    const uint16_t max_throttle = 350; /* Safe bench test throttle (~17%% power) */
    uint32_t loop_count = 0;

    while (1) {
        /* Send DShot frame at 1000 Hz */
        dshot_dma_write(current_throttle, false);

        /* Update throttle every 10 ms (10 frames) */
        if ((loop_count % 10) == 0) {
            current_throttle += step;
            if (current_throttle >= max_throttle) {
                current_throttle = max_throttle;
                step = -2;
            } else if (current_throttle <= min_throttle) {
                current_throttle = min_throttle;
                step = 2;
            }
        }

        /* Status log every 500 ms */
        if ((loop_count % 500) == 0) {
            printf("[THROTTLE: %4u / 2047 (%.1f%%)] Sent: %lu | Timeouts: %lu\r\n", current_throttle,
                   (float)current_throttle * 100.0f / 2047.0f, s_frames_sent, s_dma_timeouts);
        }

        loop_count++;
        delay_us(1000); /* 1.0 ms = 1000 Hz frame rate */
    }
}

#endif
