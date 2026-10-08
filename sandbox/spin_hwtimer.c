/**
 * @file spin_hwtimer.c
 * @brief Direct port of ufnalski/dshot_pwm_dma_l432kc to STM32G474RE.
 *
 * Source: https://github.com/ufnalski/dshot_pwm_dma_l432kc
 *
 * Hardware Connections on NUCLEO-G474RE:
 *   - ESC Signal: PA8 (Morpho CN10 pin 23 / Arduino D7)
 *   - Ground: Any GND pin
 *   - ESC Power: Bench Supply / Battery
 *
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* DShot Timing @ 170 MHz SYSCLK (PSC = 0) */
/* 170 MHz / 567 = 299.8 kHz (DShot300) */
#define DSHOT_TIM_ARR   (567U - 1U) /* 566 */
#define DSHOT_BIT_0     189U        /* 1/3 duty = 1.11 us */
#define DSHOT_BIT_1     378U        /* 2/3 duty = 2.22 us */
#define DSHOT_TELEMETRY 1           /* ufnalski: DSHOT_TELEMETRY 1 */
#define SPEED_MIN       48U         /* 0-47 are reserved for codes */
#define SPEED_MAX       2047U

typedef enum { DSHOT_SPIN_DIRECTION_NORMAL = 0x00U, DSHOT_SPIN_DIRECTION_REVERSE = 0x01U } DShot_SpinDirectionTypeDef;

/* Handles */
extern TIM_HandleTypeDef htim1;
static DMA_HandleTypeDef hdma_tim1_ch1;

/* 17-halfword buffer (16 bits + 1 trailing clamp) */
static uint16_t dshot_dmabuffer_ccr[17];

void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}

/* Exact callback from ufnalski dshot150.c */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
        }
    }
}

/* Exact packet builder from ufnalski dshot150.c */
uint16_t dshot_prepare_packet(uint16_t _command) {
    uint16_t packet;

    packet = (_command << 1) | (DSHOT_TELEMETRY ? 1 : 0);

    // compute checksum
    uint8_t csum = 0;
    uint16_t csum_data = packet;

    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;  // xor data by nibbles (AKA nybbles)
        csum_data >>= 4;
    }

    csum &= 0xf;

    packet = (packet << 4) | csum;

    return packet;
}

/* Exact buffer preparation from ufnalski dshot150.c */
void dshot_prepare_dmabuffer(uint16_t *_dshot_dmabuffer_ccr, uint16_t _value) {
    uint16_t packet;
    packet = dshot_prepare_packet(_value);

    for (int i = 0; i < 16; i++) {
        _dshot_dmabuffer_ccr[i] = (packet & 0x8000) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        packet <<= 1;
    }
    // https://electronics.stackexchange.com/questions/377604/stm32-pwm-generates-excessive-pulses
    _dshot_dmabuffer_ccr[16] = 0;
}

/* Exact arming routine from ufnalski dshot150.c */
void dshot_arm_esc(void) {
    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 0);

    for (int i = 0; i < 1000; i++) {
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_dmabuffer_ccr, 17);
        HAL_Delay(1);
    }
}

/* Exact direction routine from ufnalski dshot150.c */
void dshot_set_spin_direction(DShot_SpinDirectionTypeDef spin_dir) {
    if (spin_dir == DSHOT_SPIN_DIRECTION_NORMAL) {
        dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 20);
    } else if (spin_dir == DSHOT_SPIN_DIRECTION_REVERSE) {
        dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 21);
    } else {
        dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, 0);
    }

    for (int i = 0; i < 10; i++) {
        HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_dmabuffer_ccr, 17);
        HAL_Delay(1);
    }
}

static uint32_t s_frames_sent = 0;
static uint32_t s_errors = 0;

/* Exact speed send from ufnalski dshot150.c */
void dshot_send_ref_speed(uint16_t _motor_speed) {
    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, _motor_speed);
    HAL_StatusTypeDef st = HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_dmabuffer_ccr, 17);
    if (st == HAL_OK) {
        s_frames_sent++;
    } else {
        s_errors++;
    }
    HAL_Delay(1);
}

/* Peripheral initialization matching ufnalski tim.c and dma.c on STM32G474 */
static void MX_TIM1_Init(void) {
    /* Stop any 50 Hz PWM from bsp_init */
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_DeInit(&htim1);

    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_DBGMCU_FREEZE_TIM1();

    /* PA8 -> TIM1_CH1, Push-Pull, pull-down per README hint */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 Time Base */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = DSHOT_TIM_ARR;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    /* TIM1 Channel 1 PWM1 */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    /* MOE enable */
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* DMA1 Channel 1 mapped to TIM1_CH1 (42U) */
    hdma_tim1_ch1.Instance = DMA1_Channel1;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim1_ch1);

    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

#define DSHOT_3D_NEUTRAL 1048U
#define DSHOT_3D_FWD_MIN 1120U
#define DSHOT_3D_REV_MIN 976U

static void hold_speed(uint16_t speed, uint32_t duration_ms) {
    for (uint32_t i = 0; i < duration_ms; i++) {
        dshot_send_ref_speed(speed);
        led_toggle();
    }
}

static void ramp_speed(uint16_t from, uint16_t to) {
    if (from < to) {
        for (uint16_t s = from; s <= to; s += 2) {
            dshot_send_ref_speed(s);
            dshot_send_ref_speed(s);
            dshot_send_ref_speed(s);
            led_toggle();
        }
    } else {
        for (uint16_t s = from; s >= to; s -= 2) {
            dshot_send_ref_speed(s);
            dshot_send_ref_speed(s);
            dshot_send_ref_speed(s);
            led_toggle();
        }
    }
}

void app_main(void) {
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Hardware DMA DShot 3D Bidirectional      \r\n");
    printf("======================================================\r\n");

    /* Hold PA8 LOW during boot */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    printf("[1] Delay 2000 ms (let motor settle)...\r\n");
    HAL_Delay(2000);

    /* Initialize TIM1 and DMA */
    MX_TIM1_Init();
    printf("[2] Peripheral init done.\r\n");

    /* Arm ESC: In BLHeli_S / Bluejay, arming signal is strictly 0 */
    printf("[3] Arming ESC (streaming 0 for 2.0s)...\r\n");
    for (int i = 0; i < 2000; i++) {
        dshot_send_ref_speed(0);
        led_toggle();
    }
    printf("[4] Armed! ESC ready.\r\n");

    /* Continuous Bidirectional Alternating Loop - 100% Full Power Test */
    while (1) {
        /* Phase 1: Direction A (48..1047 range, 1047 = 100% Max Forward) */
        printf(">>> [DIR A] Startup kick to 150 (hold 350ms)...\r\n");
        hold_speed(150, 350);
        printf(">>> [DIR A] Ramping 150 -> 1047 (100%% Full Power)...\r\n");
        ramp_speed(150, 1047);
        printf(">>> [DIR A] Holding 1047 (MAX POWER) for 10.0s...\r\n");
        hold_speed(1047, 10000);
        printf(">>> [DIR A] Decelerating 1047 -> 100...\r\n");
        ramp_speed(1047, 100);
        printf(">>> [STOP] Coasting to standstill (hold 0 for 2.5s)...\r\n");
        hold_speed(0, 2500);

        /* Phase 2: Direction B (1048..2047 range, 2047 = 100% Max Reverse) */
        printf(">>> [DIR B] Startup kick to 1150 (hold 350ms)...\r\n");
        hold_speed(1150, 350);
        printf(">>> [DIR B] Ramping 1150 -> 2047 (100%% Full Power)...\r\n");
        ramp_speed(1150, 2047);
        printf(">>> [DIR B] Holding 2047 (MAX POWER) for 10.0s...\r\n");
        hold_speed(2047, 10000);
        printf(">>> [DIR B] Decelerating 2047 -> 1100...\r\n");
        ramp_speed(2047, 1100);
        printf(">>> [STOP] Coasting to standstill (hold 0 for 2.5s)...\r\n");
        hold_speed(0, 2500);
    }
}
