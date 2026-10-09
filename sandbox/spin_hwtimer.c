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
#include "dshot.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* DShot Timing @ 170 MHz SYSCLK (PSC = 0) */
/* 170 MHz / 567 = 299.8 kHz (DShot300) */
#define DSHOT_TIM_ARR  (567U - 1U) /* 566 */
#define DSHOT_BIT_0    189U        /* 1/3 duty = 1.11 us */
#define DSHOT_BIT_1    378U        /* 2/3 duty = 2.22 us */
#define TELEM_BUF_SIZE 48U
#define SPEED_MIN      48U /* 0-47 are reserved for codes */
#define SPEED_MAX      2047U

typedef enum { DSHOT_SPIN_DIRECTION_NORMAL = 0x00U, DSHOT_SPIN_DIRECTION_REVERSE = 0x01U } DShot_SpinDirectionTypeDef;

/* Handles */
extern TIM_HandleTypeDef htim1;
static DMA_HandleTypeDef hdma_tim1_ch1;
static DMA_HandleTypeDef hdma_tim1_ch2;

/* 17-halfword TX buffer (16 bits + 1 trailing clamp) */
static uint16_t dshot_dmabuffer_ccr[17];

/* Telemetry RX capture buffer (edge timestamps captured from TIM1_CH2 indirect on PA8) */
static uint16_t s_telem_raw_timestamps[TELEM_BUF_SIZE];
static dshot_telemetry_t s_latest_telemetry = {0};
static uint32_t s_frames_sent = 0;
static uint32_t s_telem_received = 0;
static uint32_t s_telem_crc_errors = 0;

void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}

void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch2);
}

static volatile uint32_t s_pulse_cplt_count = 0;
static volatile uint32_t s_last_edges = 0;
static volatile uint32_t s_max_edges = 0;
static volatile HAL_StatusTypeDef s_ic_start_status = HAL_OK;

/* Callback fired when 16-bit DShot PWM packet transmission completes */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        s_pulse_cplt_count++;
        /* Release line: set CCR1 to 0 (line idles HIGH via pull-up) */
        TIM1->CCR1 = 0;
        TIM1->ARR = 0xFFFF; /* Allow 16-bit free-running capture without 566-tick rollover */

        /* Prepare Channel 2 Indirect Input Capture DMA to receive ESC telemetry */
        s_ic_start_status =
            HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)s_telem_raw_timestamps, TELEM_BUF_SIZE);
    }
}

/* Prepare DMA buffer for 16-bit Bidirectional DShot packet (Inverted CRC enabled) */
static void dshot_prepare_dmabuffer(uint16_t *_dshot_dmabuffer_ccr, uint16_t _value, bool request_telemetry) {
    uint16_t packet = dshot_prepare_packet(_value, request_telemetry, true);

    for (int i = 0; i < 16; i++) {
        _dshot_dmabuffer_ccr[i] = (packet & 0x8000) ? DSHOT_BIT_1 : DSHOT_BIT_0;
        packet <<= 1;
    }
    _dshot_dmabuffer_ccr[16] = 0;
}

/* Decode captured edge timestamps from TIM1_CH2 indirect input capture */
static bool decode_telemetry_edges(const uint16_t *timestamps, uint16_t count, dshot_telemetry_t *out_telem) {
    if (count < 10) {
        return false;
    }

    /* Compute delta intervals between consecutive edge timestamps */
    uint16_t deltas[TELEM_BUF_SIZE];
    uint16_t delta_count = count - 1;
    uint16_t min_delta = 0xFFFF;

    for (uint16_t i = 0; i < delta_count; i++) {
        deltas[i] = (uint16_t)(timestamps[i + 1] - timestamps[i]);
        /* Minimum valid bit time: at 170 MHz, 375 kHz is 453 ticks, 300 kHz is 567 ticks */
        if (deltas[i] > 250 && deltas[i] < min_delta) {
            min_delta = deltas[i];
        }
    }

    if (min_delta < 250 || min_delta > 700) {
        return false;
    }

    /* Use min_delta as estimated 1-bit period T */
    uint16_t T = min_delta;
    uint16_t half_T = T / 2;

    /* Reconstruct 21 wire level bits */
    uint32_t bits = 0;
    uint8_t bit_idx = 0;
    uint8_t level = 0; /* Starts low on first transition */

    for (uint16_t i = 0; i < delta_count && bit_idx < 21; i++) {
        uint16_t dt = deltas[i];
        if (dt < 200) {
            continue; /* Skip spurious glitch */
        }
        uint16_t n = (dt + half_T) / T;
        if (n < 1)
            n = 1;
        if (n > 3)
            n = 3;

        for (uint16_t k = 0; k < n && bit_idx < 21; k++) {
            bits = (bits << 1) | (level & 1U);
            bit_idx++;
        }
        level ^= 1U; /* Level flips at each edge */
    }

    while (bit_idx < 21) {
        bits = (bits << 1) | (level & 1U);
        bit_idx++;
    }

    static const uint8_t s_gcr_table[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09, 0x0A,
                                            0x0B, 0xFF, 0x0D, 0x0E, 0x0F, 0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05,
                                            0x06, 0x07, 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF};

    uint32_t candidates[8];
    candidates[0] = bits & 0xFFFFFU;
    candidates[1] = (bits >> 1) & 0xFFFFFU;
    candidates[2] = (~bits) & 0xFFFFFU;
    candidates[3] = ((~bits) >> 1) & 0xFFFFFU;

    for (int c = 0; c < 4; c++) {
        uint32_t rev = 0;
        for (int b = 0; b < 20; b++) {
            if (candidates[c] & (1U << b))
                rev |= (1U << (19 - b));
        }
        candidates[4 + c] = rev;
    }

    uint16_t frame_16 = 0;
    for (int c = 0; c < 8; c++) {
        uint32_t raw = candidates[c] ^ (candidates[c] >> 1);
        uint8_t n0 = s_gcr_table[raw & 0x1FU];
        uint8_t n1 = s_gcr_table[(raw >> 5) & 0x1FU];
        uint8_t n2 = s_gcr_table[(raw >> 10) & 0x1FU];
        uint8_t n3 = s_gcr_table[(raw >> 15) & 0x1FU];
        if (n0 == 0xFF || n1 == 0xFF || n2 == 0xFF || n3 == 0xFF)
            continue;
        uint8_t csum = (n3 ^ n2 ^ n1 ^ n0) & 0x0F;
        if (csum == 0x0F || csum == 0x00) {
            frame_16 = (uint16_t)((n3 << 12) | (n2 << 8) | (n1 << 4) | n0);
            dshot_parse_telemetry(frame_16, 7U, out_telem);
            return true;
        }
    }

    return false;
}

/* Transmit 1 DShot frame and collect returned telemetry */
void dshot_send_ref_speed(uint16_t _motor_speed) {
    TIM1->ARR = DSHOT_TIM_ARR;
    TIM1->CNT = 0;
    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, _motor_speed, true);
    HAL_StatusTypeDef st = HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_dmabuffer_ccr, 17);
    if (st == HAL_OK) {
        s_frames_sent++;
    }

    /* Wait 1 ms cycle (covers 56us TX + 30us deadtime + 60us RX) */
    HAL_Delay(1);

    /* Read captured telemetry from TIM1 Channel 2 DMA */
    uint16_t remaining = __HAL_DMA_GET_COUNTER(&hdma_tim1_ch2);
    uint16_t edges = TELEM_BUF_SIZE - remaining;
    s_last_edges = edges;
    if (edges > s_max_edges) {
        s_max_edges = edges;
    }
    HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_2);
    /* Re-enable timer counter so it keeps running freely */
    __HAL_TIM_ENABLE(&htim1);

    if (edges >= 10) {
        if (decode_telemetry_edges(s_telem_raw_timestamps, edges, &s_latest_telemetry)) {
            s_telem_received++;
        } else {
            s_telem_crc_errors++;
        }
    }
}

/* Transmit 1 DShot command packet */
static void dshot_send_command(uint8_t cmd) {
    TIM1->ARR = DSHOT_TIM_ARR;
    TIM1->CNT = 0;
    dshot_prepare_dmabuffer(dshot_dmabuffer_ccr, cmd, true);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)dshot_dmabuffer_ccr, 17);
    HAL_Delay(1);
    HAL_TIM_IC_Stop_DMA(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_ENABLE(&htim1);
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

    /* PA8 -> TIM1_CH1, Open-Drain with Pull-up (requires external 1k pull-up resistor to 3V3) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 Time Base (Free-Running 170 MHz counter) */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = DSHOT_TIM_ARR;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    /* TIM1 Channel 1: PWM1 Mode, active LOW polarity for Inverted DShot */
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET; /* Inactive = HIGH (pulled up by 1k resistor) */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    /* TIM1 Channel 2: Indirect Input Capture on TI1 (PA8), capturing Both Edges */
    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 2;
    HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);

    /* MOE enable */
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* DMA1 Channel 1 mapped to TIM1_CH1 (Request 42U) */
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

    /* DMA1 Channel 2 mapped to TIM1_CH2 (Request 43U) */
    hdma_tim1_ch2.Instance = DMA1_Channel2;
    hdma_tim1_ch2.Init.Request = DMA_REQUEST_TIM1_CH2;
    hdma_tim1_ch2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch2.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch2.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_tim1_ch2);
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC2], hdma_tim1_ch2);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

#define DSHOT_3D_NEUTRAL 1048U
#define DSHOT_3D_FWD_MIN 1120U
#define DSHOT_3D_REV_MIN 976U

static void hold_speed(uint16_t speed, uint32_t duration_ms) {
    for (uint32_t i = 0; i < duration_ms; i++) {
        dshot_send_ref_speed(speed);
        led_toggle();
        if ((i % 500) == 0 && i > 0) {
            float err_pct = (s_telem_received + s_telem_crc_errors > 0)
                                ? ((float)s_telem_crc_errors / (float)(s_telem_received + s_telem_crc_errors)) * 100.0f
                                : 0.0f;
            printf("  [TELEM] RPM: %5lu | eRPM: %6lu | V: %4.1fV | I: %4.1fA | T: %2dC | Stress: %3u | Packets: %lu | "
                   "Err: %.2f%%\r\n",
                   (unsigned long)s_latest_telemetry.rpm, (unsigned long)s_latest_telemetry.erpm,
                   (double)s_latest_telemetry.voltage_v, (double)s_latest_telemetry.current_a,
                   (int)s_latest_telemetry.temperature_c, (unsigned int)s_latest_telemetry.stress_level,
                   (unsigned long)s_telem_received, (double)err_pct);
        }
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
    printf("   X19 ROV - Hardware BDShot 3D + Live RPM Telemetry  \r\n");
    printf("======================================================\r\n");

    /* Hold PA8 HIGH (idle state) during boot via 1k pull-up */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    printf("[1] Delay 2000 ms (let ESC boot and motor settle)...\r\n");
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

    /* Enable Extended DShot Telemetry (EDT) for voltage/current/temp/stress */
    printf("[5] Enabling Extended DShot Telemetry (EDT)...\r\n");
    for (int i = 0; i < 15; i++) {
        dshot_send_command(DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE);
    }
    for (int i = 0; i < 200; i++) {
        dshot_send_ref_speed(0);
        led_toggle();
    }

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
