/**
 * @file spin_only.c
 * @brief Bare-minimum T200 bench spin test on NUCLEO-G474RE.
 *
 * Sends continuous DShot300 frames on PA8 (TIM1_CH1 / Arduino D7),
 * arms the Bluejay ESC, and runs a simple ramp-up -> hold -> ramp-down loop.
 *
 * No telemetry capture, no EDT, no decoding. This file exists to isolate
 * "does the motor spin when the link is clean" from the more elaborate
 * instrumented firmware in sandbox.c.
 *
 * Pinout on NUCLEO-G474RE:
 *   PA8  (Arduino D7)  ESC signal
 *   GND  any GND pin   common ground
 */

#include "bsp.h"
#include "dshot.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

#define DSHOT_BAUD_KHZ   300U  /* DShot300 */

/*
 * 3D / Forward-Reverse throttle map (Bluejay):
 *   0        = disarmed
 *   1048     = neutral (armed, motor stopped)
 *   1049-2000 = forward, 0.1% .. 100%
 *   48-1047  = reverse, 100% .. 0.1%
 */
#define DSHOT_3D_NEUTRAL 1048U
#define DSHOT_3D_MAX     2000U

/*
 * Known-good bench value: 2000 (100% forward in 3D mode). 1800 and 1980 also
 * spun cleanly. Use 2000 as the operating point.
 */
#define MOTOR_SPIN_VALUE 2000U

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

static uint32_t s_t_bit = 567U;
static uint32_t s_t1h   = 425U;
static uint32_t s_t0h   = 213U;

#define GPIOA_MODER_PINS_MASK   (0x00FF0000U)
#define GPIOA_MODER_OUTPUT_MODE (0x00550000U)

static void dshot_preinit_idle(void) {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void dshot_hw_init(uint32_t baud_khz) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Keep line actively driven HIGH (BDShot idle level) for noise immunity */
    GPIOA->BSRR = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK) | GPIOA_MODER_OUTPUT_MODE;

    uint32_t sysclk = SystemCoreClock;
    s_t_bit = sysclk / (baud_khz * 1000U);
    s_t1h   = (s_t_bit * 3U) / 4U; /* 75% active low for '1' */
    s_t0h   = (s_t_bit * 3U) / 8U; /* 37.5% active low for '0' */
}

/* Send one 16-bit DShot packet: actively driven Push-Pull, line held HIGH between bits. */
static void dshot_send(uint16_t packet) {
    const uint32_t drive_low  = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11) << 16;
    const uint32_t drive_high = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;

    __disable_irq();
    GPIOA->BSRR = drive_high;
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK) | GPIOA_MODER_OUTPUT_MODE;

    uint32_t bit_start = DWT->CYCCNT;
    for (int i = 15; i >= 0; i--) {
        uint32_t t_low = (packet & (1U << i)) ? s_t1h : s_t0h;

        while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {}
        GPIOA->BSRR = drive_low;

        uint32_t low_end = bit_start + t_low;
        while ((int32_t)(DWT->CYCCNT - low_end) < 0) {}

        GPIOA->BSRR = drive_high;
        bit_start += s_t_bit;
    }
    while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {}
    GPIOA->BSRR = drive_high;
    __enable_irq();
}

static uint16_t pkt(uint16_t value) {
    return dshot_prepare_packet(value, false, true);
}

/* Stream a constant throttle for duration_ms at exactly 1 kHz */
static void spin_hold(uint16_t value, uint32_t duration_ms) {
    uint16_t packet = pkt(value);
    uint32_t frame_cycles = SystemCoreClock / 1000U;
    uint32_t next = DWT->CYCCNT;
    for (uint32_t i = 0; i < duration_ms; i++) {
        next += frame_cycles;
        dshot_send(packet);
        while ((int32_t)(DWT->CYCCNT - next) < 0) {}
    }
}

/* Linear ramp between two throttle values at 1 kHz */
static void spin_ramp(uint16_t from, uint16_t to, uint32_t duration_ms) {
    uint32_t frame_cycles = SystemCoreClock / 1000U;
    uint32_t next = DWT->CYCCNT;
    for (uint32_t i = 0; i < duration_ms; i++) {
        next += frame_cycles;
        uint16_t throttle = (uint16_t)(from +
            ((int32_t)(to - from) * (int32_t)i) / (int32_t)duration_ms);
        dshot_send(pkt(throttle));
        while ((int32_t)(DWT->CYCCNT - next) < 0) {}
    }
}

void app_main(void) {
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Bare-Min DShot Spin Bench (no telemetry)\r\n");
    printf("======================================================\r\n");

    /* 1. Idle HIGH for 2 s so the ESC boots */
    dshot_preinit_idle();
    printf("[1] Idle HIGH 2s...\r\n");
    for (int i = 2; i > 0; i--) {
        printf("   %d s\r\n", i);
        led_toggle();
        delay_ms(1000);
    }

    /* 2. Init engine */
    dshot_hw_init(DSHOT_BAUD_KHZ);
    printf("[2] DShot engine init\r\n");
    delay_ms(300);

    /* 3. Beep once so we know the ESC is alive */
    printf("[3] Beep 1\r\n");
    for (int i = 0; i < 10; i++) {
        dshot_send(pkt(1));
        delay_ms(2);
    }
    delay_ms(500);

    /* 4. Arm: stream 0-throttle frames for 3 s (0 = stop/disarmed request) */
    printf("[4] Arming @ throttle=0 for 3s...\r\n");
    for (int i = 3; i > 0; i--) {
        printf("   %d s\r\n", i);
        led_toggle();
        spin_hold(0, 1000);
    }

    printf("[5] ARMED - spin profile at %u\r\n", MOTOR_SPIN_VALUE);
    while (1) {
        printf("RAMP UP\r\n");
        led_toggle();
        spin_ramp(DSHOT_3D_NEUTRAL, MOTOR_SPIN_VALUE, 4000);

        printf("HOLD %u\r\n", MOTOR_SPIN_VALUE);
        spin_hold(MOTOR_SPIN_VALUE, 5000);

        printf("RAMP DOWN\r\n");
        spin_ramp(MOTOR_SPIN_VALUE, DSHOT_3D_NEUTRAL, 4000);

        /* Neutral rest keeps the ESC armed; dropping to 0 would disarm it. */
        printf("REST %u\r\n", DSHOT_3D_NEUTRAL);
        spin_hold(DSHOT_3D_NEUTRAL, 1500);
    }
}
