/**
 * @file spin_only.c
 * @brief Bare-minimum T200 bench spin test on NUCLEO-G474RE (CPU BIT-BANGED).
 *
 * NOTE: This is a direct CPU cycle-timed bit-banging (DWT CYCCNT) baseline driver.
 * It does not use hardware timers or DMA. It serves as a known-good electrical
 * and ESC baseline for comparison against hardware timer / DMA implementations.
 *
 * Sends continuous DShot300 frames on PA8 (TIM1_CH1 / Arduino D7),
 * arms the Bluejay ESC, and runs a simple ramp-up -> hold -> ramp-down loop.
 *
 * Pinout on NUCLEO-G474RE:
 *   PA8  (Arduino D7)  ESC signal
 *   GND  any GND pin   common ground
 */

#if defined(ROV_UNIT_TEST) || defined(X19_UNIT_TEST)

#include "bsp.h"
#include <stdio.h>

void app_main(void) {
    printf("[Host SIL] Spin-only bench runs only on embedded target (STM32G474RE).\r\n");
}

#else

#include "bsp.h"
#include "dshot.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

#define DSHOT_BAUD_KHZ   300U /* DShot300 */

/*
 * 3D / Forward-Reverse throttle map (Bluejay):
 *   0        = disarmed
 *   1048     = neutral (armed, motor stopped)
 *   1049-2000 = forward, 0.1% .. 100%
 *   48-1047  = reverse, 100% .. 0.1%
 */
#define DSHOT_3D_MAX     2000U

/*
 * Strong bench operating value: 1700 (~65% forward in 3D mode).
 * Overcomes 14-pole cogging torque cleanly while staying below 1900+ cutoff.
 */
#define MOTOR_SPIN_VALUE 1700U

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

static uint32_t s_t_bit = 567U;
static uint32_t s_t1h = 425U;
static uint32_t s_t0h = 213U;

#define GPIOA_MODER_PINS_MASK   (0x00FF0000U)
#define GPIOA_MODER_OUTPUT_MODE (0x00550000U)
#define GPIOA_MODER_PA8_MASK    (0x00030000U)

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

    /* Enable ART accelerator (prefetch buffer, instruction cache, data cache) */
    FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    uint32_t sysclk = SystemCoreClock;
    s_t_bit = sysclk / (baud_khz * 1000U);
    s_t1h = (s_t_bit * 3U) / 4U; /* 75% active low for '1' */
    s_t0h = (s_t_bit * 3U) / 8U; /* 37.5% active low for '0' */
}

/* Send one 16-bit DShot packet: actively driven Push-Pull, line held HIGH between bits. */
static void dshot_send(uint16_t packet) {
    const uint32_t drive_low = (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11) << 16;
    const uint32_t drive_high = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;

    __disable_irq();
    GPIOA->BSRR = drive_high;
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK) | GPIOA_MODER_OUTPUT_MODE;

    /* Precompute 16 pulse low-widths so bit-bang loop does zero arithmetic */
    uint32_t t_low[16];
    for (int i = 0; i < 16; i++) {
        t_low[i] = (packet & (1U << (15 - i))) ? s_t1h : s_t0h;
    }

    /* Synchronize first bit on a clean future deadline so bit 0 (MSB) has identical timing */
    uint32_t bit_start = DWT->CYCCNT + 60U;
    while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {
    }

    for (int i = 0; i < 16; i++) {
        GPIOA->BSRR = drive_low;
        uint32_t low_end = bit_start + t_low[i];
        while ((int32_t)(DWT->CYCCNT - low_end) < 0) {
        }

        GPIOA->BSRR = drive_high;
        bit_start += s_t_bit;
        if (i < 15) {
            while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {
            }
        }
    }
    while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {
    }
    GPIOA->BSRR = drive_high;

    /* Release pin to Input (High-Z with Pull-Up) so ESC can transmit telemetry without collision */
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK);

    __enable_irq();
}

/*
 * CRC polarity selects bidirectional (BDShot) vs unidirectional DShot.
 * Inverted CRC (1) is required when ESC boots up in Idle-HIGH mode.
 */
#define DSHOT_INVERT_CRC        1U

static uint16_t pkt(uint16_t value) {
    return dshot_prepare_packet(value, false, DSHOT_INVERT_CRC != 0U);
}

/* Stream a constant throttle for duration_ms at exactly 1 kHz */
static void spin_hold(uint16_t value, uint32_t duration_ms) {
    uint16_t packet = pkt(value);
    uint32_t frame_cycles = SystemCoreClock / 1000U;
    uint32_t next = DWT->CYCCNT;
    for (uint32_t i = 0; i < duration_ms; i++) {
        next += frame_cycles;
        dshot_send(packet);

        if ((int32_t)(DWT->CYCCNT - next) > 0) {
            next = DWT->CYCCNT + frame_cycles;
        } else {
            while ((int32_t)(DWT->CYCCNT - next) < 0) {
            }
        }
    }
}

/* Linear ramp between two throttle values at 1 kHz */
static void spin_ramp(uint16_t from, uint16_t to, uint32_t duration_ms) {
    uint32_t frame_cycles = SystemCoreClock / 1000U;
    uint32_t next = DWT->CYCCNT;
    for (uint32_t i = 0; i < duration_ms; i++) {
        next += frame_cycles;
        uint16_t throttle = (uint16_t)(from + ((int32_t)(to - from) * (int32_t)i) / (int32_t)duration_ms);
        dshot_send(pkt(throttle));

        if ((int32_t)(DWT->CYCCNT - next) > 0) {
            next = DWT->CYCCNT + frame_cycles;
        } else {
            while ((int32_t)(DWT->CYCCNT - next) < 0) {
            }
        }
    }
}

void app_main(void) {
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Forward-Only DShot300 Spin Bench         \r\n");
    printf("======================================================\r\n");

    /* 1. Idle HIGH for 2 s so the ESC boots */
    dshot_preinit_idle();
    printf("[1] Idle HIGH 2s (ESC Bootup)...\r\n");
    for (int i = 2; i > 0; i--) {
        printf("   %d s\r\n", i);
        led_toggle();
        delay_ms(1000);
    }

    /* 2. Init engine */
    dshot_hw_init(DSHOT_BAUD_KHZ);
    printf("[2] DShot engine init (BDShot / High-Z release)\r\n");
    delay_ms(300);

    /* 3. Beep once so we know the ESC is alive */
    printf("[3] Sending Beep 1...\r\n");
    for (int i = 0; i < 10; i++) {
        dshot_send(pkt(1));
        delay_ms(2);
    }
    delay_ms(500);

    /* 4. Stream 0-throttle frames to Arm the ESC (Continuous 1 kHz for 3s) */
    printf("[4] Arming @ throttle=0 for 3s continuous...\r\n");
    for (int i = 3; i > 0; i--) {
        printf("   Arming: %d s...\r\n", i);
        led_toggle();
        spin_hold(0, 1000);
    }

    /* Target operating throttle in Forward-Only mode:
     * 48 = minimum spin, 2047 = maximum (100%).
     * 1800 = ~88% power (avoids 1900+ high-throttle ESC saturation/jitter). */
    const uint16_t spin_target = 1800U;

    printf("[5] ARMED! Starting Continuous Motion Loop...\r\n");
    printf(">> Ramping 48 -> %u (~88%% power) over 4.0s and holding for 15.0s...\r\n", spin_target);

    while (1) {
        led_toggle();

        /* Phase 1: Smooth Ramp Up (48 -> 1800 over 4.0s) */
        spin_ramp(48, spin_target, 4000);

        /* Phase 2: Sustained Hold at target for 15.0s */
        spin_hold(spin_target, 15000);

        /* Phase 3: Smooth Ramp Down (1800 -> 48 over 3.0s) */
        spin_ramp(spin_target, 48, 3000);

        /* Phase 4: Rest at 0 (stopped, armed) for 2.0s */
        spin_hold(0, 2000);
    }
}

#endif /* !ROV_UNIT_TEST */
