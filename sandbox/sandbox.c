/**
 * @file sandbox.c
 * @brief T200 Thruster & Bluejay ESC Digital DShot Driver with Continuous 1 kHz Streaming.
 *
 * Implements smooth acceleration to max speed and deceleration in a continuous loop.
 * Eliminates ESC watchdog timeouts by streaming frames at a rock-solid 1000 Hz.
 *
 * Pinout on NUCLEO-G474RE:
 *   - Channel 0: PA8  (Morpho CN10-23 / Arduino D7) - Primary ESC signal
 *   - Channel 1: PA9  (Morpho CN10-21 / Arduino D8)
 *   - Channel 2: PA10 (Morpho CN10-33 / Arduino D2)
 *   - Channel 3: PA11 (Morpho CN10-14)
 *   - GND: Any Nucleo GND pin
 */

#include "bsp.h"
#include "dshot.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define DSHOT_BAUD_KHZ    300U /* 300 kHz (DShot300) */
#define MOTOR_POLE_PAIRS  7U   /* BlueRobotics T200 thruster: 14 poles = 7 pole pairs */
#define DSHOT_CAPTURE_RX  0U   /* 1 = run per-frame telemetry capture after TX */

static uint32_t s_dshot_t_bit = 567U;
static uint32_t s_dshot_t1h   = 425U;
static uint32_t s_dshot_t0h   = 213U;

/* GCR bit timing at 375 kHz (5/4 of 300k): 2.667 us per bit */
#define GCR_BIT_CYCLES  453U
#define GCR_HALF_CYCLES 226U

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

/* Bitmasks for PA8..PA11 in GPIOA->MODER (bits 16 to 23) */
#define GPIOA_MODER_PINS_MASK   (0x00FF0000U)
#define GPIOA_MODER_OUTPUT_MODE (0x00550000U) /* 01: General Output */
#define GPIOA_MODER_INPUT_MODE  (0x00000000U) /* 00: Input (High-Z) */

static volatile uint32_t s_last_raw21 = 0U;

static dshot_telemetry_t s_telemetry = {0};
static volatile uint32_t s_gcr_bit_cycles = 453U; /* Filled in from SystemCoreClock */
static bool dshot_capture_telemetry(uint32_t *raw_bits);

/* Debug: edge-delta capture of the last telemetry window */
static uint32_t s_dbg_deltas[40];
static uint32_t s_dbg_n;

/**
 * @brief Configure pins as High-Z input with pullup.
 */
static void dshot_preinit_idle_high(void) {
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

/**
 * @brief Initialize DWT cycle counter and configure output pins for inverted DShot.
 */
static void dshot_init(uint32_t baud_khz) {
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

    /* Start in Input mode (High-Z with Pull-Up) */
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK) | GPIOA_MODER_INPUT_MODE;

    uint32_t sysclk = SystemCoreClock; /* 170,000,000 Hz */
    s_dshot_t_bit = sysclk / (baud_khz * 1000U);
    s_dshot_t1h   = (s_dshot_t_bit * 3U) / 4U; /* 75% active low */
    s_dshot_t0h   = (s_dshot_t_bit * 3U) / 8U; /* 37.5% active low */

    /* Telemetry back-channel runs at 5/4 x the DShot bitrate */
    s_gcr_bit_cycles = sysclk / ((baud_khz * 1000U * 5U) / 4U);
}

/**
 * @brief Transmit 16-bit DShot frame: Push-Pull active-low pulses, actively driven HIGH between frames.
 */
static void dshot_send_frame(uint16_t packet) {
    const uint32_t pin_mask_a = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    const uint32_t drive_low_a  = pin_mask_a << 16;
    const uint32_t drive_high_a = pin_mask_a;

    __disable_irq();

    /* 1. Ensure pin is Push-Pull Output and driven HIGH */
    GPIOA->BSRR = drive_high_a;
    GPIOA->MODER = (GPIOA->MODER & ~GPIOA_MODER_PINS_MASK) | GPIOA_MODER_OUTPUT_MODE;

    uint32_t bit_start = DWT->CYCCNT;
    for (int i = 15; i >= 0; i--) {
        uint32_t t_low = (packet & (1U << i)) ? s_dshot_t1h : s_dshot_t0h;

        /* Wait until start of bit window */
        while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {}

        /* Active LOW pulse (falling edge) */
        GPIOA->BSRR = drive_low_a;

        /* Wait for low duration */
        uint32_t low_end = bit_start + t_low;
        while ((int32_t)(DWT->CYCCNT - low_end) < 0) {}

        /* Return HIGH (rising edge) */
        GPIOA->BSRR = drive_high_a;

        bit_start += s_dshot_t_bit;
    }

    /* Wait for 16th bit period to end */
    while ((int32_t)(DWT->CYCCNT - bit_start) < 0) {}

    /* 2. Keep pin actively driven HIGH in Push-Pull mode (immune to motor noise) */
    GPIOA->BSRR = drive_high_a;

    __enable_irq();

    /* 3. Bidirectional DShot: ESC answers ~30us later on the same wire (PA8).
     * Capture the GCR telemetry stream, decode it, and fold it into s_telemetry. */
#if DSHOT_CAPTURE_RX
    uint32_t raw21 = 0U;
    if (dshot_capture_telemetry(&raw21)) {
        s_last_raw21 = raw21;
        uint16_t frame16 = 0U;
        if (dshot_decode_telemetry_frame(raw21, &frame16) ||
            dshot_decode_telemetry_frame((~raw21) & 0x001FFFFFU, &frame16)) {
            dshot_parse_telemetry(frame16, MOTOR_POLE_PAIRS, &s_telemetry);
            s_telemetry.telemetry_received++;
        } else {
            s_telemetry.telemetry_errors++;
        }
    }
    s_telemetry.packets_sent++;
#endif
}

/**
 * @brief Capture one 21-bit GCR telemetry frame from the ESC on PA8.
 * @param raw_bits Output: 21 sampled line levels, first-received in the MSB.
 * @return true if a response edge was found within the timeout window.
 */
static bool dshot_capture_telemetry(uint32_t *raw_bits) {
    /* Switch only PA8 to High-Z input with pull-up; PA9-PA11 stay driven HIGH.
     * The ESC needs the line released to drive its response back. */
    GPIOA->MODER = (GPIOA->MODER & ~(0x3U << 16)) | (0x0U << 16);

    /* Edge-timestamp capture of the ESC response (debug) */
    GPIOA->MODER = (GPIOA->MODER & ~(0x3U << 16)) | (0x0U << 16);

    uint32_t t_end = DWT->CYCCNT + 60U * s_gcr_bit_cycles;
    uint32_t last_t = DWT->CYCCNT;
    uint32_t prev = (GPIOA->IDR & GPIO_PIN_8) ? 1U : 0U;
    uint32_t n = 0U;
    while (((int32_t)(t_end - DWT->CYCCNT) > 0) && (n < 40U)) {
        uint32_t lvl = (GPIOA->IDR & GPIO_PIN_8) ? 1U : 0U;
        if (lvl != prev) {
            s_dbg_deltas[n++] = DWT->CYCCNT - last_t;
            last_t = DWT->CYCCNT;
            prev = lvl;
        }
    }
    s_dbg_n = n;

    *raw_bits = 0U;
    goto restore;

restore:
    /* Restore PA8 to actively-driven Push-Pull HIGH between frames */
    GPIOA->BSRR = GPIO_PIN_8;
    GPIOA->MODER = (GPIOA->MODER & ~(0x3U << 16)) | (0x1U << 16);
    return false;
}

/**
 * @brief Stream a constant throttle value for duration_ms at exact 1000.0 Hz via DWT.
 */
static void dshot_print_telemetry(void) {
    printf("   [Telemetry] eRPM=%lu RPM=%lu V=%.2f I=%.1fA T=%dC stress=%u status=0x%02X maxstress=%u last=%d rx=%lu err=%lu raw=0x%05lX\r\n",
           (unsigned long)s_telemetry.erpm, (unsigned long)s_telemetry.rpm,
           (double)s_telemetry.voltage_v, (double)s_telemetry.current_a,
           (int)s_telemetry.temperature_c, (unsigned)s_telemetry.stress_level,
           (unsigned)s_telemetry.status_flags, (unsigned)s_telemetry.status_max_stress,
           (int)s_telemetry.last_type,
           (unsigned long)s_telemetry.telemetry_received,
           (unsigned long)s_telemetry.telemetry_errors,
           (unsigned long)s_last_raw21);
    printf("   [Edges] n=%lu:", (unsigned long)s_dbg_n);
    for (uint32_t i = 0; i < s_dbg_n && i < 20U; i++) {
        printf(" %lu", (unsigned long)s_dbg_deltas[i]);
    }
    printf("\r\n");
}

static void dshot_hold(uint16_t value, uint32_t duration_ms) {
    uint16_t packet = dshot_prepare_packet(value, false, true);
    uint32_t frame_cycles = SystemCoreClock / 1000U; /* 1 ms per frame */
    uint32_t next_tick = DWT->CYCCNT;

    for (uint32_t ms = 0; ms < duration_ms; ms++) {
        next_tick += frame_cycles;
        dshot_send_frame(packet);
        while ((int32_t)(DWT->CYCCNT - next_tick) < 0) {}
    }
}

/**
 * @brief Smooth continuous ramp between two throttle levels over duration_ms at exact 1000.0 Hz.
 */
static void dshot_ramp(uint16_t start_val, uint16_t end_val, uint32_t duration_ms) {
    uint32_t frame_cycles = SystemCoreClock / 1000U; /* 1 ms per frame */
    uint32_t next_tick = DWT->CYCCNT;

    for (uint32_t ms = 0; ms < duration_ms; ms++) {
        next_tick += frame_cycles;

        uint16_t throttle = start_val;
        if (duration_ms > 0) {
            throttle = (uint16_t)(start_val +
                ((int32_t)(end_val - start_val) * (int32_t)ms) / (int32_t)duration_ms);
        }

        uint16_t packet = dshot_prepare_packet(throttle, false, true);
        dshot_send_frame(packet);

        while ((int32_t)(DWT->CYCCNT - next_tick) < 0) {}
    }
}

void app_main(void) {
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - T200 Thruster DShot300 High-Speed Bench  \r\n");
    printf("======================================================\r\n");
    printf("Protocol: Bidirectional DShot300 (1000.0 Hz Exact DWT Stream)\r\n");
    printf("Line Driver: Push-Pull Active-HIGH (Motor Noise Immune)\r\n");
    printf("Target Board: NUCLEO-G474RE (SYSCLK @ 170 MHz)\r\n");
    printf("Hardware Pin: Arduino D7 (PA8 / TIM1_CH1)\r\n");
    printf("------------------------------------------------------\r\n");

    /* 1. Hold pin High-Z Pull-up for 2s for ESC bootup */
    dshot_preinit_idle_high();
    printf("[Step 1] Line held in Idle-HIGH pullup for 2s (ESC Bootup)...\r\n");
    for (int i = 2; i > 0; i--) {
        printf("   Boot wait: %d s...\r\n", i);
        led_toggle();
        delay_ms(1000);
    }

    /* 2. Configure DShot Engine */
    dshot_init(DSHOT_BAUD_KHZ);
    printf("[Step 2] DShot Engine Initialized. Push-Pull Active-HIGH.\r\n");
    delay_ms(300);

    /* 3. Send Beep 1 Command */
    printf("[Step 3] Sending DShot BEEP 1 Command...\r\n");
    uint16_t beep_packet = dshot_prepare_packet(1, false, true);
    for (int i = 0; i < 10; i++) {
        dshot_send_frame(beep_packet);
        delay_ms(2);
    }
    delay_ms(500);

    /* 4. Stream Zero-Throttle Frames to Arm the ESC (Continuous 1 kHz for 3s) */
    printf("[Step 4] Streaming Zero Throttle to Arm ESC (3s continuous)...\r\n");
    for (int i = 3; i > 0; i--) {
        printf("     Arming hold (throttle = 0): %d s...\r\n", i);
        led_toggle();
        dshot_hold(0, 1000);
    }

    printf("\r\n[Step 5] ESC is ARMED! Starting Continuous Motion Loop...\r\n");
    uint32_t cycle = 1;

    while (1) {
        printf("\r\n======================================================\r\n");
        printf(">>> [Cycle %lu] Starting Smooth Max-Speed Profile <<<\r\n", (unsigned long)cycle);
        printf("======================================================\r\n");

        /* Phase 1: Smooth Forward Ramp to MAX SPEED (1048 -> 2000 over 3.0 seconds) */
        printf(">> [Phase 1: Ramp Up] Accelerating smoothly 0%% -> 100%% (Throttle 1048 -> 2000, 3.0s)...\r\n");
        led_toggle();
        dshot_ramp(1048, 2000, 3000);
        dshot_print_telemetry();

        /* Phase 2: Hold at MAX SPEED (2000) for 2.5 seconds */
        printf(">> [Phase 2: MAX SPEED HOLD] Throttle 2000 (100%% Full Power) for 2.5 seconds...\r\n");
        led_toggle();
        dshot_hold(2000, 2500);
        dshot_print_telemetry();

        /* Phase 3: Smooth Ramp Down from Max Speed (2000 -> 1048 over 3.0 seconds) */
        printf(">> [Phase 3: Ramp Down] Decelerating smoothly 100%% -> 0%% (Throttle 2000 -> 1048, 3.0s)...\r\n");
        led_toggle();
        dshot_ramp(2000, 1048, 3000);
        dshot_print_telemetry();

        /* Phase 4: Gentle Rest at Neutral (1048) for 1.5 seconds (stays armed) */
        printf(">> [Phase 4: Neutral Rest] Throttle 1048 (Motor Stopped, Armed) for 1.5s...\r\n");
        led_toggle();
        dshot_hold(1048, 1500);
        dshot_print_telemetry();

        cycle++;
    }
}
