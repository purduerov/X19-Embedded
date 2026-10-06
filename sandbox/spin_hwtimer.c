/**
 * @file spin_hwtimer.c
 * @brief T200 bench spin test using a hardware-timer + DMA DShot300 driver.
 *
 * Same profile as spin_only.c, but the frame is emitted by TIM1 CH1 (PWM
 * mode 2) with a circular DMA transfer per update event, so bit timing comes
 * from the timer clock rather than CPU execution. Comparing this against
 * spin_only.c isolates whether the motor twitch was transmit jitter.
 *
 * The hardware driver is inlined here because the sandbox target compiles a
 * single file; the same code lives in dshot_hw.c / dshot_hw.h for reuse.
 *
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdio.h>

/* -------------------- hardware DShot driver -------------------- */

#define DSHOT_BAUD_KHZ 300U
/* TIM1 CH1 on PA8 (Arduino D7). */
/* Plain pointers: the HAL __HAL_TIM_* macros expect a TIM_HandleTypeDef's
   Instance field, so using them against a bare peripheral pointer fails. We drive
   the registers directly instead. */
static TIM_TypeDef *const s_tim = TIM1;
static DMA_Channel_TypeDef *const s_dma = DMA1_Channel1;
#define DSHOT_DMA_REQUEST DMA_REQUEST_TIM1_UP
#define DSHOT_FRAME_BITS  16U

/*
 * One half-word per bit. The DMA transfer width must match this: with the
 * default byte PSIZE, 16 transfers would advance only 16 bytes and write the
 * wrong width into CCR1, corrupting the frame.
 */
static volatile uint16_t s_dma_buffer[DSHOT_FRAME_BITS];
static volatile bool s_frame_done = true;

static uint16_t hw_build_packet(uint16_t value, bool telemetry_request, bool invert_crc) {
    uint16_t packet = (uint16_t)((value << 1) | (telemetry_request ? 1U : 0U));

    uint16_t csum = 0U;
    uint16_t csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    if (invert_crc) {
        csum = (uint16_t)(~csum);
    }
    csum &= 0x0FU;

    return (uint16_t)((packet << 4) | csum);
}

static void hw_init(uint32_t baud_khz) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    s_tim->CR1 &= ~TIM_CR1_CEN;
    s_tim->CR1 = 0U;
    s_tim->CR2 = 0U;
    s_tim->SMCR = 0U;
    s_tim->DIER = 0U;
    s_tim->DIER |= TIM_DIER_UDE; /* Update event generates DMA request */
    s_tim->SR = 0U;
    s_tim->CCMR1 = 0U;
    s_tim->CCER = 0U;
    s_tim->CNT = 0U;
    s_tim->RCR = 0U;

    /*
     * PWM mode 2 (OC1M = 110b): output LOW while CNT < CCR1, HIGH afterwards.
     * CCR1 is therefore the bit's low time, which is the inverted DShot
     * (BDShot) waveform with the line idling HIGH.
     *
     * OC1PE is deliberately left clear. The DMA request is generated at the
     * update event, so a direct CCR write already lands exactly on the period
     * boundary; enabling the preload would push each value one extra bit
     * period late and shift the whole frame by one bit.
     */
    s_tim->CCMR1 &= ~TIM_CCMR1_OC1PE;
    s_tim->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    s_tim->CCER |= TIM_CCER_CC1E;

    s_tim->PSC = 0U;
    s_tim->ARR = (SystemCoreClock / (baud_khz * 1000U)) - 1U;
    s_tim->EGR = TIM_EGR_UG;
    s_tim->SR = 0U;

    s_tim->CCR1 = 0U; /* low time 0 => line stays HIGH (BDShot idle) */
    s_tim->BDTR |= TIM_BDTR_MOE;

    /* Circular DMA: one half-word write to CCR1 per update event. */
    s_dma->CPAR = (uint32_t)&s_tim->CCR1;
    s_dma->CMAR = (uint32_t)s_dma_buffer;
    s_dma->CNDTR = 0U;
    s_dma->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | /* peripheral half-word, matches CCR1 */
                 DMA_CCR_MSIZE_0;                                /* memory half-word, matches the buffer */

    DMAMUX1_Channel1->CCR = DSHOT_DMA_REQUEST;

    s_frame_done = true;
}

static void hw_send_frame(uint16_t packet) {
    uint32_t period = s_tim->ARR + 1U;
    uint32_t d1h = (period * 3U) / 4U; /* '1' -> 75% low */
    uint32_t d0h = (period * 3U) / 8U; /* '0' -> 37.5% low */

    /* Bits go out MSB first and DMA starts from buffer[0], so index 0 is the
     * first bit on the wire. */
    for (uint32_t i = 0; i < DSHOT_FRAME_BITS; i++) {
        uint32_t bit = (packet >> (DSHOT_FRAME_BITS - 1U - i)) & 1U;
        s_dma_buffer[i] = bit ? d1h : d0h;
    }

    s_frame_done = false;

    /* Clear any stale completion flag from the previous frame before arming. */
    DMA1->IFCR = DMA_IFCR_CTCIF1;

    s_dma->CNDTR = DSHOT_FRAME_BITS;
    s_dma->CCR |= DMA_CCR_EN;

    /* Prime the first bit before the counter starts. */
    s_tim->CCR1 = s_dma_buffer[0];

    s_tim->CNT = 0U;
    s_tim->CR1 |= TIM_CR1_CEN;
}

/*
 * Counts frames where the DMA never reported completion. A non-zero value means
 * the TIM1_UP -> DMA1 request path is not firing, so the compare values never
 * advance past the primed first bit. Reported during the profile as evidence
 * rather than being hidden inside a silent busy-wait.
 */
static uint32_t s_dma_timeouts;

static void hw_wait_frame_done(void) {
    /*
     * Wait for the DMA transfer-complete flag. The 16th update event is exactly
     * where the last bit of the frame ends, so this is a hardware-accurate
     * completion signal.
     *
     * This deliberately does NOT wait on a CNT difference: the counter wraps
     * every ARR+1 counts (566), so CNT - start can never reach the 16-bit frame
     * length and such a loop never terminates.
     *
     * The wait is bounded by a DWT deadline so that a misconfigured request
     * path reports a timeout instead of deadlocking the CPU.
     */
    uint32_t deadline = DWT->CYCCNT + (SystemCoreClock / 200U); /* 5 ms */

    while ((DMA1->ISR & DMA_ISR_TCIF1) == 0U) {
        if ((int32_t)(DWT->CYCCNT - deadline) >= 0) {
            s_dma_timeouts++;
            break;
        }
    }

    DMA1->IFCR = DMA_IFCR_CTCIF1;

    s_tim->CR1 &= ~TIM_CR1_CEN;
    s_dma->CCR &= ~DMA_CCR_EN;

    /* Park the line HIGH for the inter-frame gap. */
    s_tim->CCR1 = 0U;
    s_tim->CNT = 0U;

    s_frame_done = true;
}

/* -------------------- spin profile -------------------- */

/*
 * 3D / Forward-Reverse throttle map (Bluejay):
 *   0          = disarmed
 *   1048       = neutral (armed, stopped)
 *   1049-2000  = forward 0.1% .. 100%
 */
#define DSHOT_3D_NEUTRAL 1048U

/* Bench result: 1800, 1980 and 2000 all spun cleanly; 2000 is fastest. */
#define MOTOR_SPIN_VALUE 2000U

static uint16_t pkt(uint16_t v) {
    return hw_build_packet(v, false, true);
}

static void send_hold(uint16_t value, uint32_t count) {
    uint16_t packet = pkt(value);
    uint32_t frame_cycles = SystemCoreClock / 1000U; /* 1 ms */
    uint32_t next = DWT->CYCCNT;

    for (uint32_t i = 0; i < count; i++) {
        next += frame_cycles;
        hw_send_frame(packet);
        hw_wait_frame_done();
        while ((int32_t)(DWT->CYCCNT - next) < 0) {
        }
    }
}

static void send_ramp(uint16_t from, uint16_t to, uint32_t count) {
    uint32_t frame_cycles = SystemCoreClock / 1000U;
    uint32_t next = DWT->CYCCNT;

    for (uint32_t i = 0; i < count; i++) {
        next += frame_cycles;
        uint16_t throttle = (uint16_t)(from + (((int32_t)to - (int32_t)from) * (int32_t)i) / (int32_t)count);
        hw_send_frame(pkt(throttle));
        hw_wait_frame_done();
        while ((int32_t)(DWT->CYCCNT - next) < 0) {
        }
    }
}

void app_main(void) {
    bsp_init();
    setvbuf(stdout, NULL, _IONBF, 0);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    printf("\r\n======================================================\r\n");
    printf("  T200 bench spin - HARDWARE TIMER + DMA DShot300      \r\n");
    printf("======================================================\r\n");

    /*
     * PA8 must be muxed to TIM1_CH1 (AF6) for the hardware frame to reach the
     * ESC. Configuring it as a plain GPIO leaves the timer driving nothing,
     * which looks exactly like a dead wire on the bench.
     */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    printf("[1] idle HIGH 2s...\r\n");
    for (int i = 2; i > 0; i--) {
        printf("   %d s\r\n", i);
        led_toggle();
        delay_ms(1000);
    }

    hw_init(DSHOT_BAUD_KHZ);
    printf("[2] TIM1 + circular DMA DShot300 engine ready\r\n");
    delay_ms(300);

    printf("[3] Beep 1\r\n");
    for (int i = 0; i < 10; i++) {
        hw_send_frame(pkt(1));
        hw_wait_frame_done();
        delay_ms(2);
    }
    delay_ms(500);

    printf("[4] Arming @ throttle=0 for 3s...\r\n");
    for (int i = 3; i > 0; i--) {
        printf("   %d s\r\n", i);
        led_toggle();
        send_hold(0, 1000);
    }

    printf("[5] ARMED - spin profile at %u\r\n", MOTOR_SPIN_VALUE);
    while (1) {
        printf("RAMP UP (48 -> %u)\r\n", MOTOR_SPIN_VALUE);
        led_toggle();
        send_ramp(48, MOTOR_SPIN_VALUE, 3000);

        printf("HOLD %u (100%% power for 10s)  [dma_timeouts=%lu]\r\n", MOTOR_SPIN_VALUE,
               (unsigned long)s_dma_timeouts);
        send_hold(MOTOR_SPIN_VALUE, 10000);

        printf("RAMP DOWN (%u -> 48)\r\n", MOTOR_SPIN_VALUE);
        send_ramp(MOTOR_SPIN_VALUE, 48, 3000);

        printf("REST 0 (stopped, armed)\r\n");
        send_hold(0, 2000);
    }
}