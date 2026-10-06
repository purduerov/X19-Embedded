/**
 * @file dshot_hw.c
 * @brief Hardware-timer DShot300 transmit driver implementation.
 *
 * TIM1 CH1 drives PA8 (Arduino D7) in PWM mode 2, so the pin idles HIGH and
 * pulls LOW up to CCR1 within each bit period - exactly the inverted DShot
 * (BDShot) waveform, where CCR1 is the "low time" of the bit.
 *
 * A circular DMA transfer driven by TIM1's update event writes one compare
 * value per bit period, so the sequence of bits is emitted by hardware. Bit
 * period accuracy therefore comes from the timer clock and not from CPU
 * execution, which removes the ~1 us per-edge jitter the bit-banged version
 * had and which showed up as motor twitch.
 *
 * @organization Purdue ROV
 */

#include "dshot_hw.h"

#include "stm32g4xx_hal.h"

#define DSHOT_TIMER       TIM1
#define DSHOT_DMA_CHANNEL DMA1_Channel1
#define DSHOT_DMA_REQUEST DMA_REQUEST_TIM1_UP

#define DSHOT_FRAME_BITS 16U

/* One half-word per bit; PSIZE/MSIZE below must match or the frame corrupts. */
static volatile uint16_t s_dma_buffer[DSHOT_FRAME_BITS];
static volatile bool s_frame_done = true;

uint16_t dshot_hw_build_packet(uint16_t value, bool telemetry_request, bool invert_crc) {
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

void dshot_hw_init(uint32_t baud_khz) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    __HAL_TIM_DISABLE(DSHOT_TIMER);
    DSHOT_TIMER->CR1 = 0U;
    DSHOT_TIMER->CR2 = 0U;
    DSHOT_TIMER->SMCR = 0U;
    DSHOT_TIMER->DIER = 0U;
    DSHOT_TIMER->SR = 0U;
    DSHOT_TIMER->CCMR1 = 0U;
    DSHOT_TIMER->CCER = 0U;
    DSHOT_TIMER->CNT = 0U;
    DSHOT_TIMER->RCR = 0U;

    /*
     * PWM mode 2 (OC1M = 110b): output is LOW while CNT < CCR1 and HIGH for
     * the rest of the period. CCR1 is therefore the bit's low time, which is
     * the inverted DShot (BDShot) waveform with the line idling HIGH.
     *
     * OC1PE is deliberately left clear. The DMA request is generated at the
     * update event, so a direct CCR write already lands exactly on the period
     * boundary; enabling the preload would push each value one extra bit
     * period late and shift the whole frame by one bit.
     */
    DSHOT_TIMER->CCMR1 &= ~TIM_CCMR1_OC1PE;
    DSHOT_TIMER->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    DSHOT_TIMER->CCER |= TIM_CCER_CC1E;

    /*
     * PA8 must be muxed to TIM1_CH1 (AF6) or the timer drives nothing and the
     * line sits at its last GPIO level, which reads as a dead wire on the bench.
     */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Full 170 MHz timer clock, one counter tick per DShot bit. */
    DSHOT_TIMER->PSC = 0U;
    DSHOT_TIMER->ARR = (SystemCoreClock / (baud_khz * 1000U)) - 1U;
    DSHOT_TIMER->EGR = TIM_EGR_UG;
    DSHOT_TIMER->SR = 0U;

    DSHOT_TIMER->CCR1 = 0U; /* idle: low time zero, line stays HIGH */
    __HAL_TIM_MOE_ENABLE(DSHOT_TIMER);

    /* Route TIM1 update events to DMA1 Channel 1 through the DMAMUX. */
    DSHOT_DMA_CHANNEL->CPAR = (uint32_t)&DSHOT_TIMER->CCR1;
    DSHOT_DMA_CHANNEL->CMAR = (uint32_t)s_dma_buffer;
    DSHOT_DMA_CHANNEL->CNDTR = 0U; /* no transfers armed until a frame */

    /* Memory increment, half-word wide to CCR1, circular. */
    DSHOT_DMA_CHANNEL->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | /* peripheral half-word, matches CCR1 */
                             DMA_CCR_MSIZE_0;                                /* memory half-word, matches the buffer */
    DSHOT_DMA_CHANNEL->CMAR = (uint32_t)s_dma_buffer;

    /* Route the TIM1 update request through the DMAMUX. */
    DMAMUX1_Channel1->CCR = DSHOT_DMA_REQUEST;

    s_frame_done = true;
}

void dshot_hw_send_frame(uint16_t packet) {
    uint32_t period = DSHOT_TIMER->ARR + 1U;
    uint32_t d1h = (period * 3U) / 4U; /* '1' -> 75% low */
    uint32_t d0h = (period * 3U) / 8U; /* '0' -> 37.5% low */

    /*
     * Frame bits go out MSB first, and the DMA transfers one 16-bit word per
     * update event starting with buffer[0], so index 0 must be the first bit
     * on the wire.
     */
    for (uint32_t i = 0; i < DSHOT_FRAME_BITS; i++) {
        uint32_t bit = (packet >> (DSHOT_FRAME_BITS - 1U - i)) & 1U;
        s_dma_buffer[i] = bit ? d1h : d0h;
    }

    s_frame_done = false;

    /* One 16-bit transfer per bit period, wrapping for the 16 bits. */
    DSHOT_DMA_CHANNEL->CNDTR = DSHOT_FRAME_BITS;
    __HAL_DMA_ENABLE(DSHOT_DMA_CHANNEL);

    /* Drive the first bit's compare value before the counter starts. */
    DSHOT_TIMER->CCR1 = s_dma_buffer[0];

    DSHOT_TIMER->CNT = 0U;
    __HAL_TIM_ENABLE(DSHOT_TIMER);
}

void dshot_hw_wait_frame_done(void) {
    /*
     * Wait out exactly 16 bit periods. Counting timer ticks rather than wall
     * clock keeps this exact and immune to interrupt jitter.
     */
    uint32_t frame_ticks = (DSHOT_TIMER->ARR + 1U) * DSHOT_FRAME_BITS;
    uint32_t start = DSHOT_TIMER->CNT;
    uint32_t elapsed = 0U;

    while (elapsed < frame_ticks) {
        elapsed = (DSHOT_TIMER->CNT - start);
    }

    __HAL_TIM_DISABLE(DSHOT_TIMER);
    __HAL_DMA_DISABLE(DSHOT_DMA_CHANNEL);

    /* Park the line HIGH for the inter-frame gap. */
    DSHOT_TIMER->CCR1 = 0U;
    DSHOT_TIMER->CNT = 0U;

    s_frame_done = true;
}