/**
 * @file dshot_dma.h
 * @brief STM32G474 Official HAL TIM PWM + DMA DShot Driver.
 *
 * Implements hardware-timed, jitter-free DShot streaming via TIM1_CH1 (PA8)
 * and DMA1_Channel1 (DMAMUX request 42: TIM1_CH1).
 */

#ifndef DSHOT_DMA_H
#define DSHOT_DMA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { DSHOT_SPEED_150 = 150, DSHOT_SPEED_300 = 300, DSHOT_SPEED_600 = 600 } dshot_speed_t;

/**
 * @brief Initialize TIM1 Channel 1 (PA8) and DMA1 Channel 1 for DShot.
 * @param speed DShot baud rate (150, 300, or 600 kHz).
 */
void dshot_dma_init(dshot_speed_t speed);

/**
 * @brief Transmit a 16-bit DShot packet via DMA PWM.
 * @param throttle Throttle value (0 = disarmed/idle, 48 = min run, 2047 = max).
 * @param telemetry Request telemetry bit (false = 0, true = 1).
 * @return true if DMA transmission started successfully, false if busy or error.
 */
bool dshot_dma_write(uint16_t throttle, bool telemetry);

/**
 * @brief Check if a DMA packet transmission is currently in progress.
 */
bool dshot_dma_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_DMA_H */
