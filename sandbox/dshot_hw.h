/**
 * @file dshot_hw.h
 * @brief Hardware-timer DShot300 transmit driver for NUCLEO-G474RE.
 *
 * Generates the 16-bit DShot frame entirely in hardware: TIM1 CH1 runs in PWM
 * mode with a period equal to one DShot bit, and a short DMA burst writes each
 * bit's compare value into the preload register. Because the timer defines the
 * bit period, frame timing does not depend on CPU execution, so the jitter the
 * bit-banged version suffered from (~1 us per bit edge) is eliminated.
 *
 * Wire format is inverted DShot (BDShot): idle HIGH, '1' is a 75% low pulse,
 * '0' is a 37.5% low pulse, and the CRC is inverted.
 *
 * @organization Purdue ROV
 */

#ifndef DSHOT_HW_H
#define DSHOT_HW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure TIM1 CH1 and the DMA request path for DShot300 on PA8.
 *
 * Must be called once, after the HAL clock is up. Leaves PA8 driven HIGH as
 * the BDShot idle level.
 */
void dshot_hw_init(uint32_t baud_khz);

/**
 * @brief Transmit one 16-bit DShot frame.
 * @param packet Pre-assembled 16-bit frame (throttle, telemetry bit, CRC).
 *
 * Non-blocking: the frame is emitted by the timer in hardware. The function
 * waits only for the frame to finish so the next frame cannot overlap.
 */
void dshot_hw_send_frame(uint16_t packet);

/**
 * @brief Build a 16-bit BDShot packet from an 11-bit value.
 * @param value Throttle or special command (0-2047).
 * @param telemetry_request Set the telemetry-request bit.
 * @param invert_crc Use the inverted CRC required by bidirectional DShot.
 */
uint16_t dshot_hw_build_packet(uint16_t value, bool telemetry_request, bool invert_crc);

/**
 * @brief Busy-wait until a previously started frame has fully completed.
 */
void dshot_hw_wait_frame_done(void);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_HW_H */