/**
 * @file mock_bsp.h
 * @brief Mock Board Support Package (BSP) for Host-Native Testing.
 * @organization Purdue ROV
 */

#ifndef MOCK_BSP_H
#define MOCK_BSP_H

#include "bsp.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset mock BSP state (time = 0, LEDs off, PWMs 1500 us neutral, etc.).
 */
void mock_bsp_reset(void);

/**
 * @brief Set the virtual system time in milliseconds.
 */
void mock_bsp_set_time_ms(uint32_t ms);

/**
 * @brief Advance the virtual system time by delta_ms.
 */
void mock_bsp_advance_time_ms(uint32_t delta_ms);

/**
 * @brief Query diagnostic LED state.
 */
bool mock_bsp_get_led_state(void);

/**
 * @brief Query how many times the LED was toggled.
 */
uint32_t mock_bsp_get_led_toggle_count(void);

/**
 * @brief Query current commanded pulse width on ESC channel (0 to 7).
 */
uint16_t mock_bsp_get_pwm_us(uint8_t channel);

/**
 * @brief Query active pneumatic solenoid output bitmask.
 */
uint16_t mock_bsp_get_solenoid_mask(void);

/**
 * @brief Inject wet/dry state on floor leak probe (0 or 1).
 */
void mock_bsp_set_leak_probe(uint8_t probe_idx, bool wet);

/**
 * @brief Configure whether delay_ms() auto-advances virtual time.
 * @param enable true to auto-advance virtual clock by ms parameter.
 */
void mock_bsp_set_auto_advance_delay(bool enable);

/**
 * @brief Check if emergency brake was tripped.
 */
bool mock_bsp_is_emergency_brake_tripped(void);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_BSP_H */
