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

/**
 * @brief Set the mocked logic rail voltage.
 * @param voltage_mv Logic rail voltage in millivolts.
 */
void mock_bsp_set_logic_voltage_mv(uint32_t voltage_mv);

/**
 * @brief Set the mocked PCB temperature.
 * @param temperature_c PCB temperature in degrees Celsius.
 */
void mock_bsp_set_pcb_temperature_c(float temperature_c);

/**
 * @brief Set the mocked LM74700 status.
 * @param status_ok true if the ideal diode controller is healthy, false if faulted.
 */
void mock_bsp_set_lm74700_status_ok(bool status_ok);

/**
 * @brief Check whether a mocked power brick is currently enabled.
 * @param brick_idx Power brick index.
 * @return true if the brick is enabled, false otherwise.
 */
bool mock_bsp_is_power_brick_enabled(uint8_t brick_idx);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_BSP_H */
