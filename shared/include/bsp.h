/**
 * @file bsp.h
 * @brief Board Support Package (BSP) Interface for X19 Subsea Nodes.
 * @author Purdue ROV Embedded Team
 *
 * Provides platform-independent timing, delay, and board-level hardware
 * utilities. Application code (app.c) calls these functions rather than
 * vendor ST HAL functions.
 */

#ifndef X19_BSP_H
#define X19_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize low-level board hardware, clocks, peripherals, and debug UART.
 */
void bsp_init(void);

/**
 * @brief Get elapsed system time in milliseconds since boot.
 * @return Milliseconds elapsed.
 */
uint32_t time_get_ms(void);

/**
 * @brief Get elapsed system time in microseconds since boot.
 * @return Microseconds elapsed.
 */
uint64_t time_get_us(void);

/**
 * @brief Non-preemptive millisecond delay.
 * @param ms Duration to delay in milliseconds.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Toggle the board heartbeat / diagnostic indicator LED.
 */
void led_toggle(void);

/**
 * @brief Set the board indicator LED state.
 * @param state true = ON, false = OFF.
 */
void led_set(bool state);

/**
 * @brief Output pulse width to an ESC channel (0 to 7).
 * @param channel ESC index (0 to 7).
 * @param pulse_us Pulse width in microseconds (1000 to 2000 us).
 */
void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us);

/**
 * @brief Get the last commanded pulse width on an ESC channel.
 * @param channel ESC index (0 to 7).
 * @return Pulse width in microseconds.
 */
uint16_t bsp_pwm_get_us(uint8_t channel);

/**
 * @brief Set pneumatic solenoid driver states.
 * @param mask 10-bit bitmask of energized solenoids.
 */
void bsp_solenoid_set(uint16_t mask);

/**
 * @brief Get active pneumatic solenoid driver bitmask.
 * @return 10-bit bitmask.
 */
uint16_t bsp_solenoid_get(void);

/**
 * @brief Read floor leak probe sensor input.
 * @param probe_idx Probe index (0 or 1).
 * @return true if water contact detected (wet), false if dry.
 */
bool bsp_leak_probe_read(uint8_t probe_idx);

/**
 * @brief Hardware Emergency Brake trigger (TIMx_BDTR BKIN cutoff / latch).
 */
void bsp_emergency_brake_trip(void);

/**
 * @brief Check if hardware emergency brake is tripped.
 * @return true if tripped, false if normal.
 */
bool bsp_is_emergency_brake_tripped(void);

/**
 * @brief Enable a specific DC-DC converter brick on Node 3 (0 to 3).
 * @param brick_idx Converter brick index (0 to 3).
 */
void bsp_power_brick_enable(uint8_t brick_idx);

/**
 * @brief Disable all 4 DC-DC converter bricks immediately.
 */
void bsp_power_brick_disable_all(void);

/**
 * @brief Get 5.2V logic rail voltage in millivolts (measured via INA237).
 * @return Voltage in millivolts.
 */
uint32_t bsp_get_logic_voltage_mv(void);

/**
 * @brief Query TI LM74700-Q1 ideal diode status.
 * @return true if diode status indicates normal operation (no reverse current / fault), false otherwise.
 */
bool bsp_lm74700_status_ok(void);

/**
 * @brief Get PCB temperature in degrees Celsius from onboard sensor (TMP1075).
 * @return Temperature in degrees C.
 */
float bsp_get_pcb_temperature_c(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_BSP_H */
