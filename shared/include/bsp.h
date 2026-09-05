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

#ifdef __cplusplus
}
#endif

#endif /* X19_BSP_H */
