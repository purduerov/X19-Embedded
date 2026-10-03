/**
 * @file actuator_service.h
 * @brief High-level actuator service (ESC PWM & pneumatic solenoids).
 * @author Purdue ROV Embedded Team
 *
 * Provides domain-level actuator control for thrusters and tooling.
 * Enforces neutral bounds clamping [1000..2000 us] and safety interlocks.
 */

#ifndef X19_ACTUATOR_SERVICE_H
#define X19_ACTUATOR_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the actuator service, setting all outputs to neutral.
 */
void actuator_service_init(void);

/**
 * @brief Set the command pulse width on an ESC channel with safety clamping.
 * @param channel ESC index (0 to 7).
 * @param pulse_us Requested pulse width in microseconds (clamped to 1000..2000 us).
 */
void pwm_set_pulse_us(uint8_t channel, uint16_t pulse_us);

/**
 * @brief Get the active pulse width on an ESC channel.
 * @param channel ESC index (0 to 7).
 * @return Pulse width in microseconds.
 */
uint16_t pwm_get_pulse_us(uint8_t channel);

/**
 * @brief Set pneumatic solenoid output mask.
 * @param mask 10-bit solenoid bitmask.
 */
void solenoid_set_mask(uint16_t mask);

/**
 * @brief Get active pneumatic solenoid output mask.
 * @return 10-bit solenoid bitmask.
 */
uint16_t solenoid_get_mask(void);

/**
 * @brief Neutralize all actuator outputs immediately (safe stop).
 */
void actuators_stop_all(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_ACTUATOR_SERVICE_H */
