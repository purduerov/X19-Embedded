/**
 * @file power_service.h
 * @brief High-level power conversion and thermal monitoring domain service.
 * @author Purdue ROV Embedded Team
 *
 * Provides domain-level operations for Power Slab telemetry and safety:
 * converter brick control, PCB copper temperature, and diode health.
 */

#ifndef X19_POWER_SERVICE_H
#define X19_POWER_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Disable all 12 V power converter bricks immediately.
 */
void power_bricks_disable_all(void);

/**
 * @brief Read PCB copper temperature from onboard sensor.
 * @return Temperature in degrees Celsius.
 */
float power_get_pcb_temperature_c(void);

/**
 * @brief Check ideal diode controller (LM74700) status.
 * @return true if diode controller operating nominally, false on fault.
 */
bool power_lm74700_is_ok(void);

/**
 * @brief Read 5V logic rail voltage.
 * @return Voltage in millivolts.
 */
uint16_t power_get_logic_voltage_mv(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_POWER_SERVICE_H */
