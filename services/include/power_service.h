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

#include "rov_types.h"

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

/**
 * @brief Initialize the power service and its 5V rail monitor (INA237).
 *
 * Acquires the board I2C bus and configures the rail monitor. Safe to call
 * more than once; the last result is retained.
 *
 * @return ROV_OK on success, error code otherwise.
 */
rov_status_t power_service_init(void);

/**
 * @brief Read the 5V rail voltage and current in engineering units.
 * @param bus_voltage_v Pointer to store bus voltage in volts (may be NULL).
 * @param current_a Pointer to store shunt current in amperes (may be NULL).
 * @return ROV_OK if readings are valid, error code otherwise. On error the
 *         caller's outputs are left untouched.
 */
rov_status_t power_read_rail(float *bus_voltage_v, float *current_a);

#ifdef __cplusplus
}
#endif

#endif /* X19_POWER_SERVICE_H */
