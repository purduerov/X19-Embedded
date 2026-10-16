/**
 * @file mock_ina226.h
 * @brief INA226 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#ifndef MOCK_INA226_H
#define MOCK_INA226_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the manufacturer and die ID registers so ina226_init() succeeds.
 * @param addr 7-bit I2C address, 0x40..0x4F.
 */
void mock_ina226_present(uint8_t addr);

/**
 * @brief Write the VBUS and VSHUNT registers of the INA226 at @p addr.
 * @param addr 7-bit I2C address.
 * @param bus_voltage_v Target bus voltage in volts.
 * @param current_a Target shunt current in amperes.
 * @param shunt_resistor_ohms Shunt value the driver under test is configured with.
 */
void mock_ina226_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_INA226_H */
