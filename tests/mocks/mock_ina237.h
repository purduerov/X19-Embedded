/**
 * @file mock_ina237.h
 * @brief INA237 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Places an INA237 at chosen bus voltage / shunt current / die temperature by
 * writing the raw register words a real part would have produced. The
 * conversion is linear, so the inverse is exact arithmetic.
 */

#ifndef MOCK_INA237_H
#define MOCK_INA237_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the VBUS and VSHUNT registers of the INA237 at @p addr.
 * @param addr 7-bit I2C address, 0x40..0x4F.
 * @param bus_voltage_v Target bus voltage in volts.
 * @param current_a Target shunt current in amperes.
 * @param shunt_resistor_ohms Shunt value the driver under test is configured with.
 */
void mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms);

/**
 * @brief Write the DIETEMP register of the INA237 at @p addr.
 * @param addr 7-bit I2C address.
 * @param temp_c Target die temperature in degrees Celsius.
 */
void mock_ina237_set_die_temperature(uint8_t addr, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_INA237_H */
