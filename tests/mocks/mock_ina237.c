/**
 * @file mock_ina237.c
 * @brief INA237 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_ina237.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/ina237.c. */
#define INA237_REG_VSHUNT   0x04u
#define INA237_REG_VBUS     0x05u
#define INA237_REG_DIE_TEMP 0x06u

/* Default ADCRANGE = 0 conversion factors. */
#define INA237_SHUNT_VOLTAGE_LSB_V 0.000005f /* 5 uV per count */
#define INA237_BUS_VOLTAGE_LSB_V   0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C      0.125f    /* 0.125 C per count */

static int16_t round_to_i16(float value) {
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

void mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms) {
    uint16_t vbus_counts = (uint16_t)round_to_i16(bus_voltage_v / INA237_BUS_VOLTAGE_LSB_V);
    int16_t vshunt_counts = round_to_i16((current_a * shunt_resistor_ohms) / INA237_SHUNT_VOLTAGE_LSB_V);

    mock_bsp_i2c_set_word(addr, INA237_REG_VBUS, vbus_counts);
    mock_bsp_i2c_set_word(addr, INA237_REG_VSHUNT, (uint16_t)vshunt_counts);
}

void mock_ina237_set_die_temperature(uint8_t addr, float temp_c) {
    int16_t counts = round_to_i16(temp_c / INA237_DIE_TEMP_LSB_C);

    /* Die temperature is 12-bit, left-justified in the 16-bit register. */
    mock_bsp_i2c_set_word(addr, INA237_REG_DIE_TEMP, (uint16_t)((uint16_t)counts << 4));
}
