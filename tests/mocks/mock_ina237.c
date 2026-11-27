/**
 * @file mock_ina237.c
 * @brief INA237 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_ina237.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/ina237.c. */
#define INA237_REG_VBUS     0x05u
#define INA237_REG_DIE_TEMP 0x06u
#define INA237_REG_CURRENT  0x07u
#define INA237_REG_POWER    0x08u

#define INA237_BUS_VOLTAGE_LSB_V 0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C    0.125f    /* 0.125 C per count */
#define INA237_CURRENT_LSB_A     0.0005f

static int16_t round_to_i16(float value) {
    if (value >= 32767.0f) {
        return INT16_MAX;
    }

    if (value <= -32768.0f) {
        return INT16_MIN;
    }

    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static uint32_t round_to_u24(float value) {
    if (value <= 0.0f) {
        return 0u;
    }

    if (value >= 16777215.0f) {
        return 0xFFFFFFu;
    }

    return (uint32_t)(value + 0.5f);
}

void mock_ina237_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms) {
    uint16_t vbus_counts = (uint16_t)round_to_i16(bus_voltage_v / INA237_BUS_VOLTAGE_LSB_V);
    int16_t current_counts = round_to_i16(current_a / INA237_CURRENT_LSB_A);
    uint32_t power_counts = round_to_u24((bus_voltage_v * current_a) / (0.2f * INA237_CURRENT_LSB_A));

    mock_bsp_i2c_set_word(addr, INA237_REG_VBUS, vbus_counts);
    mock_bsp_i2c_set_word(addr, INA237_REG_CURRENT, (uint16_t)current_counts);
    mock_bsp_i2c_set_word(addr, INA237_REG_POWER, (uint16_t)(power_counts >> 8));
    mock_bsp_i2c_set_word(addr, (uint8_t)(INA237_REG_POWER + 1u), (uint16_t)((power_counts & 0xFFu) << 8));
    (void)shunt_resistor_ohms;
}

void mock_ina237_set_die_temperature(uint8_t addr, float temp_c) {
    int16_t counts = round_to_i16(temp_c / INA237_DIE_TEMP_LSB_C);

    /* Die temperature is 12-bit, left-justified in the 16-bit register. */
    mock_bsp_i2c_set_word(addr, INA237_REG_DIE_TEMP, (uint16_t)((uint16_t)counts << 4));
}
