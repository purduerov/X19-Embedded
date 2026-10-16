/**
 * @file mock_ina226.c
 * @brief INA226 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_ina226.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/ina226.c. */
#define INA226_REG_SHUNT_VOLTAGE   0x01u
#define INA226_REG_BUS_VOLTAGE     0x02u
#define INA226_REG_MANUFACTURER_ID 0xFEu
#define INA226_REG_DIE_ID          0xFFu

#define INA226_MANUFACTURER_ID 0x5449u
#define INA226_DIE_ID          0x2260u

#define INA226_SHUNT_VOLTAGE_LSB_V 0.0000025f /* 2.5 uV per count */
#define INA226_BUS_VOLTAGE_LSB_V   0.00125f   /* 1.25 mV per count */

static int16_t round_to_i16(float value) {
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

void mock_ina226_present(uint8_t addr) {
    mock_bsp_i2c_set_word(addr, INA226_REG_MANUFACTURER_ID, INA226_MANUFACTURER_ID);
    mock_bsp_i2c_set_word(addr, INA226_REG_DIE_ID, INA226_DIE_ID);
}

void mock_ina226_set_reading(uint8_t addr, float bus_voltage_v, float current_a, float shunt_resistor_ohms) {
    uint16_t vbus_counts = (uint16_t)round_to_i16(bus_voltage_v / INA226_BUS_VOLTAGE_LSB_V);
    int16_t vshunt_counts = round_to_i16((current_a * shunt_resistor_ohms) / INA226_SHUNT_VOLTAGE_LSB_V);

    mock_bsp_i2c_set_word(addr, INA226_REG_BUS_VOLTAGE, vbus_counts);
    mock_bsp_i2c_set_word(addr, INA226_REG_SHUNT_VOLTAGE, (uint16_t)vshunt_counts);
}
