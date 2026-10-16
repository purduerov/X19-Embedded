/**
 * @file ina226.c
 * @brief TI INA226 Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina226.h"

#include <string.h>

/* INA226 register pointer values. */
#define INA226_REG_SHUNT_VOLTAGE   0x01u
#define INA226_REG_BUS_VOLTAGE     0x02u
#define INA226_REG_MANUFACTURER_ID 0xFEu
#define INA226_REG_DIE_ID          0xFFu

#define INA226_MANUFACTURER_ID 0x5449u
#define INA226_DIE_ID_A        0x2260u
#define INA226_DIE_ID_B        0x2261u

/* Fixed conversion factors. */
#define INA226_SHUNT_VOLTAGE_LSB_V 0.0000025f /* 2.5 uV per count */
#define INA226_BUS_VOLTAGE_LSB_V   0.00125f   /* 1.25 mV per count */

static uint16_t read_u16_be(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static rov_status_t ina226_check_id(ina226_dev_t *dev) {
    uint8_t manufacturer_raw[2] = {0u, 0u};
    uint8_t die_raw[2] = {0u, 0u};
    uint16_t die_id;

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_MANUFACTURER_ID, manufacturer_raw,
                                           sizeof(manufacturer_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_DIE_ID, die_raw, sizeof(die_raw));

    if (status != ROV_OK) {
        return status;
    }

    if (read_u16_be(manufacturer_raw) != INA226_MANUFACTURER_ID) {
        return ROV_ERROR;
    }

    die_id = read_u16_be(die_raw);

    if (die_id != INA226_DIE_ID_A && die_id != INA226_DIE_ID_B) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

rov_status_t ina226_init(ina226_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (dev == NULL || bus == NULL || shunt_resistor_ohms <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ina226_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;

    return ina226_check_id(dev);
}

rov_status_t ina226_read_power(ina226_dev_t *dev) {
    uint8_t vbus_raw[2] = {0u, 0u};
    uint8_t vshunt_raw[2] = {0u, 0u};
    uint16_t vbus;
    int16_t vshunt;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_BUS_VOLTAGE, vbus_raw, sizeof(vbus_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA226_REG_SHUNT_VOLTAGE, vshunt_raw, sizeof(vshunt_raw));

    if (status != ROV_OK) {
        return status;
    }

    vbus = read_u16_be(vbus_raw);
    vshunt = (int16_t)read_u16_be(vshunt_raw);

    dev->voltage_v = (float)vbus * INA226_BUS_VOLTAGE_LSB_V;
    dev->current_a = ((float)vshunt * INA226_SHUNT_VOLTAGE_LSB_V) / dev->shunt_resistor_ohms;
    dev->power_w = dev->voltage_v * dev->current_a;

    return ROV_OK;
}
