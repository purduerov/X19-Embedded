/**
 * @file ina237.c
 * @brief TI INA237AIDGSR Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina237.h"

#include <string.h>

/* INA237 register pointer values. */
#define INA237_REG_ADC_CONFIG 0x01u
#define INA237_REG_VSHUNT     0x04u
#define INA237_REG_VBUS       0x05u
#define INA237_REG_DIE_TEMP   0x06u

/*
 * ADC_CONFIG: continuous bus + shunt + temperature (MODE = 1111), 1052 us
 * conversion time on all three fields, 16-sample average. Reset is 0x0000
 * (shutdown), so this write is what starts the part converting.
 */
#define INA237_ADC_CONFIG_CONTINUOUS 0xFB6Au

/* Default ADCRANGE = 0 conversion factors. */
#define INA237_SHUNT_VOLTAGE_LSB_V 0.000005f /* 5 uV per count */
#define INA237_BUS_VOLTAGE_LSB_V   0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C      0.125f    /* 0.125 C per count */

static rov_status_t ina237_start_conversions(ina237_dev_t *dev) {
    const uint8_t config[2] = {(uint8_t)(INA237_ADC_CONFIG_CONTINUOUS >> 8),
                               (uint8_t)(INA237_ADC_CONFIG_CONTINUOUS & 0xFFu)};

    return rov_i2c_mem_write(dev->bus, dev->i2c_addr, INA237_REG_ADC_CONFIG, config, sizeof(config));
}

rov_status_t ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (dev == NULL || bus == NULL || shunt_resistor_ohms <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ina237_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;

    return ina237_start_conversions(dev);
}

rov_status_t ina237_read_power(ina237_dev_t *dev) {
    uint8_t vbus_raw[2] = {0u, 0u};
    uint8_t vshunt_raw[2] = {0u, 0u};
    uint8_t temp_raw[2] = {0u, 0u};
    uint16_t vbus;
    int16_t vshunt;
    int16_t temp;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_VBUS, vbus_raw, sizeof(vbus_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_VSHUNT, vshunt_raw, sizeof(vshunt_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_DIE_TEMP, temp_raw, sizeof(temp_raw));

    if (status != ROV_OK) {
        return status;
    }

    vbus = (uint16_t)(((uint16_t)vbus_raw[0] << 8) | (uint16_t)vbus_raw[1]);
    vshunt = (int16_t)(((uint16_t)vshunt_raw[0] << 8) | (uint16_t)vshunt_raw[1]);
    temp = (int16_t)(((uint16_t)temp_raw[0] << 8) | (uint16_t)temp_raw[1]);

    dev->bus_voltage_v = (float)vbus * INA237_BUS_VOLTAGE_LSB_V;
    dev->shunt_current_a = ((float)vshunt * INA237_SHUNT_VOLTAGE_LSB_V) / dev->shunt_resistor_ohms;
    dev->power_w = dev->bus_voltage_v * dev->shunt_current_a;

    /* Die temperature is 12-bit two's complement, left-justified in 16 bits. */
    dev->die_temp_c = (float)(int16_t)(temp >> 4) * INA237_DIE_TEMP_LSB_C;

    return ROV_OK;
}
