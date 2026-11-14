/**
 * @file ina237.c
 * @brief TI INA237AIDGSR Power Monitor Driver Implementation.
 * @organization Purdue ROV
 */

#include "ina237.h"

#include <string.h>

/* INA237 register pointer values. */
#define INA237_REG_ADC_CONFIG 0x01u
#define INA237_REG_SHUNT_CAL  0x02u
#define INA237_REG_VBUS       0x05u
#define INA237_REG_DIE_TEMP   0x06u
#define INA237_REG_CURRENT    0x07u
#define INA237_REG_POWER      0x08u

/*
 * ADC_CONFIG: continuous bus + shunt + temperature (MODE = 1111), 1052 us
 * conversion time on all three fields, 4-sample average.
 */
#define INA237_ADC_CONFIG_CONTINUOUS 0xFB69u

/*
 * Use a fixed 0.5 mA/LSB current scale as requested for the 1 mOhm rail
 * shunt. INA237's calibration equation is:
 *
 *   SHUNT_CAL = 819.2e6 * CURRENT_LSB * R_SHUNT
 */
#define INA237_CURRENT_LSB_A 0.0005f

/* Default ADCRANGE = 0 conversion factors. */
#define INA237_BUS_VOLTAGE_LSB_V   0.003125f /* 3.125 mV per count */
#define INA237_DIE_TEMP_LSB_C      0.125f    /* 0.125 C per count */

static rov_status_t ina237_configure_adc(ina237_dev_t *dev) {
    const uint8_t config[2] = {(uint8_t)(INA237_ADC_CONFIG_CONTINUOUS >> 8),
                               (uint8_t)(INA237_ADC_CONFIG_CONTINUOUS & 0xFFu)};

    return rov_i2c_mem_write(dev->bus, dev->i2c_addr, INA237_REG_ADC_CONFIG, config, sizeof(config));
}

static rov_status_t ina237_configure_calibration(ina237_dev_t *dev) {
    float calibration = 819200000.0f * dev->current_lsb_a * dev->shunt_resistor_ohms;
    uint32_t calibration_rounded;
    uint8_t data[2];

    if (!(calibration >= 1.0f) || calibration > 65535.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    calibration_rounded = (uint32_t)(calibration + 0.5f);
    data[0] = (uint8_t)(calibration_rounded >> 8);
    data[1] = (uint8_t)(calibration_rounded & 0xFFu);

    return rov_i2c_mem_write(dev->bus, dev->i2c_addr, INA237_REG_SHUNT_CAL, data, sizeof(data));
}

rov_status_t ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms) {
    if (dev == NULL || bus == NULL || shunt_resistor_ohms <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ina237_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    dev->current_lsb_a = INA237_CURRENT_LSB_A;

    rov_status_t status = ina237_configure_adc(dev);
    if (status != ROV_OK) {
        return status;
    }

    return ina237_configure_calibration(dev);
}

rov_status_t ina237_read_power(ina237_dev_t *dev) {
    uint8_t vbus_raw[2] = {0u, 0u};
    uint8_t current_raw[2] = {0u, 0u};
    uint8_t power_raw[2] = {0u, 0u};
    uint8_t temp_raw[2] = {0u, 0u};
    uint16_t vbus;
    int16_t current;
    uint16_t power;
    int16_t temp;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_VBUS, vbus_raw, sizeof(vbus_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_CURRENT, current_raw, sizeof(current_raw));

    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_POWER, power_raw, sizeof(power_raw));
    if (status != ROV_OK) {
        return status;
    }

    status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, INA237_REG_DIE_TEMP, temp_raw, sizeof(temp_raw));

    if (status != ROV_OK) {
        return status;
    }

    vbus = (uint16_t)(((uint16_t)vbus_raw[0] << 8) | (uint16_t)vbus_raw[1]);
    current = (int16_t)(((uint16_t)current_raw[0] << 8) | (uint16_t)current_raw[1]);
    power = (uint16_t)(((uint16_t)power_raw[0] << 8) | (uint16_t)power_raw[1]);
    temp = (int16_t)(((uint16_t)temp_raw[0] << 8) | (uint16_t)temp_raw[1]);

    dev->bus_voltage_v = (float)vbus * INA237_BUS_VOLTAGE_LSB_V;
    dev->shunt_current_a = (float)current * dev->current_lsb_a;
    dev->power_w = (float)power * (25.0f * dev->current_lsb_a);

    /* Die temperature is 12-bit two's complement, left-justified in 16 bits. */
    dev->die_temp_c = (float)(int16_t)(temp >> 4) * INA237_DIE_TEMP_LSB_C;

    return ROV_OK;
}
