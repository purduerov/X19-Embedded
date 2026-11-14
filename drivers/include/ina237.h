/**
 * @file ina237.h
 * @brief TI INA237AIDGSR High-Precision Power Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA237_H
#define INA237_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by ina237_init() */
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float current_lsb_a;
    float bus_voltage_v;
    float shunt_current_a;
    float power_w;
    float die_temp_c;
} ina237_dev_t;

/**
 * @brief Bind an INA237 at @p i2c_addr to @p bus and configure its ADC.
 *
 * The INA237 has no device-ID register (the 0x3F ID register exists on the
 * INA238 only), so init cannot verify the chip and validates its arguments
 * only. It writes ADC_CONFIG to select 16-sample averaging; the power-on
 * reset (0xFB68) is already continuous at 1052 us, averaging one sample.
 */
rov_status_t ina237_init(ina237_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms);

rov_status_t ina237_read_power(ina237_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA237_H */
