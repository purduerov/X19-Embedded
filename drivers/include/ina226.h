/**
 * @file ina226.h
 * @brief TI INA226 Power & Shunt Current Monitor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef INA226_H
#define INA226_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by ina226_init() */
    uint8_t i2c_addr;
    float shunt_resistor_ohms;
    float voltage_v;
    float current_a;
    float power_w;
} ina226_dev_t;

/**
 * @brief Bind an INA226 at @p i2c_addr to @p bus.
 *
 * Unlike the INA237, the INA226 carries a manufacturer ID (0xFE = 0x5449) and a
 * die ID (0xFF = 0x2260 or 0x2261), so init probes both and rejects a part that
 * does not answer as an INA226. The power-on CONFIG default is already
 * continuous shunt + bus conversion, so init writes no configuration.
 */
rov_status_t ina226_init(ina226_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr, float shunt_resistor_ohms);

rov_status_t ina226_read_power(ina226_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* INA226_H */
