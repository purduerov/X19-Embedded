/**
 * @file tmp1075.h
 * @brief TI TMP1075NDRLR PCB Copper Digital Temperature Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef TMP1075_H
#define TMP1075_H

#include "rov_i2c.h"
#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const rov_i2c_bus_t *bus; /* injected transport; set by tmp1075_init() */
    uint8_t i2c_addr;
    float temperature_c;
} tmp1075_dev_t;

/**
 * @brief Bind a TMP1075 at @p i2c_addr to @p bus.
 *
 * The board part is the TMP1075N orderable, which has no device-ID register
 * (the 0x0F ID register exists only on non-N parts), so init cannot verify the
 * chip and validates its arguments only. Presence is proven by a successful
 * temperature read.
 */
rov_status_t tmp1075_init(tmp1075_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr);

rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TMP1075_H */
