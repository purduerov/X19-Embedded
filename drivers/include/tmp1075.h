/**
 * @file tmp1075.h
 * @brief TI TMP1075NDRLR PCB Copper Digital Temperature Sensor I2C Driver.
 * @organization Purdue ROV
 */

#ifndef TMP1075_H
#define TMP1075_H

#include "rov_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t i2c_addr;
    float temperature_c;
} tmp1075_dev_t;

rov_status_t tmp1075_init(tmp1075_dev_t *dev, uint8_t i2c_addr);
rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TMP1075_H */
