/**
 * @file tps25990.h
 * @brief TI TPS25990 PMBus / I2C eFuse & Brick Telemetry Driver.
 * @organization Purdue ROV
 */

#ifndef TPS25990_H
#define TPS25990_H

#include "x19_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t pmbus_addr;
    float output_voltage_v;
    float output_current_a;
    float temperature_c;
    uint16_t status_word;
} tps25990_dev_t;

x19_status_t tps25990_init(tps25990_dev_t *dev, uint8_t pmbus_addr);
x19_status_t tps25990_read_telemetry(tps25990_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* TPS25990_H */
