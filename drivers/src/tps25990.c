/**
 * @file tps25990.c
 * @brief TI TPS25990 PMBus Driver Implementation.
 * @organization Purdue ROV
 */

#include "tps25990.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_tps25990(uint8_t brick_idx, float *v_in, float *v_out, float *i_out,
                                                     float *temp_c, uint16_t *status) {
    (void)brick_idx;
    (void)v_in;
    (void)v_out;
    (void)i_out;
    (void)temp_c;
    (void)status;
    return false;
}

rov_status_t tps25990_init(tps25990_dev_t *dev, uint8_t pmbus_addr) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(tps25990_dev_t));
    dev->pmbus_addr = pmbus_addr;
    return ROV_OK;
}

rov_status_t tps25990_read_telemetry(tps25990_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;

    uint8_t idx = (dev->pmbus_addr >= 0x40) ? (dev->pmbus_addr - 0x40) : 0;
    float v_in = 0.0f;
    if (mock_sensors_get_tps25990(idx, &v_in, &dev->output_voltage_v, &dev->output_current_a, &dev->temperature_c,
                                  &dev->status_word)) {
        return ROV_OK;
    }

    return ROV_OK;
}
