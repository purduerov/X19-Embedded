/**
 * @file pmbus_brick.c
 * @brief Mornsun Isolated DC-DC Converter Brick PMBus Driver Implementation.
 * @organization Purdue ROV
 */

#include "pmbus_brick.h"
#include <stdbool.h>
#include <string.h>

__attribute__((weak)) bool mock_sensors_get_tps25990(uint8_t index, float *v_in, float *v_out, float *i_out,
                                                     float *temp_c, uint16_t *status) {
    (void)index;
    (void)v_in;
    (void)v_out;
    (void)i_out;
    (void)temp_c;
    (void)status;
    return false;
}

/* Performance optimization: PMBus Linear11 encoding uses a 5-bit exponent.
 * Instead of branching and executing a slow FPU division for negative exponents
 * (~14 cycles on Cortex-M4), we precompute the 2^N multipliers for all 32 possible
 * 5-bit exponent values. This reduces the operation to an O(1) array lookup
 * and a fast single-cycle FPU multiplication. */
static const float pmbus_exp_lut[32] = {1.0f,
                                        2.0f,
                                        4.0f,
                                        8.0f,
                                        16.0f,
                                        32.0f,
                                        64.0f,
                                        128.0f,
                                        256.0f,
                                        512.0f,
                                        1024.0f,
                                        2048.0f,
                                        4096.0f,
                                        8192.0f,
                                        16384.0f,
                                        32768.0f,
                                        1.52587890625e-05f,
                                        3.0517578125e-05f,
                                        6.103515625e-05f,
                                        0.0001220703125f,
                                        0.000244140625f,
                                        0.00048828125f,
                                        0.0009765625f,
                                        0.001953125f,
                                        0.00390625f,
                                        0.0078125f,
                                        0.015625f,
                                        0.03125f,
                                        0.0625f,
                                        0.125f,
                                        0.25f,
                                        0.5f};

float pmbus_linear11_to_float(uint16_t raw_value) {
    uint8_t exp_index = (raw_value >> 11) & 0x1F;
    int16_t mantissa = (int16_t)(raw_value & 0x07FF);
    if (mantissa > 1023) {
        mantissa -= 2048;
    }
    return (float)mantissa * pmbus_exp_lut[exp_index];
}

rov_status_t pmbus_brick_init(pmbus_brick_dev_t *dev, uint8_t pmbus_addr) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(pmbus_brick_dev_t));
    dev->pmbus_addr = pmbus_addr;
    dev->input_voltage_v = 48.0f;
    dev->output_voltage_v = (pmbus_addr == 0x40) ? 5.2f : 12.0f;
    return ROV_OK;
}

rov_status_t pmbus_brick_read_telemetry(pmbus_brick_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    uint8_t idx = (dev->pmbus_addr >= 0x40) ? (uint8_t)(dev->pmbus_addr - 0x40) : 0;
    if (mock_sensors_get_tps25990(idx, &dev->input_voltage_v, &dev->output_voltage_v, &dev->output_current_a,
                                  &dev->temperature_c, &dev->status_word)) {
        return ROV_OK;
    }
    return ROV_OK;
}
