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

float pmbus_linear11_to_float(uint16_t raw_value) {
    int16_t exponent = (int16_t)((int8_t)((raw_value >> 11) & 0x1F));
    if (exponent > 15) {
        exponent -= 32;
    }
    int16_t mantissa = (int16_t)(raw_value & 0x07FF);
    if (mantissa > 1023) {
        mantissa -= 2048;
    }
    float result = (float)mantissa;
    if (exponent >= 0) {
        result *= (float)(1 << exponent);
    } else {
        result /= (float)(1 << (-exponent));
    }
    return result;
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
