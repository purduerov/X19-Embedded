/**
 * @file pmbus_brick.c
 * @brief Mornsun Isolated DC-DC Converter Brick PMBus Driver Implementation.
 * @organization Purdue ROV
 */

#include "pmbus_brick.h"
#include "bsp.h"
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
    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    if (!pmbus_brick_address_valid(pmbus_addr)) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    dev->pmbus_addr = pmbus_addr;
    dev->input_voltage_v = 48.0f;

    if (pmbus_addr == PMBUS_BRICK_ADDR_5V2) {
        dev->output_voltage_v = 5.2f;
    } else {
        dev->output_voltage_v = 12.0f;
    }

    return ROV_OK;
}

rov_status_t pmbus_brick_read_telemetry(pmbus_brick_dev_t *dev) {
    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    uint8_t idx = (uint8_t)(dev->pmbus_addr - PMBUS_BRICK_ADDR_12V_0);

    /*
     * Host SIL path.
     * If mock telemetry is available, use it and avoid hardware I2C.
     */
    if (mock_sensors_get_tps25990(idx, &dev->input_voltage_v, &dev->output_voltage_v, &dev->output_current_a,
                                  &dev->temperature_c, &dev->status_word)) {
        return ROV_OK;
    }

    /*
     * Hardware PMBus path.
     */
    uint16_t raw_vin = 0U;
    uint16_t raw_vout = 0U;
    uint16_t raw_iout = 0U;
    uint16_t raw_temp = 0U;
    uint16_t raw_status = 0U;

    if (!bsp_pmbus_read_word(dev->pmbus_addr, PMBUS_CMD_READ_VIN, &raw_vin)) {
        return ROV_ERROR;
    }

    if (!bsp_pmbus_read_word(dev->pmbus_addr, PMBUS_CMD_READ_VOUT, &raw_vout)) {
        return ROV_ERROR;
    }

    if (!bsp_pmbus_read_word(dev->pmbus_addr, PMBUS_CMD_READ_IOUT, &raw_iout)) {
        return ROV_ERROR;
    }

    if (!bsp_pmbus_read_word(dev->pmbus_addr, PMBUS_CMD_READ_TEMPERATURE1, &raw_temp)) {
        return ROV_ERROR;
    }

    if (!bsp_pmbus_read_word(dev->pmbus_addr, PMBUS_CMD_STATUS_WORD, &raw_status)) {
        return ROV_ERROR;
    }

    /*
     * Convert PMBus telemetry.
     */
    dev->input_voltage_v = pmbus_linear11_to_float(raw_vin);

    dev->output_voltage_v = pmbus_linear16_to_float(raw_vout, -12);

    dev->output_current_a = pmbus_linear11_to_float(raw_iout);

    dev->temperature_c = pmbus_linear11_to_float(raw_temp);

    dev->status_word = raw_status;

    return ROV_OK;
}

bool pmbus_brick_address_valid(uint8_t pmbus_addr) {
    return (pmbus_addr >= PMBUS_BRICK_ADDR_12V_0) && (pmbus_addr <= PMBUS_BRICK_ADDR_5V2);
}

uint8_t pmbus_brick_next_address(uint8_t current_addr) {
    if (!pmbus_brick_address_valid(current_addr)) {
        return PMBUS_BRICK_ADDR_12V_0;
    }

    if (current_addr == PMBUS_BRICK_ADDR_5V2) {
        return PMBUS_BRICK_ADDR_12V_0;
    }

    return (uint8_t)(current_addr + 1U);
}

bool pmbus_status_has_overcurrent(uint16_t status_word) {
    return (status_word & PMBUS_STATUS_IOUT_OC) != 0U;
}

bool pmbus_status_has_overvoltage(uint16_t status_word) {
    return (status_word & PMBUS_STATUS_VOUT_OV) != 0U;
}

bool pmbus_status_has_uvlo(uint16_t status_word) {
    return (status_word & PMBUS_STATUS_VIN_UV) != 0U;
}

bool pmbus_status_has_thermal_fault(uint16_t status_word) {
    return (status_word & PMBUS_STATUS_TEMP) != 0U;
}

float pmbus_linear16_to_float(uint16_t raw_value, int8_t exponent) {
    float result = (float)raw_value;

    if (exponent >= 0) {
        result *= (float)(1UL << exponent);
    } else {
        result /= (float)(1UL << (-exponent));
    }

    return result;
}
