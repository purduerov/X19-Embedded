/**
 * @file ms5837.c
 * @brief TE Connectivity MS5837-30BA Hydrostatic Pressure & Depth Driver Implementation.
 * @organization Purdue ROV
 */

#include "ms5837.h"
#include "bsp.h"
#include <stdbool.h>
#include <string.h>

#define MS5837_I2C_ADDR             0x76U
#define MS5837_CMD_RESET            0x1EU
#define MS5837_CMD_ADC_READ         0x00U
#define MS5837_CMD_CONVERT_D1       0x48U
#define MS5837_CMD_CONVERT_D2       0x58U
#define MS5837_CMD_PROM_BASE        0xA0U
#define MS5837_CONVERSION_TIME_MS   20U
#define MS5837_02BA_30BA_SEPARATION 37000U

#define MS5837_ATMOSPHERIC_MBAR 1013.25f
#define MS5837_GRAVITY_M_S2     9.80665f

__attribute__((weak)) bool mock_sensors_get_ms5837(float *pressure_mbar, float *temp_c) {
    (void)pressure_mbar;
    (void)temp_c;
    return false;
}

static bool ms5837_write_command(uint8_t command) {
    return bsp_i2c_write(MS5837_I2C_ADDR, &command, 1U);
}

static bool ms5837_read_prom_word(uint8_t index, uint16_t *value) {
    uint8_t command = (uint8_t)(MS5837_CMD_PROM_BASE + (index * 2U));
    uint8_t data[2];

    if (!value) {
        return false;
    }

    if (!bsp_i2c_write(MS5837_I2C_ADDR, &command, 1U)) {
        return false;
    }

    if (!bsp_i2c_read(MS5837_I2C_ADDR, data, 2U)) {
        return false;
    }

    *value = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);

    return true;
}

static bool ms5837_read_adc(uint32_t *value) {
    uint8_t command = MS5837_CMD_ADC_READ;
    uint8_t data[3] = {0};

    if (!value) {
        return false;
    }

    if (!bsp_i2c_write(MS5837_I2C_ADDR, &command, 1U)) {
        return false;
    }

    if (!bsp_i2c_read(MS5837_I2C_ADDR, data, 3U)) {
        return false;
    }

    uint32_t adc = ((uint32_t)data[0] << 16U) | ((uint32_t)data[1] << 8U) | (uint32_t)data[2];
    if (adc == 0U) {
        return false;
    }

    *value = adc;

    return true;
}

static uint8_t ms5837_crc4(const uint16_t prom[8]) {
    uint16_t prom_copy[8];
    uint16_t remainder = 0U;

    memcpy(prom_copy, prom, sizeof(prom_copy));

    prom_copy[0] &= 0x0FFFU;
    prom_copy[7] = 0U;

    for (uint8_t cnt = 0U; cnt < 16U; cnt++) {
        if ((cnt & 1U) != 0U) {
            remainder ^= (uint16_t)(prom_copy[cnt >> 1U] & 0x00FFU);
        } else {
            remainder ^= (uint16_t)(prom_copy[cnt >> 1U] >> 8U);
        }

        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((remainder & 0x8000U) != 0U) {
                remainder = (uint16_t)((remainder << 1U) ^ 0x3000U);
            } else {
                remainder <<= 1U;
            }
        }
    }

    return (uint8_t)((remainder >> 12U) & 0x0FU);
}

static void ms5837_calculate_depth(ms5837_dev_t *dev, float fluid_density_kg_m3) {
    float delta_p_pa = (dev->pressure_mbar * 100.0f) - (MS5837_ATMOSPHERIC_MBAR * 100.0f);

    if (delta_p_pa <= 0.0f) {
        dev->depth_meters = 0.0f;
        return;
    }

    if (dev->cached_fluid_density != fluid_density_kg_m3) {
        dev->cached_fluid_density = fluid_density_kg_m3;
        dev->inv_rho_g = 1.0f / (fluid_density_kg_m3 * MS5837_GRAVITY_M_S2);
    }

    dev->depth_meters = delta_p_pa * dev->inv_rho_g;
}

static void ms5837_compensate(ms5837_dev_t *dev) {
    int32_t d_t = (int32_t)dev->raw_temperature - ((int32_t)dev->cal_coeffs[5] << 8U);
    int32_t temperature = 2000 + (int32_t)(((int64_t)d_t * (int64_t)dev->cal_coeffs[6]) >> 23U);

    int64_t offset;
    int64_t sensitivity;

    if (dev->is_model_02ba) {
        offset = ((int64_t)dev->cal_coeffs[2] << 17U) + (((int64_t)dev->cal_coeffs[4] * (int64_t)d_t) >> 6U);
        sensitivity = ((int64_t)dev->cal_coeffs[1] << 16U) + (((int64_t)dev->cal_coeffs[3] * (int64_t)d_t) >> 7U);
    } else {
        offset = ((int64_t)dev->cal_coeffs[2] << 16U) + (((int64_t)dev->cal_coeffs[4] * (int64_t)d_t) >> 7U);
        sensitivity = ((int64_t)dev->cal_coeffs[1] << 15U) + (((int64_t)dev->cal_coeffs[3] * (int64_t)d_t) >> 8U);
    }

    int64_t temp_correction = 0;
    int64_t offset_correction = 0;
    int64_t sensitivity_correction = 0;
    int64_t d_t_sq = (int64_t)d_t * (int64_t)d_t;

    if (dev->is_model_02ba) {
        if (temperature < 2000) {
            int64_t temp_delta = (int64_t)temperature - 2000LL;
            int64_t temp_delta_sq = temp_delta * temp_delta;
            temp_correction = (11LL * d_t_sq) >> 35U;
            offset_correction = (31LL * temp_delta_sq) >> 3U;
            sensitivity_correction = (63LL * temp_delta_sq) >> 5U;
        }
    } else {
        if (temperature < 2000) {
            int64_t temp_delta = (int64_t)temperature - 2000LL;
            int64_t temp_delta_sq = temp_delta * temp_delta;

            temp_correction = (3LL * d_t_sq) >> 33U;
            offset_correction = (3LL * temp_delta_sq) >> 1U;
            sensitivity_correction = (5LL * temp_delta_sq) >> 3U;

            if (temperature < -1500) {
                int64_t very_low_delta = (int64_t)temperature + 1500LL;
                int64_t very_low_delta_sq = very_low_delta * very_low_delta;

                offset_correction += 7LL * very_low_delta_sq;
                sensitivity_correction += 4LL * very_low_delta_sq;
            }
        } else {
            int64_t temp_delta = (int64_t)temperature - 2000LL;
            temp_correction = (2LL * d_t_sq) >> 37U;
            offset_correction = (temp_delta * temp_delta) >> 4U;
            sensitivity_correction = 0LL;
        }
    }

    temperature -= (int32_t)temp_correction;
    offset -= offset_correction;
    sensitivity -= sensitivity_correction;

    dev->temperature_c = (float)temperature / 100.0f;

    if (dev->is_model_02ba) {
        int32_t p = (int32_t)(((((int64_t)dev->raw_pressure * sensitivity) >> 21U) - offset) >> 15U);
        dev->pressure_mbar = (float)p / 100.0f;
    } else {
        int32_t pressure_x10 = (int32_t)(((((int64_t)dev->raw_pressure * sensitivity) >> 21U) - offset) >> 13U);
        dev->pressure_mbar = (float)pressure_x10 / 10.0f;
    }
}

rov_status_t ms5837_init(ms5837_dev_t *dev) {
    if (!dev) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    dev->cached_fluid_density = -1.0f;
    dev->state = MS5837_STATE_IDLE;

    float mock_pressure;
    float mock_temperature;

    if (mock_sensors_get_ms5837(&mock_pressure, &mock_temperature)) {
        dev->initialized = true;
        return ROV_OK;
    }

    if (!ms5837_write_command(MS5837_CMD_RESET)) {
        return ROV_ERROR;
    }

    /*
     * Reset is only performed during initialization.
     * D1/D2 conversions below remain nonblocking.
     */
    delay_ms(10U);

    for (uint8_t i = 0U; i < 7U; i++) {
        if (!ms5837_read_prom_word(i, &dev->cal_coeffs[i])) {
            return ROV_ERROR;
        }
    }

    uint8_t expected_crc = (uint8_t)(dev->cal_coeffs[0] >> 12U);
    uint8_t calculated_crc = ms5837_crc4(dev->cal_coeffs);

    if (expected_crc != calculated_crc) {
        return ROV_ERR_CRC_MISMATCH;
    }

    /* Auto-detect MS5837-02BA vs MS5837-30BA based on PROM Word 1 sensitivity */
    dev->is_model_02ba = (dev->cal_coeffs[1] > MS5837_02BA_30BA_SEPARATION);
    dev->initialized = true;

    return ROV_OK;
}

rov_status_t ms5837_read_pressure_depth(ms5837_dev_t *dev, float fluid_density_kg_m3) {
    if (!dev || fluid_density_kg_m3 <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    if (mock_sensors_get_ms5837(&dev->pressure_mbar, &dev->temperature_c)) {
        uint32_t now = time_get_ms();
        switch (dev->state) {
        case MS5837_STATE_IDLE:
            dev->conversion_start_ms = now;
            dev->state = MS5837_STATE_WAIT_D1;
            return ROV_BUSY;
        case MS5837_STATE_WAIT_D1:
            if ((uint32_t)(now - dev->conversion_start_ms) < MS5837_CONVERSION_TIME_MS) {
                return ROV_BUSY;
            }
            dev->conversion_start_ms = now;
            dev->state = MS5837_STATE_WAIT_D2;
            return ROV_BUSY;
        case MS5837_STATE_WAIT_D2:
            if ((uint32_t)(now - dev->conversion_start_ms) < MS5837_CONVERSION_TIME_MS) {
                return ROV_BUSY;
            }
            ms5837_calculate_depth(dev, fluid_density_kg_m3);
            dev->state = MS5837_STATE_IDLE;
            return ROV_OK;
        default:
            dev->state = MS5837_STATE_IDLE;
            return ROV_ERROR;
        }
    }

    if (!dev->initialized) {
        return ROV_ERROR;
    }

    uint32_t now = time_get_ms();

    switch (dev->state) {
    case MS5837_STATE_IDLE:
        if (!ms5837_write_command(MS5837_CMD_CONVERT_D1)) {
            return ROV_ERROR;
        }

        dev->conversion_start_ms = now;
        dev->state = MS5837_STATE_WAIT_D1;
        return ROV_BUSY;

    case MS5837_STATE_WAIT_D1:
        if ((uint32_t)(now - dev->conversion_start_ms) < MS5837_CONVERSION_TIME_MS) {
            return ROV_BUSY;
        }

        if (!ms5837_read_adc(&dev->raw_pressure)) {
            dev->state = MS5837_STATE_IDLE;
            return ROV_ERROR;
        }

        if (!ms5837_write_command(MS5837_CMD_CONVERT_D2)) {
            dev->state = MS5837_STATE_IDLE;
            return ROV_ERROR;
        }

        dev->conversion_start_ms = now;
        dev->state = MS5837_STATE_WAIT_D2;
        return ROV_BUSY;

    case MS5837_STATE_WAIT_D2:
        if ((uint32_t)(now - dev->conversion_start_ms) < MS5837_CONVERSION_TIME_MS) {
            return ROV_BUSY;
        }

        if (!ms5837_read_adc(&dev->raw_temperature)) {
            dev->state = MS5837_STATE_IDLE;
            return ROV_ERROR;
        }

        ms5837_compensate(dev);
        ms5837_calculate_depth(dev, fluid_density_kg_m3);

        dev->state = MS5837_STATE_IDLE;
        return ROV_OK;

    default:
        dev->state = MS5837_STATE_IDLE;
        return ROV_ERROR;
    }
}

rov_status_t ms5837_read_sync(ms5837_dev_t *dev, float fluid_density_kg_m3) {
    if (!dev || fluid_density_kg_m3 <= 0.0f) {
        return ROV_ERR_INVALID_ARG;
    }

    if (mock_sensors_get_ms5837(&dev->pressure_mbar, &dev->temperature_c)) {
        ms5837_calculate_depth(dev, fluid_density_kg_m3);
        return ROV_OK;
    }

    if (!dev->initialized) {
        return ROV_ERROR;
    }

    /* 1. Request D1 (Pressure) conversion */
    if (!ms5837_write_command(MS5837_CMD_CONVERT_D1)) {
        return ROV_ERROR;
    }
    delay_ms(MS5837_CONVERSION_TIME_MS);

    /* 2. Read D1 ADC result (24 bits) */
    if (!ms5837_read_adc(&dev->raw_pressure)) {
        return ROV_ERROR;
    }

    /* 3. Request D2 (Temperature) conversion */
    if (!ms5837_write_command(MS5837_CMD_CONVERT_D2)) {
        return ROV_ERROR;
    }
    delay_ms(MS5837_CONVERSION_TIME_MS);

    /* 4. Read D2 ADC result (24 bits) */
    if (!ms5837_read_adc(&dev->raw_temperature)) {
        return ROV_ERROR;
    }

    /* 5. Compensate and calculate depth */
    ms5837_compensate(dev);
    ms5837_calculate_depth(dev, fluid_density_kg_m3);

    dev->state = MS5837_STATE_IDLE;
    return ROV_OK;
}
