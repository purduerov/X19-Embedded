/**
 * @file bme280.c
 * @brief Bosch BME280 Environmental Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include "bsp.h"

#include <stdbool.h>
#include <string.h>

/* BME280 register map */
#define BME280_CHIP_ID 0x60u

#define BME280_REG_CALIB00   0x88u
#define BME280_REG_CHIP_ID   0xD0u
#define BME280_REG_CALIB26   0xE1u
#define BME280_REG_CTRL_HUM  0xF2u
#define BME280_REG_CTRL_MEAS 0xF4u
#define BME280_REG_CONFIG    0xF5u
#define BME280_REG_DATA      0xF7u

/*
 * The raw value every measurement register reads as before a conversion
 * completes.  See the staleness check in bme280_read_all().
 */
#define BME280_RAW_SENTINEL 0x80000

static rov_status_t bme280_check_id(bme280_dev_t *dev) {
    uint8_t id = 0;

    rov_status_t status = bsp_i2c_mem_read(dev->i2c_addr, BME280_REG_CHIP_ID, &id, 1);

    if (status != ROV_OK) {
        return status;
    }

    if (id != BME280_CHIP_ID) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t read_s16_le(const uint8_t *p) {
    return (int16_t)read_u16_le(p);
}

static rov_status_t bme280_read_calibration(bme280_dev_t *dev) {
    uint8_t calib1[26];
    uint8_t calib2[7];
    rov_status_t status;

    status = bsp_i2c_mem_read(dev->i2c_addr, BME280_REG_CALIB00, calib1, sizeof(calib1));

    if (status != ROV_OK) {
        return status;
    }

    /*
     * Temperature calibration coefficients.
     */
    dev->calib.dig_T1 = read_u16_le(&calib1[0]);
    dev->calib.dig_T2 = read_s16_le(&calib1[2]);
    dev->calib.dig_T3 = read_s16_le(&calib1[4]);

    /*
     * Pressure calibration coefficients.
     */
    dev->calib.dig_P1 = read_u16_le(&calib1[6]);
    dev->calib.dig_P2 = read_s16_le(&calib1[8]);
    dev->calib.dig_P3 = read_s16_le(&calib1[10]);
    dev->calib.dig_P4 = read_s16_le(&calib1[12]);
    dev->calib.dig_P5 = read_s16_le(&calib1[14]);
    dev->calib.dig_P6 = read_s16_le(&calib1[16]);
    dev->calib.dig_P7 = read_s16_le(&calib1[18]);
    dev->calib.dig_P8 = read_s16_le(&calib1[20]);
    dev->calib.dig_P9 = read_s16_le(&calib1[22]);

    /*
     * H1 is stored at register 0xA1.
     * Since calib1 begins at 0x88, 0xA1 corresponds to index 25.
     */
    dev->calib.dig_H1 = calib1[25];

    /*
     * Read the remaining humidity calibration coefficients.
     */
    status = bsp_i2c_mem_read(dev->i2c_addr, BME280_REG_CALIB26, calib2, sizeof(calib2));

    if (status != ROV_OK) {
        return status;
    }

    dev->calib.dig_H2 = read_s16_le(&calib2[0]);
    dev->calib.dig_H3 = calib2[2];

    /*
     * H4 and H5 are signed 12-bit values whose bits are split
     * across registers E4, E5, and E6.
     */
    dev->calib.dig_H4 = (int16_t)(((int16_t)(int8_t)calib2[3] << 4) | (calib2[4] & 0x0Fu));

    dev->calib.dig_H5 = (int16_t)(((int16_t)(int8_t)calib2[5] << 4) | (calib2[4] >> 4));

    dev->calib.dig_H6 = (int8_t)calib2[6];

    /*
     * These coefficients are required by the compensation formulas.
     */
    if (dev->calib.dig_T1 == 0u || dev->calib.dig_P1 == 0u) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

static rov_status_t bme280_configure(bme280_dev_t *dev) {
    uint8_t value;
    rov_status_t status;

    /*
     * ctrl_hum:
     * humidity oversampling x1
     */
    value = 0x01u;

    status = bsp_i2c_mem_write(dev->i2c_addr, BME280_REG_CTRL_HUM, &value, 1);

    if (status != ROV_OK) {
        return status;
    }

    /*
     * config:
     * standby time = 0.5 ms
     * IIR filter coefficient = 16
     */
    value = 0x10u;

    status = bsp_i2c_mem_write(dev->i2c_addr, BME280_REG_CONFIG, &value, 1);

    if (status != ROV_OK) {
        return status;
    }

    /*
     * ctrl_meas:
     * temperature oversampling x2
     * pressure oversampling x16
     * normal mode
     *
     * ctrl_hum must be written before ctrl_meas because the
     * humidity configuration is latched when ctrl_meas is written.
     */
    value = 0x57u;

    status = bsp_i2c_mem_write(dev->i2c_addr, BME280_REG_CTRL_MEAS, &value, 1);

    if (status != ROV_OK) {
        return status;
    }

    return ROV_OK;
}

rov_status_t bme280_init(bme280_dev_t *dev) {
    rov_status_t status;

    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    /*
     * Try the primary BME280 I2C address first.
     */
    dev->i2c_addr = BME280_I2C_ADDR_PRIMARY;

    status = bme280_check_id(dev);

    /*
     * If no valid BME280 is found at 0x76, try 0x77.
     */
    if (status != ROV_OK) {
        dev->i2c_addr = BME280_I2C_ADDR_SECONDARY;

        status = bme280_check_id(dev);

        if (status != ROV_OK) {
            return status;
        }
    }

    /*
     * Read the factory calibration coefficients stored in the sensor.
     */
    status = bme280_read_calibration(dev);

    if (status != ROV_OK) {
        return status;
    }

    /*
     * Configure humidity, pressure, temperature, operating mode,
     * and filtering.
     */
    status = bme280_configure(dev);

    if (status != ROV_OK) {
        return status;
    }

    dev->initialized = true;

    return ROV_OK;
}

/*
 * Bosch integer temperature compensation algorithm.
 *
 * Returns temperature in units of 0.01 degrees Celsius.
 *
 * This function also calculates t_fine, which must be calculated
 * before pressure and humidity compensation.
 */
static int32_t bme280_compensate_temperature(bme280_dev_t *dev, int32_t adc_t) {
    int32_t var1;
    int32_t var2;

    var1 = ((((adc_t >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * (int32_t)dev->calib.dig_T2) >> 11;

    var2 = (((((adc_t >> 4) - (int32_t)dev->calib.dig_T1) * ((adc_t >> 4) - (int32_t)dev->calib.dig_T1)) >> 12) *
            (int32_t)dev->calib.dig_T3) >>
           14;

    dev->t_fine = var1 + var2;

    return (dev->t_fine * 5 + 128) >> 8;
}

/*
 * Bosch 64-bit integer pressure compensation algorithm.
 *
 * Returns pressure in Q24.8 format in Pa.
 */
static uint32_t bme280_compensate_pressure(bme280_dev_t *dev, int32_t adc_p) {
    int64_t var1;
    int64_t var2;
    int64_t p;

    var1 = (int64_t)dev->t_fine - 128000;

    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;

    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);

    var2 = var2 + ((int64_t)dev->calib.dig_P4 << 35);

    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);

    var1 = (((((int64_t)1 << 47) + var1) * (int64_t)dev->calib.dig_P1) >> 33);

    /*
     * Protect against division by zero.
     */
    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_p;

    p = (((p << 31) - var2) * 3125) / var1;

    var1 = ((int64_t)dev->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;

    var2 = ((int64_t)dev->calib.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->calib.dig_P7 << 4);

    return (uint32_t)p;
}

/*
 * Bosch humidity compensation, from the datasheet's 64-bit float reference.
 *
 * This is deliberately the floating-point form rather than the datasheet's
 * integer variant.  The integer transcription is widely copied and does not
 * reproduce this reference: measured against the datasheet's own example
 * trimming at 25.08 C, the integer form returns 0.37 %RH where the reference
 * returns 48.59 %RH for adc_H = 30000, and collapses to zero above
 * adc_H = 33000.  An earlier revision of this driver used that transcription
 * and therefore reported a flat ~0.00-0.03 %RH for every input, which silently
 * disabled the humidity half of node1's leak detection.
 *
 * The float form is the datasheet's normative algorithm, so it is the one that
 * can be pinned to a published vector: tests/test_driver_bme280.c asserts
 * 48.5-48.7 %RH for that same example.  The cost is a handful of FPU
 * operations per poll, which is irrelevant at node1's 10 Hz rate.
 *
 * Returns percent relative humidity, already clamped to [0, 100].
 */
static float bme280_compensate_humidity(bme280_dev_t *dev, int32_t adc_h) {
    double var_h;

    var_h = (double)dev->t_fine / 5120.0;
    var_h = var_h - 76800.0;

    var_h = ((double)adc_h - (((double)dev->calib.dig_H4 * 64.0) + (((double)dev->calib.dig_H5 / 16384.0) * var_h))) *
            ((double)dev->calib.dig_H2 / 65536.0);

    var_h = var_h * (1.0 + (((double)dev->calib.dig_H3 / 67108864.0) * var_h));

    var_h = var_h * (1.0 - (((double)dev->calib.dig_H1 * var_h) / 524288.0));

    if (var_h > 100.0) {
        var_h = 100.0;
    }

    if (var_h < 0.0) {
        var_h = 0.0;
    }

    return (float)var_h;
}

rov_status_t bme280_read_all(bme280_dev_t *dev) {
    uint8_t data[8];
    rov_status_t status;
    int32_t adc_p;
    int32_t adc_t;
    int32_t adc_h;
    int32_t temp_x100;
    uint32_t pressure_q24_8;
    float humidity_pct;

    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return ROV_ERROR;
    }

    /*
     * Read all measurement registers in one transaction:
     *
     * F7-F9: pressure
     * FA-FC: temperature
     * FD-FE: humidity
     */
    status = bsp_i2c_mem_read(dev->i2c_addr, BME280_REG_DATA, data, sizeof(data));

    if (status != ROV_OK) {
        return status;
    }

    /*
     * Pressure and temperature are stored as 20-bit values.
     * Humidity is stored as a 16-bit value.
     */
    adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    adc_h = ((int32_t)data[6] << 8) | (int32_t)data[7];

    /*
     * A skipped conversion leaves every raw register at the power-on sentinel
     * 0x80000, and compensating that yields a plausible-looking but entirely
     * fictitious reading.  The first poll after bme280_init() always hits this,
     * because configuring oversampling starts a conversion that has not
     * finished yet -- so without this check the first telemetry frame after
     * boot carries garbage that the leak thresholds downstream would then
     * evaluate.
     *
     * Reporting an error leaves the caller's existing validity flag false for
     * this cycle, which is the path node1 already has for a sensor it could not
     * read.  Returning stale-but-valid numbers instead would let a leak
     * threshold be evaluated against a sample that was never taken.
     */
    if (adc_t == BME280_RAW_SENTINEL || adc_p == BME280_RAW_SENTINEL || adc_h == BME280_RAW_SENTINEL) {
        return ROV_ERROR;
    }

    /*
     * Temperature must be calculated first because it produces
     * t_fine, which pressure and humidity compensation require.
     */
    temp_x100 = bme280_compensate_temperature(dev, adc_t);

    pressure_q24_8 = bme280_compensate_pressure(dev, adc_p);

    humidity_pct = bme280_compensate_humidity(dev, adc_h);

    /*
     * The Bosch reference returns 0 from the pressure routine when its
     * intermediate divisor collapses, which for a 20-bit raw value means the
     * calibration is unusable rather than that the pressure is zero.  0 hPa is
     * not a reading a surface vessel can produce, so reporting it as a success
     * would hand a caller a number that looks measured and is not; fail
     * instead and let the caller keep its previous sample.
     */
    if (pressure_q24_8 == 0u) {
        return ROV_ERROR;
    }

    /*
     * Convert Bosch's integer output formats to the units used
     * by the rest of the ROV software.
     */
    dev->temperature_c = (float)temp_x100 / 100.0f;

    dev->pressure_hpa = (float)pressure_q24_8 / 25600.0f;

    dev->humidity_pct = humidity_pct;

    return ROV_OK;
}
