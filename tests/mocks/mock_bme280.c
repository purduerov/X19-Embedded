/**
 * @file mock_bme280.c
 * @brief BME280 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Bosch's compensation formulas are transcribed here so the model can be
 * inverted: a test asks for 1013.25 hPa and this works out which raw ADC words
 * a real BME280, trimmed with the datasheet's example coefficients, would have
 * produced. The driver under test then performs its own compensation on those
 * words.
 *
 * The humidity reference is the datasheet's 64-bit float form rather than the
 * widely copied integer transcription, because the integer form does not
 * reproduce it -- see the note on humidity_pct_from_raw() below.
 */

#include "mock_bme280.h"

#include "mock_bsp.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Register map, matching drivers/src/bme280.c. */
#define BME280_CHIP_ID_REG 0xD0u
#define BME280_CALIB00_REG 0x88u
#define BME280_CALIB26_REG 0xE1u
#define BME280_DATA_REG    0xF7u
#define BME280_CHIP_ID     0x60u

/* The power-on / disabled value the part reports before a conversion. */
#define BME280_RAW_SENTINEL 0x80000L

/*
 * Bosch's datasheet worked example, used as this model's trimming.
 *
 * Typed via explicit casts where the coefficient is written out below: the
 * pressure block mixes unsigned values above INT16_MAX (dig_P1 = 36477) with
 * negative ones, and an `int` literal that silently narrows is a different
 * sensor than the datasheet describes.
 */
#define DIG_T1 27504
#define DIG_T2 26435
#define DIG_T3 (-1000)
#define DIG_P1 36477
#define DIG_P2 (-10685)
#define DIG_P3 3024
#define DIG_P4 2855
#define DIG_P5 140
#define DIG_P6 (-7)
#define DIG_P7 15500
#define DIG_P8 (-14600)
#define DIG_P9 6000
#define DIG_H1 75
#define DIG_H2 362
#define DIG_H3 0
#define DIG_H4 334
#define DIG_H5 50
#define DIG_H6 30

static int32_t compensate_t_fine(int32_t adc_t) {
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)DIG_T1 << 1))) * (int32_t)DIG_T2) >> 11;
    int32_t var2 =
        (((((adc_t >> 4) - (int32_t)DIG_T1) * ((adc_t >> 4) - (int32_t)DIG_T1)) >> 12) * (int32_t)DIG_T3) >> 14;

    return var1 + var2;
}

static float temperature_c_from_t_fine(int32_t t_fine) {
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

/*
 * Bosch pressure compensation. Note the direction: the compensated pressure
 * DECREASES as the raw word increases (adc_P = 0 is the high-pressure end of
 * the range, adc_P = 415148 is about 1006 hPa with this trimming). The
 * inversion below depends on that.
 */
static float pressure_hpa_from_raw(int32_t adc_p, int32_t t_fine) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)DIG_P6;
    int64_t p;

    var2 = var2 + ((var1 * (int64_t)DIG_P5) << 17);
    var2 = var2 + ((int64_t)DIG_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)DIG_P3) >> 8) + ((var1 * (int64_t)DIG_P2) << 12);
    var1 = (((((int64_t)1 << 47) + var1) * (int64_t)DIG_P1) >> 33);

    if (var1 == 0) {
        return 0.0f;
    }

    p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = ((int64_t)DIG_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)DIG_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)DIG_P7 << 4);

    if (p < 0) {
        p = 0;
    }

    return (float)(uint32_t)p / 25600.0f;
}

/*
 * Datasheet 64-bit float humidity reference, matching the driver's
 * bme280_compensate_humidity().
 *
 * Not the integer transcription that circulates in most BME280 drivers: with
 * this example trimming at 25.08 C, that form returns 0.37 %RH for adc_H =
 * 30000 where this reference returns 48.59 %RH, and collapses to zero above
 * adc_H = 33000. The driver had that form and reported a flat ~0.00-0.03 %RH
 * for every input, which silently disabled the humidity half of node1's leak
 * detection.
 *
 * The result is non-decreasing over the raw range: flat at 0 %RH below roughly
 * adc_H = 19000, rising through the useful span, flat at 100 %RH above roughly
 * adc_H = 41000. Non-decreasing is enough for the bisection below to find the
 * crossing, though a request outside the span lands on the nearest reachable
 * value rather than the requested one.
 */
static float humidity_pct_from_raw(int32_t adc_h, int32_t t_fine) {
    double var_h;

    var_h = (double)t_fine / 5120.0;
    var_h = var_h - 76800.0;
    var_h =
        ((double)adc_h - (((double)DIG_H4 * 64.0) + (((double)DIG_H5 / 16384.0) * var_h))) * ((double)DIG_H2 / 65536.0);
    var_h = var_h * (1.0 + (((double)DIG_H3 / 67108864.0) * var_h));
    var_h = var_h * (1.0 - (((double)DIG_H1 * var_h) / 524288.0));

    if (var_h > 100.0) {
        var_h = 100.0;
    }
    if (var_h < 0.0) {
        var_h = 0.0;
    }

    return (float)var_h;
}

/*
 * Store one coefficient as the two little-endian bytes the part keeps it in.
 *
 * Takes uint16_t rather than int16_t because the block mixes signed and
 * unsigned trimming: dig_P1 is 36477, which does not fit in an int16_t, and
 * dig_P2 is negative. The bytes are identical either way, so the width only
 * matters for not overflowing on the way in.
 */
static void put_u16_le(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
}

void mock_bme280_present(uint8_t addr) {
    uint8_t calib1[26] = {0};
    uint8_t calib2[7] = {0};

    mock_bsp_i2c_set_reg(addr, BME280_CHIP_ID_REG, BME280_CHIP_ID);

    put_u16_le(&calib1[0], (uint16_t)DIG_T1);
    put_u16_le(&calib1[2], (uint16_t)DIG_T2);
    put_u16_le(&calib1[4], (uint16_t)DIG_T3);

    put_u16_le(&calib1[6], (uint16_t)DIG_P1);
    put_u16_le(&calib1[8], (uint16_t)DIG_P2);
    put_u16_le(&calib1[10], (uint16_t)DIG_P3);
    put_u16_le(&calib1[12], (uint16_t)DIG_P4);
    put_u16_le(&calib1[14], (uint16_t)DIG_P5);
    put_u16_le(&calib1[16], (uint16_t)DIG_P6);
    put_u16_le(&calib1[18], (uint16_t)DIG_P7);
    put_u16_le(&calib1[20], (uint16_t)DIG_P8);
    put_u16_le(&calib1[22], (uint16_t)DIG_P9);

    /* dig_H1 lives alone at 0xA1, which is index 25 of the block at 0x88. */
    calib1[25] = (uint8_t)DIG_H1;
    mock_bsp_i2c_set_regs(addr, BME280_CALIB00_REG, calib1, sizeof(calib1));

    put_u16_le(&calib2[0], (uint16_t)DIG_H2);
    calib2[2] = (uint8_t)DIG_H3;
    /* dig_H4 and dig_H5 share register 0xE5: low nibble and high nibble. */
    calib2[3] = (uint8_t)(((uint16_t)DIG_H4 >> 4) & 0xFFu);
    calib2[4] = (uint8_t)(((uint16_t)DIG_H4 & 0x0Fu) | (((uint16_t)DIG_H5 & 0x0Fu) << 4));
    calib2[5] = (uint8_t)(((uint16_t)DIG_H5 >> 4) & 0xFFu);
    calib2[6] = (uint8_t)DIG_H6;
    mock_bsp_i2c_set_regs(addr, BME280_CALIB26_REG, calib2, sizeof(calib2));
}

static float measure_temperature(int32_t adc_t, int32_t unused_t_fine) {
    (void)unused_t_fine;
    return temperature_c_from_t_fine(compensate_t_fine(adc_t));
}

static float measure_pressure(int32_t adc_p, int32_t t_fine) {
    return pressure_hpa_from_raw(adc_p, t_fine);
}

static float measure_humidity(int32_t adc_h, int32_t t_fine) {
    return humidity_pct_from_raw(adc_h, t_fine);
}

/*
 * Bisection for the raw word whose compensated value is @p target.
 *
 * @p increasing selects the direction: temperature and humidity rise with the
 * raw word, pressure falls with it. Getting this backwards does not fail
 * loudly -- it just converges on an end of the range and the model reports a
 * flat 0 or a saturated value -- so the direction is a parameter rather than an
 * assumption baked into one helper.
 *
 * 40 iterations over a 20-bit range is far finer than any caller's threshold
 * and costs microseconds, running only from test setup.
 */
static int32_t solve_raw_word(int32_t lo, int32_t hi, float target, float (*measure)(int32_t, int32_t), int32_t t_fine,
                              bool increasing) {
    for (int i = 0; i < 40; i++) {
        int32_t mid = lo + ((hi - lo) / 2);
        float got;

        if (mid <= lo || mid >= hi) {
            break;
        }

        got = measure(mid, t_fine);

        if (increasing) {
            if (got < target) {
                lo = mid;
            } else {
                hi = mid;
            }
        } else {
            if (got > target) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }

    return lo + ((hi - lo) / 2);
}

void mock_bme280_set_reading(uint8_t addr, float pressure_hpa, float humidity_pct, float temp_c) {
    int32_t adc_t;
    int32_t adc_p;
    int32_t adc_h;
    int32_t t_fine;
    uint8_t data[8];

    /*
     * Temperature first: it produces t_fine, and both pressure and humidity
     * compensation are conditioned on it, so neither can be solved for until
     * adc_t is fixed.
     */
    adc_t = solve_raw_word(0, 0xFFFFF, temp_c, measure_temperature, 0, true);
    t_fine = compensate_t_fine(adc_t);

    /* Pressure decreases with the raw word; humidity is non-decreasing. */
    adc_p = solve_raw_word(0, 0xFFFFF, pressure_hpa, measure_pressure, t_fine, false);
    adc_h = solve_raw_word(0, 0xFFFF, humidity_pct, measure_humidity, t_fine, true);

    /* Pressure and temperature are 20-bit; humidity is 16-bit. */
    data[0] = (uint8_t)((adc_p >> 12) & 0xFF);
    data[1] = (uint8_t)((adc_p >> 4) & 0xFF);
    data[2] = (uint8_t)((adc_p & 0x0F) << 4);
    data[3] = (uint8_t)((adc_t >> 12) & 0xFF);
    data[4] = (uint8_t)((adc_t >> 4) & 0xFF);
    data[5] = (uint8_t)((adc_t & 0x0F) << 4);
    data[6] = (uint8_t)((adc_h >> 8) & 0xFF);
    data[7] = (uint8_t)(adc_h & 0xFF);

    mock_bsp_i2c_set_regs(addr, BME280_DATA_REG, data, sizeof(data));
}

bool mock_bme280_get_modelled_reading(uint8_t addr, float *pressure_hpa, float *humidity_pct, float *temp_c) {
    uint8_t data[8];
    int32_t adc_p;
    int32_t adc_t;
    int32_t adc_h;
    int32_t t_fine;

    if (mock_bsp_i2c_get_reg(addr, BME280_DATA_REG) == 0x00u &&
        mock_bsp_i2c_get_reg(addr, (uint8_t)(BME280_DATA_REG + 1)) == 0x00u) {
        return false;
    }

    for (uint8_t i = 0; i < sizeof(data); i++) {
        data[i] = mock_bsp_i2c_get_reg(addr, (uint8_t)(BME280_DATA_REG + i));
    }

    adc_p = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    adc_t = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    adc_h = ((int32_t)data[6] << 8) | (int32_t)data[7];

    if (adc_t == (int32_t)BME280_RAW_SENTINEL || adc_p == (int32_t)BME280_RAW_SENTINEL ||
        adc_h == (int32_t)BME280_RAW_SENTINEL) {
        return false;
    }

    t_fine = compensate_t_fine(adc_t);

    if (temp_c != NULL) {
        *temp_c = temperature_c_from_t_fine(t_fine);
    }
    if (pressure_hpa != NULL) {
        *pressure_hpa = pressure_hpa_from_raw(adc_p, t_fine);
    }
    if (humidity_pct != NULL) {
        *humidity_pct = humidity_pct_from_raw(adc_h, t_fine);
    }

    return true;
}
