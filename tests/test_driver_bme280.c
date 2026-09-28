/**
 * @file test_driver_bme280.c
 * @brief Unit tests for Bosch BME280 Driver.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include "mocks/mock_bme280.h"
#include "mocks/mock_bsp.h"
#include "rov_safety.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/*
 * BME280 registers used by the test.
 *
 * These are repeated here because the register definitions inside
 * bme280.c are private to the driver.
 */
#define BME280_TEST_ADDR 0x76u

#define BME280_REG_CALIB00   0x88u
#define BME280_REG_CHIP_ID   0xD0u
#define BME280_REG_CALIB26   0xE1u
#define BME280_REG_CTRL_HUM  0xF2u
#define BME280_REG_CTRL_MEAS 0xF4u
#define BME280_REG_CONFIG    0xF5u
#define BME280_REG_DATA      0xF7u

#define BME280_CHIP_ID 0x60u

/*
 * Store a 16-bit unsigned value in little-endian format.
 */
static void write_u16_le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

/*
 * Store a 16-bit signed value in little-endian format.
 */
static void write_s16_le(uint8_t *data, int16_t value) {
    write_u16_le(data, (uint16_t)value);
}

/*
 * Set up a fake BME280 using Bosch's example temperature and
 * pressure calibration values.
 */
static void setup_bme280(void) {
    uint8_t calib1[26] = {0};
    uint8_t calib2[7] = {0};

    mock_bsp_reset();

    /*
     * The driver checks register 0xD0 during initialization.
     * A real BME280 returns 0x60.
     */
    mock_bsp_i2c_set_reg(BME280_TEST_ADDR, BME280_REG_CHIP_ID, BME280_CHIP_ID);

    /*
     * Temperature calibration coefficients.
     */
    write_u16_le(&calib1[0], 27504);
    write_s16_le(&calib1[2], 26435);
    write_s16_le(&calib1[4], -1000);

    /*
     * Pressure calibration coefficients.
     */
    write_u16_le(&calib1[6], 36477);
    write_s16_le(&calib1[8], -10685);
    write_s16_le(&calib1[10], 3024);
    write_s16_le(&calib1[12], 2855);
    write_s16_le(&calib1[14], 140);
    write_s16_le(&calib1[16], -7);
    write_s16_le(&calib1[18], 15500);
    write_s16_le(&calib1[20], -14600);
    write_s16_le(&calib1[22], 6000);

    /*
     * H1 lives at register 0xA1, which is byte 25 of the
     * calibration block beginning at 0x88.
     *
     * We only need valid nonzero humidity coefficients here.
     * Humidity can receive its own known-vector test separately.
     */
    calib1[25] = 75;

    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_CALIB00, calib1, sizeof(calib1));

    /*
     * Humidity calibration coefficients.
     *
     * These provide usable values so the complete read path can
     * execute. A dedicated humidity reference-vector test should
     * be added once we have a verified Bosch/reference vector.
     */
    write_s16_le(&calib2[0], 362);
    calib2[2] = 0;

    /*
     * H4 = 334
     * H5 = 50
     *
     * H4 and H5 share register E5, so they have to be packed
     * according to the BME280 register format.
     */
    {
        int16_t h4 = 334;
        int16_t h5 = 50;

        calib2[3] = (uint8_t)((h4 >> 4) & 0xFF);
        calib2[4] = (uint8_t)((h4 & 0x0F) | ((h5 & 0x0F) << 4));
        calib2[5] = (uint8_t)((h5 >> 4) & 0xFF);
    }

    calib2[6] = 30;

    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_CALIB26, calib2, sizeof(calib2));
}

/*
 * Load Bosch's example raw temperature and pressure ADC values.
 *
 * adc_T = 519888
 * adc_P = 415148
 *
 * Humidity is given an arbitrary raw value here so the complete
 * measurement path executes.
 */
static void setup_measurement(void) {
    uint8_t data[8] = {0};

    int32_t adc_p = 415148;
    int32_t adc_t = 519888;
    int32_t adc_h = 30000;

    /*
     * Pressure is a 20-bit value occupying F7, F8 and the
     * upper four bits of F9.
     */
    data[0] = (uint8_t)((adc_p >> 12) & 0xFF);
    data[1] = (uint8_t)((adc_p >> 4) & 0xFF);
    data[2] = (uint8_t)((adc_p & 0x0F) << 4);

    /*
     * Temperature is another 20-bit value occupying
     * FA, FB and the upper four bits of FC.
     */
    data[3] = (uint8_t)((adc_t >> 12) & 0xFF);
    data[4] = (uint8_t)((adc_t >> 4) & 0xFF);
    data[5] = (uint8_t)((adc_t & 0x0F) << 4);

    /*
     * Humidity is a 16-bit value occupying FD and FE.
     */
    data[6] = (uint8_t)((adc_h >> 8) & 0xFF);
    data[7] = (uint8_t)(adc_h & 0xFF);

    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_DATA, data, sizeof(data));
}

/*
 * Verify NULL pointer protection.
 */
static void test_null_pointer_guards(void) {
    assert(bme280_init(NULL) == ROV_ERR_INVALID_ARG);
    assert(bme280_read_all(NULL) == ROV_ERR_INVALID_ARG);
}

/*
 * Verify that reading before initialization fails.
 */
static void test_read_before_init(void) {
    bme280_dev_t dev = {0};

    assert(bme280_read_all(&dev) != ROV_OK);
}

/*
 * Verify successful initialization of a BME280 at address 0x76.
 */
static void test_init(void) {
    bme280_dev_t dev;

    setup_bme280();

    assert(bme280_init(&dev) == ROV_OK);

    assert(dev.initialized == true);
    assert(dev.i2c_addr == BME280_TEST_ADDR);

    /*
     * Make sure calibration values were actually decoded.
     */
    assert(dev.calib.dig_T1 == 27504);
    assert(dev.calib.dig_T2 == 26435);
    assert(dev.calib.dig_T3 == -1000);

    assert(dev.calib.dig_P1 == 36477);
    assert(dev.calib.dig_P2 == -10685);
}

/*
 * Verify the configuration required by the BME280 issue.
 */
static void test_configuration(void) {
    bme280_dev_t dev;

    setup_bme280();

    assert(bme280_init(&dev) == ROV_OK);

    /*
     * ctrl_hum:
     * humidity oversampling x1
     */
    assert(mock_bsp_i2c_get_reg(BME280_TEST_ADDR, BME280_REG_CTRL_HUM) == 0x01u);

    /*
     * config:
     * standby 0.5 ms
     * IIR filter coefficient 16
     */
    assert(mock_bsp_i2c_get_reg(BME280_TEST_ADDR, BME280_REG_CONFIG) == 0x10u);

    /*
     * ctrl_meas:
     * temperature x2
     * pressure x16
     * normal mode
     */
    assert(mock_bsp_i2c_get_reg(BME280_TEST_ADDR, BME280_REG_CTRL_MEAS) == 0x57u);
}

/*
 * Verify that an invalid chip ID causes initialization to fail.
 */
static void test_wrong_chip_id(void) {
    bme280_dev_t dev;

    mock_bsp_reset();

    /*
     * 0x42 is not the BME280 chip ID.
     */
    mock_bsp_i2c_set_reg(BME280_I2C_ADDR_PRIMARY, BME280_REG_CHIP_ID, 0x42u);

    mock_bsp_i2c_set_reg(BME280_I2C_ADDR_SECONDARY, BME280_REG_CHIP_ID, 0x42u);

    assert(bme280_init(&dev) != ROV_OK);
}

/*
 * Verify that invalid calibration data is rejected.
 */
static void test_invalid_calibration(void) {
    bme280_dev_t dev;
    uint8_t zero_calibration[26] = {0};

    setup_bme280();

    /*
     * Replace the valid calibration block with zeros.
     *
     * This makes dig_T1 and dig_P1 zero.
     */
    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_CALIB00, zero_calibration, sizeof(zero_calibration));

    assert(bme280_init(&dev) != ROV_OK);
}

/*
 * Verify the known Bosch temperature and pressure example.
 */
static void test_compensation(void) {
    bme280_dev_t dev;

    setup_bme280();
    setup_measurement();

    assert(bme280_init(&dev) == ROV_OK);
    assert(bme280_read_all(&dev) == ROV_OK);

    /*
     * Bosch example:
     *
     * adc_T = 519888 -> approximately 25.08 C
     * adc_P = 415148 -> approximately 1006.53 hPa
     *
     * Use ranges instead of exact floating-point equality.
     */
    assert(dev.temperature_c > 25.07f && dev.temperature_c < 25.09f);

    assert(dev.pressure_hpa > 1006.4f && dev.pressure_hpa < 1006.7f);
}

/*
 * Verify humidity against a published vector rather than against its own clamp.
 *
 * With the datasheet's example trimming at 25.08 C, the datasheet's reference
 * algorithm returns 48.59 %RH for adc_H = 30000.
 *
 * This assertion is the reason the earlier revision of this driver was caught.
 * It used the widely copied integer humidity transcription, which returns
 * 0.37 %RH for the same input, and the previous test only asserted
 * 0 <= humidity <= 100 -- bounds that the function's own clamp guarantees, so
 * a formula that returned a flat ~0.00-0.03 %RH for every possible input
 * passed. That silently disabled node1's humidity leak threshold, which sits
 * at ROV_LEAK_HUMIDITY_MAX_PCT, because 0 %RH can never cross it.
 */
static void test_humidity_reference_vector(void) {
    bme280_dev_t dev;
    uint8_t data[8] = {0};

    setup_bme280();

    data[3] = (uint8_t)((519888 >> 12) & 0xFF);
    data[4] = (uint8_t)((519888 >> 4) & 0xFF);
    data[5] = (uint8_t)((519888 & 0x0F) << 4);
    data[6] = (uint8_t)((30000 >> 8) & 0xFF);
    data[7] = (uint8_t)(30000 & 0xFF);
    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_DATA, data, sizeof(data));

    assert(bme280_init(&dev) == ROV_OK);
    assert(bme280_read_all(&dev) == ROV_OK);

    assert(dev.temperature_c > 25.07f && dev.temperature_c < 25.09f);
    assert(dev.humidity_pct > 48.5f && dev.humidity_pct < 48.7f);
}

/*
 * Verify humidity rises with the raw word, so the leak threshold is reachable.
 *
 * A single vector can be satisfied by a formula that is right once and wrong
 * everywhere else. Node1's leak logic compares against a percentage, so what
 * matters is that the reading is monotonic in the raw word and spans the
 * threshold; a flat or inverted response would pass a one-point check and still
 * make the threshold unreachable.
 */
static void test_humidity_is_monotonic_across_the_threshold(void) {
    bme280_dev_t dev;
    float previous = -1.0f;
    uint8_t data[8] = {0};
    int saw_below = 0;
    int saw_above = 0;

    setup_bme280();
    data[3] = (uint8_t)((519888 >> 12) & 0xFF);
    data[4] = (uint8_t)((519888 >> 4) & 0xFF);
    data[5] = (uint8_t)((519888 & 0x0F) << 4);

    assert(bme280_init(&dev) == ROV_OK);

    for (int32_t adc_h = 16000; adc_h <= 48000; adc_h += 1000) {
        float got;

        data[6] = (uint8_t)((adc_h >> 8) & 0xFF);
        data[7] = (uint8_t)(adc_h & 0xFF);
        mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_DATA, data, sizeof(data));

        assert(bme280_read_all(&dev) == ROV_OK);
        got = dev.humidity_pct;

        assert(got >= previous);
        previous = got;

        if (got < ROV_LEAK_HUMIDITY_MAX_PCT) {
            saw_below = 1;
        }
        if (got > ROV_LEAK_HUMIDITY_MAX_PCT) {
            saw_above = 1;
        }
    }

    assert(saw_below);
    assert(saw_above);
}

/*
 * Verify a read that hits the power-on sentinel is reported as a failure.
 *
 * The first poll after bme280_init() lands here in practice, because
 * configuring oversampling starts a conversion that has not finished. Without
 * the check the driver compensates 0x80000 into a plausible-looking reading and
 * node1 publishes it, and a leak threshold then gets evaluated against a sample
 * that was never taken.
 */
static void test_power_on_sentinel_is_rejected(void) {
    bme280_dev_t dev;
    uint8_t data[8];
    int32_t sentinel = 0x80000;

    setup_bme280();

    data[0] = (uint8_t)((sentinel >> 12) & 0xFF);
    data[1] = (uint8_t)((sentinel >> 4) & 0xFF);
    data[2] = (uint8_t)((sentinel & 0x0F) << 4);
    data[3] = data[0];
    data[4] = data[1];
    data[5] = data[2];
    data[6] = (uint8_t)((sentinel >> 8) & 0xFF);
    data[7] = (uint8_t)(sentinel & 0xFF);
    mock_bsp_i2c_set_regs(BME280_TEST_ADDR, BME280_REG_DATA, data, sizeof(data));

    assert(bme280_init(&dev) == ROV_OK);
    assert(bme280_read_all(&dev) != ROV_OK);
}

/*
 * Verify the device model can actually reach the readings it is asked for.
 *
 * tests/test_node1_pi_shield.c sets the BME280 by engineering value and then
 * asserts on leak thresholds, so if the model silently failed to invert -- and
 * left every reading at a flat 0 or a saturated 100 -- those tests would keep
 * passing while testing nothing. This asserts the model's own forward
 * computation lands on the request, so a broken inversion fails here instead.
 */
static void test_device_model_round_trip(void) {
    static const float cases[][3] = {
        {1013.25f, 42.0f, 26.5f},
        {750.0f, 30.0f, 22.0f},
        {1013.25f, 85.0f, 24.0f},
    };

    for (unsigned i = 0; i < (sizeof(cases) / sizeof(cases[0])); i++) {
        bme280_dev_t dev;
        float want_p = cases[i][0];
        float want_h = cases[i][1];
        float want_t = cases[i][2];
        float model_p = 0.0f;
        float model_h = 0.0f;
        float model_t = 0.0f;

        mock_bsp_reset();
        mock_bme280_present(BME280_TEST_ADDR);
        mock_bme280_set_reading(BME280_TEST_ADDR, want_p, want_h, want_t);

        assert(mock_bme280_get_modelled_reading(BME280_TEST_ADDR, &model_p, &model_h, &model_t));

        /* Pressure within 0.5 hPa, humidity within 0.5 %RH, temperature within 0.05 C. */
        assert(fabsf(model_p - want_p) < 0.5f);
        assert(fabsf(model_h - want_h) < 0.5f);
        assert(fabsf(model_t - want_t) < 0.05f);

        /* And the driver must agree with the model, since that is the real path. */
        assert(bme280_init(&dev) == ROV_OK);
        assert(bme280_read_all(&dev) == ROV_OK);

        assert(fabsf(dev.pressure_hpa - want_p) < 0.5f);
        assert(fabsf(dev.humidity_pct - want_h) < 0.5f);
        assert(fabsf(dev.temperature_c - want_t) < 0.05f);
    }
}

void test_bme280_driver(void) {
    test_null_pointer_guards();
    test_read_before_init();
    test_init();
    test_configuration();
    test_wrong_chip_id();
    test_invalid_calibration();
    test_compensation();
    test_humidity_reference_vector();
    test_humidity_is_monotonic_across_the_threshold();
    test_power_on_sentinel_is_rejected();
    test_device_model_round_trip();

    printf("[PASS] test_bme280_driver\n");
}

int main(void) {
    printf("Running BME280 Driver Unit Tests...\n");

    test_bme280_driver();

    printf("All BME280 Driver Tests Passed Successfully!\n");

    return 0;
}
