/**
 * @file mock_bme280.h
 * @brief BME280 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Lets a test place the sensor at a chosen engineering reading -- "1013.25 hPa,
 * 35 %RH, 24 C" -- instead of choosing raw ADC counts, which is what a caller
 * that cares about leak thresholds actually wants to express.
 *
 * @note This is a device model, not a second copy of the driver. It answers
 *       "what raw words would a real part produce at this temperature?", and it
 *       derives them by running Bosch's published compensation backwards. The
 *       driver's own arithmetic is verified separately, against the datasheet's
 *       worked example, in tests/test_driver_bme280.c. Keeping the two
 *       assertions separate is the point: if this model and the driver ever
 *       drifted together, the leak-threshold tests in
 *       tests/test_node1_pi_shield.c would still be testing the leak logic
 *       rather than silently testing a shared bug.
 */

#ifndef MOCK_BME280_H
#define MOCK_BME280_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Put a BME280 on the mock bus and load the datasheet's example calibration.
 *
 * Writes the chip id the driver probes for, plus Bosch's reference trimming
 * values, so bme280_init() succeeds at @p addr. Without this the driver sees an
 * empty register file and init fails, which is a different test.
 *
 * @param addr 7-bit I2C address, 0x76 or 0x77.
 */
void mock_bme280_present(uint8_t addr);

/**
 * @brief Set the raw measurement registers so the driver reports @p pressure_hpa,
 *        @p humidity_pct and @p temp_c.
 *
 * Solves for the ADC words by bisection on Bosch's compensation, so the values
 * the driver reports land on the requested engineering values rather than near
 * them. Bisection needs the forward direction to be monotonic over the searched
 * range, which it is for all three quantities across the sensor's operating
 * span; out-of-span requests clamp to the nearest reachable value.
 *
 * @param addr 7-bit I2C address previously passed to mock_bme280_present().
 * @param pressure_hpa Target pressure in hPa.
 * @param humidity_pct Target relative humidity in percent.
 * @param temp_c Target temperature in degrees Celsius.
 */
void mock_bme280_set_reading(uint8_t addr, float pressure_hpa, float humidity_pct, float temp_c);

/**
 * @brief Report the engineering values @p addr is currently modelled at.
 *
 * The forward direction of the model: the raw words in the register file are
 * compensated exactly as the datasheet specifies. Lets a test assert that the
 * model itself can actually reach the value it was asked for, instead of
 * trusting that it did.
 *
 * @param addr 7-bit I2C address.
 * @param pressure_hpa Out: modelled pressure in hPa.
 * @param humidity_pct Out: modelled relative humidity in percent.
 * @param temp_c Out: modelled temperature in degrees Celsius.
 * @return true if the model holds a valid measurement, false if the data
 *         registers are empty or hold the power-on sentinel.
 */
bool mock_bme280_get_modelled_reading(uint8_t addr, float *pressure_hpa, float *humidity_pct, float *temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_BME280_H */
