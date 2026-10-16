/**
 * @file mock_tmp1075.h
 * @brief TMP1075 device model for the host mock I2C register file.
 * @organization Purdue ROV
 *
 * Places a TMP1075 at a chosen temperature by writing the raw register word a
 * real part would have produced. The conversion is a single linear scale, so
 * the inverse is exact arithmetic rather than the BME280 model's bisection.
 */

#ifndef MOCK_TMP1075_H
#define MOCK_TMP1075_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write the temperature register of the TMP1075 at @p addr.
 * @param addr 7-bit I2C address, 0x48..0x4F.
 * @param temp_c Target temperature in degrees Celsius.
 */
void mock_tmp1075_set_temperature(uint8_t addr, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_TMP1075_H */
