/**
 * @file mock_tmp1075.c
 * @brief TMP1075 device model for the host mock I2C register file.
 * @organization Purdue ROV
 */

#include "mock_tmp1075.h"

#include "mock_bsp.h"

/* Register map, matching drivers/src/tmp1075.c. */
#define TMP1075_REG_TEMPERATURE 0x00u

/* 12-bit two's complement, left-justified, 0.0625 C per count. */
#define TMP1075_TEMP_LSB_C 0.0625f

void mock_tmp1075_set_temperature(uint8_t addr, float temp_c) {
    int16_t counts;

    if (temp_c >= 0.0f) {
        counts = (int16_t)((temp_c / TMP1075_TEMP_LSB_C) + 0.5f);
    } else {
        counts = (int16_t)((temp_c / TMP1075_TEMP_LSB_C) - 0.5f);
    }

    /* Left-justify the 12-bit reading into the top nibble-aligned position. */
    mock_bsp_i2c_set_word(addr, TMP1075_REG_TEMPERATURE, (uint16_t)((uint16_t)counts << 4));
}
