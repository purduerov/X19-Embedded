/**
 * @file tmp1075.c
 * @brief TI TMP1075 Temperature Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "tmp1075.h"

#include <string.h>

/* TMP1075 register pointer values. */
#define TMP1075_REG_TEMPERATURE 0x00u

/* 12-bit two's complement, left-justified in 16 bits, 0.0625 C per count. */
#define TMP1075_TEMP_LSB_C 0.0625f

rov_status_t tmp1075_init(tmp1075_dev_t *dev, const rov_i2c_bus_t *bus, uint8_t i2c_addr) {
    if (dev == NULL || bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(tmp1075_dev_t));
    dev->bus = bus;
    dev->i2c_addr = i2c_addr;

    return ROV_OK;
}

rov_status_t tmp1075_read_temperature(tmp1075_dev_t *dev) {
    uint8_t raw[2] = {0u, 0u};
    int16_t raw16;
    int16_t t12;

    if (dev == NULL || dev->bus == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    rov_status_t status = rov_i2c_mem_read(dev->bus, dev->i2c_addr, TMP1075_REG_TEMPERATURE, raw, sizeof(raw));

    if (status != ROV_OK) {
        return status;
    }

    /*
     * The temperature register is the 12-bit two's complement reading in bits
     * 15:4, with bits 3:0 reading zero. Sign-extend the 16-bit word first, then
     * drop the four pad bits, so a negative reading stays negative.
     */
    raw16 = (int16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    t12 = (int16_t)(raw16 >> 4);
    dev->temperature_c = (float)t12 * TMP1075_TEMP_LSB_C;

    return ROV_OK;
}
