/**
 * @file rov_i2c.h
 * @brief Injected I2C bus contract shared by drivers, target BSP, and host mocks.
 * @organization Purdue ROV
 *
 * A driver never talks to the board BSP directly. It holds a `rov_i2c_bus_t`
 * and issues raw byte transfers, so the same driver source runs on STM32 target
 * firmware (over HAL) and on the host (over a register-file mock).
 *
 * `mem_*` is a convenience over the raw pair: write the register pointer, then
 * read or write the payload. Command-addressed parts (e.g. MS5837) use the raw
 * pair directly, because a register pointer means nothing to them.
 */

#ifndef ROV_I2C_H
#define ROV_I2C_H

#include "rov_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rov_i2c_bus rov_i2c_bus_t;

struct rov_i2c_bus {
    void *ctx;
    rov_status_t (*write)(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len);
    rov_status_t (*read)(void *ctx, uint8_t addr, uint8_t *data, uint16_t len);
    bool (*probe)(void *ctx, uint8_t addr);
};

/**
 * @brief Write a register address, then read len bytes from the device.
 * @return ROV_OK on success, the transport error otherwise, or
 *         ROV_ERR_INVALID_ARG for a null bus/destination.
 */
static inline rov_status_t rov_i2c_mem_read(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg, uint8_t *dst,
                                            uint16_t len) {
    rov_status_t status;

    if (bus == NULL || bus->write == NULL || bus->read == NULL || dst == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    status = bus->write(bus->ctx, addr, &reg, 1U);
    if (status != ROV_OK) {
        return status;
    }

    return bus->read(bus->ctx, addr, dst, len);
}

/**
 * @brief Write a register address followed by len data bytes.
 * @return ROV_OK on success, the transport error otherwise, or
 *         ROV_ERR_INVALID_ARG for a null bus/source or a payload over 8 bytes.
 */
static inline rov_status_t rov_i2c_mem_write(const rov_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                                             const uint8_t *src, uint16_t len) {
    uint8_t buffer[1U + 8U];

    if (bus == NULL || bus->write == NULL || src == NULL || len == 0U || len > 8U) {
        return ROV_ERR_INVALID_ARG;
    }

    buffer[0] = reg;

    for (uint16_t i = 0U; i < len; i++) {
        buffer[1U + i] = src[i];
    }

    return bus->write(bus->ctx, addr, buffer, (uint16_t)(len + 1U));
}

#ifdef __cplusplus
}
#endif

#endif /* ROV_I2C_H */
