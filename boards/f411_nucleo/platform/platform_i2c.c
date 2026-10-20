/**
 * @file platform_i2c.c
 * @brief NUCLEO-F411 platform I2C driver (PB8:SCL, PB9:SDA).
 */

#include "bsp.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

void bsp_i2c_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Idle bus: SCL and SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    /* If SDA is held low by a hung slave, clock SCL 9 times to free it */
    for (volatile int i = 0; i < 300; i++) {
        __NOP();
    }
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
        for (int i = 0; i < 9; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            for (volatile int d = 0; d < 300; d++) {
                __NOP();
            }
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            for (volatile int d = 0; d < 300; d++) {
                __NOP();
            }
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) {
                break;
            }
        }
        /* STOP condition */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 300; d++) {
            __NOP();
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 300; d++) {
            __NOP();
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    }
}

static inline void i2c_delay(void) {
    /* ~5 us delay per half-clock @ 180 MHz / 100 MHz for standard ~100 kHz I2C */
    for (volatile int i = 0; i < 300; i++) {
        __NOP();
    }
}

static inline void i2c_scl(bool high) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

static inline void i2c_sda(bool high) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

static inline bool i2c_read_scl(void) {
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET);
}

static inline bool i2c_read_sda(void) {
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET);
}

bool bsp_i2c_probe(uint8_t addr) {
    /* Bus idle verification: both lines MUST be HIGH before generating START.
     * If SDA or SCL is already LOW, either:
     * 1) No pull-up resistor is present (lines floating low).
     * 2) Wire is shorted to GND.
     * 3) A slave device is hung holding the bus low.
     * In all cases, an active-low ACK cannot be reliably detected. */
    i2c_sda(true);
    i2c_scl(true);
    if (!i2c_read_sda() || !i2c_read_scl()) {
        return false;
    }

    /* START condition */
    i2c_sda(false);
    i2c_scl(false);

    /* Send 7-bit address + Write bit (0) */
    uint8_t byte = (uint8_t)(addr << 1);
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }

    /* Release SDA and read ACK (LOW = ACK) */
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);

    /* STOP condition */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);

    return ack;
}

uint8_t bsp_i2c_scan(void) {
    uint8_t count = 0;
    printf("\r\n========================================\r\n");
    printf("   I2C Bus Scanner (PB8:SCL, PB9:SDA)   \r\n");
    printf("========================================\r\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (uint8_t row = 0; row < 128; row += 16) {
        printf("%02X: ", row);
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t addr = (uint8_t)(row + col);
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
            } else if (bsp_i2c_probe(addr)) {
                printf("%02X ", addr);
                count++;
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    printf("----------------------------------------\r\n");
    printf("Scan complete: found %u device(s).\r\n", count);

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (bsp_i2c_probe(addr)) {
            const char *desc = "Unknown device";
            if (addr == 0x76 || addr == 0x77) {
                desc = "Bosch BME280 (Pressure / Humidity / Temp)";
            } else if (addr == 0x7F) {
                desc = "Delta E48SC12030 PMBus Brick (Default Fallback Address 127)";
            } else if (addr >= 0x40 && addr <= 0x47) {
                desc = "TI INA237 / INA226 (Current / Voltage / Power Monitor)";
            } else if (addr >= 0x48 && addr <= 0x4F) {
                desc = "TI TMP1075 / BNO086 (Temperature / IMU Sensor)";
            }
            printf("  -> [0x%02X] %s\r\n", addr, desc);
        }
    }
    printf("========================================\r\n\r\n");
    return count;
}

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    if (!data && len > 0)
        return false;

    /* START */
    i2c_sda(true);
    i2c_scl(true);
    i2c_sda(false);
    i2c_scl(false);

    /* Address + Write */
    uint8_t byte = (uint8_t)(addr << 1);
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);
    if (!ack) {
        i2c_sda(false);
        i2c_scl(true);
        i2c_sda(true);
        return false;
    }

    for (uint16_t b = 0; b < len; b++) {
        byte = data[b];
        for (int i = 7; i >= 0; i--) {
            i2c_sda((byte >> i) & 1);
            i2c_scl(true);
            i2c_scl(false);
        }
        i2c_sda(true);
        i2c_scl(true);
        ack = !i2c_read_sda();
        i2c_scl(false);
        if (!ack) {
            i2c_sda(false);
            i2c_scl(true);
            i2c_sda(true);
            return false;
        }
    }

    /* STOP */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);
    return true;
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (!data && len > 0)
        return false;

    /* START */
    i2c_sda(true);
    i2c_scl(true);
    i2c_sda(false);
    i2c_scl(false);

    /* Address + Read */
    uint8_t byte = (uint8_t)((addr << 1) | 0x01);
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);
    if (!ack) {
        i2c_sda(false);
        i2c_scl(true);
        i2c_sda(true);
        return false;
    }

    for (uint16_t b = 0; b < len; b++) {
        uint8_t received = 0;
        i2c_sda(true);
        for (int i = 7; i >= 0; i--) {
            i2c_scl(true);
            if (i2c_read_sda()) {
                received |= (uint8_t)(1 << i);
            }
            i2c_scl(false);
        }
        data[b] = received;

        if (b + 1 < len) {
            i2c_sda(false); /* ACK */
        } else {
            i2c_sda(true); /* NACK on last */
        }
        i2c_scl(true);
        i2c_scl(false);
    }

    /* STOP */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);
    return true;
}

rov_status_t bsp_i2c_mem_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U) {
        return ROV_ERR_INVALID_ARG;
    }

    if (!bsp_i2c_write(addr, &reg, 1U)) {
        return ROV_ERROR;
    }

    if (!bsp_i2c_read(addr, data, len)) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

rov_status_t bsp_i2c_mem_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U || len > 32U) {
        return ROV_ERR_INVALID_ARG;
    }

    uint8_t tx_data[33];
    tx_data[0] = reg;
    memcpy(&tx_data[1], data, len);

    if (!bsp_i2c_write(addr, tx_data, (uint16_t)(len + 1U))) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

/*
 * Expose the raw I2C trio as an injectable bus. Drivers take a
 * `rov_i2c_bus_t`, so this adapter is what lets the same driver source build
 * for this target and for the host mock.
 */
static rov_status_t bsp_bus_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_write(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static rov_status_t bsp_bus_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_read(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static bool bsp_bus_probe(void *ctx, uint8_t addr) {
    (void)ctx;
    return bsp_i2c_probe(addr);
}

const rov_i2c_bus_t *bsp_i2c_bus(void) {
    static const rov_i2c_bus_t bus = {NULL, bsp_bus_write, bsp_bus_read, bsp_bus_probe};

    return &bus;
}
