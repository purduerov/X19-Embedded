/**
 * @file platform_i2c.c
 * @brief NUCLEO-G474 platform I2C bus driver.
 */

#include "bsp.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void bsp_i2c_init(void) {
    /* Peripheral init is handled by CubeMX in main.c */
}

bool bsp_i2c_probe(uint8_t addr) {
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1U), 2, 5) == HAL_OK;
}

uint8_t bsp_i2c_scan(void) {
    uint8_t count = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (bsp_i2c_probe(addr)) {
            count++;
        }
    }
    return count;
}

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    if (!data || len == 0U) {
        return false;
    }
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1U), (uint8_t *)data, len, 10U) == HAL_OK;
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (!data || len == 0U) {
        return false;
    }
    return HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr << 1U), data, len, 10U) == HAL_OK;
}

rov_status_t bsp_i2c_mem_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (!data || len == 0U) {
        return ROV_ERR_INVALID_ARG;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1U), (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, data, len, 10U) !=
        HAL_OK) {
        return ROV_ERROR;
    }
    return ROV_OK;
}

rov_status_t bsp_i2c_mem_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    if (!data || len == 0U) {
        return ROV_ERR_INVALID_ARG;
    }
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1U), (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len,
                          10U) != HAL_OK) {
        return ROV_ERROR;
    }
    return ROV_OK;
}
