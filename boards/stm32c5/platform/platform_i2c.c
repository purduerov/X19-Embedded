/**
 * @file platform_i2c.c
 * @brief STM32C5 platform I2C driver.
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

bool bsp_i2c_probe(uint8_t addr) {
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2, 5) == HAL_OK);
}

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    return (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1), (uint8_t *)data, len, 50) == HAL_OK);
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    return (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr << 1), data, len, 50) == HAL_OK);
}
