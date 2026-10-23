/**
 * @file bsp_stm32c5.c
 * @brief Board Support Package setup and safety coordinator for STM32C5.
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "can_interface.h"
#include "main.h"
#include <stdio.h>

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize and start CAN hardware and filters */
    can_init();
}

bool bsp_leak_probe_read(uint8_t probe_idx) {
    if (probe_idx == 0) {
        return (HAL_GPIO_ReadPin(LEAK_PROBE0_GPIO_Port, LEAK_PROBE0_Pin) == GPIO_PIN_RESET);
    } else if (probe_idx == 1) {
        return (HAL_GPIO_ReadPin(LEAK_PROBE1_GPIO_Port, LEAK_PROBE1_Pin) == GPIO_PIN_RESET);
    }
    return false;
}

void bsp_emergency_brake_trip(void) {
    HAL_GPIO_WritePin(EMERGENCY_CUTOFF_GPIO_Port, EMERGENCY_CUTOFF_Pin, GPIO_PIN_SET);
}

bool bsp_is_emergency_brake_tripped(void) {
    return (HAL_GPIO_ReadPin(EMERGENCY_CUTOFF_GPIO_Port, EMERGENCY_CUTOFF_Pin) == GPIO_PIN_SET);
}

static rov_status_t bsp_bus_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_write(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static rov_status_t bsp_bus_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    (void)ctx;
    return bsp_i2c_read(addr, data, len) ? ROV_OK : ROV_ERROR;
}

static bool bsp_bus_probe_fn(void *ctx, uint8_t addr) {
    (void)ctx;
    return bsp_i2c_probe(addr);
}

const rov_i2c_bus_t *bsp_i2c_bus(void) {
    static const rov_i2c_bus_t bus = {NULL, bsp_bus_write, bsp_bus_read, bsp_bus_probe_fn};
    return &bus;
}
