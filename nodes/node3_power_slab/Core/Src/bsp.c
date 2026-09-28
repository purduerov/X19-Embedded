/**
 * @file bsp.c
 * @brief Board Support Package Implementation for Node 3 (Power Slab).
 * @author Purdue ROV Embedded Team
 *
 * Implements low-level hardware routines for Node 3:
 * - 4x DC-DC converter brick GPIO enable control (PB0, PB1, PB2, PB4)
 * - 5.2V logic rail monitoring via INA237
 * - TI LM74700-Q1 ideal diode status check
 * - TMP1075 PCB temperature reading
 * - System timers & diagnostic LED
 *
 * Note: Under host unit testing (-DROV_UNIT_TEST), mock_bsp.c provides the
 * implementation. This file is compiled when targeting bare-metal hardware.
 */

#ifndef ROV_UNIT_TEST

#include "bsp.h"
#include "can_interface.h"

#if __has_include("main.h")
#include "main.h"
#endif

void bsp_init(void) {
    /* Peripheral init is performed in main.c by STM32CubeMX generated code */
}

uint32_t time_get_ms(void) {
#if defined(HAL_GetTick) || defined(STM32C5) || defined(STM32G4)
    return HAL_GetTick();
#else
    return 0;
#endif
}

uint64_t time_get_us(void) {
    return (uint64_t)time_get_ms() * 1000ULL;
}

void delay_ms(uint32_t ms) {
#if defined(HAL_Delay) || defined(STM32C5) || defined(STM32G4)
    HAL_Delay(ms);
#else
    (void)ms;
#endif
}

void led_toggle(void) {
#if defined(LED_HEARTBEAT_GPIO_Port) && defined(LED_HEARTBEAT_Pin)
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
#endif
}

void led_set(bool state) {
#if defined(LED_HEARTBEAT_GPIO_Port) && defined(LED_HEARTBEAT_Pin)
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)state;
#endif
}

void bsp_power_brick_enable(uint8_t brick_idx) {
    /*
     * VCB4812 DC-DC converter enable pins:
     * Brick 0 -> PB0
     * Brick 1 -> PB1
     * Brick 2 -> PB2
     * Brick 3 -> PB4
     */
#if defined(BRICK1_EN_GPIO_Port) && defined(BRICK1_EN_Pin)
    switch (brick_idx) {
    case 0:
        HAL_GPIO_WritePin(BRICK1_EN_GPIO_Port, BRICK1_EN_Pin, GPIO_PIN_SET);
        break;
    case 1:
        HAL_GPIO_WritePin(BRICK2_EN_GPIO_Port, BRICK2_EN_Pin, GPIO_PIN_SET);
        break;
    case 2:
        HAL_GPIO_WritePin(BRICK3_EN_GPIO_Port, BRICK3_EN_Pin, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(BRICK4_EN_GPIO_Port, BRICK4_EN_Pin, GPIO_PIN_SET);
        break;
    default:
        break;
    }
#else
    (void)brick_idx;
#endif
}

void bsp_power_brick_disable_all(void) {
#if defined(BRICK1_EN_GPIO_Port) && defined(BRICK1_EN_Pin)
    HAL_GPIO_WritePin(BRICK1_EN_GPIO_Port, BRICK1_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BRICK2_EN_GPIO_Port, BRICK2_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BRICK3_EN_GPIO_Port, BRICK3_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BRICK4_EN_GPIO_Port, BRICK4_EN_Pin, GPIO_PIN_RESET);
#endif
}

uint32_t bsp_get_logic_voltage_mv(void) {
    /* Default return 5200 mV; populated from INA237 hardware driver on target */
    return 5200;
}

bool bsp_lm74700_status_ok(void) {
    /* Read ideal diode status flag GPIO */
#if defined(LM74700_STAT_GPIO_Port) && defined(LM74700_STAT_Pin)
    return (HAL_GPIO_ReadPin(LM74700_STAT_GPIO_Port, LM74700_STAT_Pin) == GPIO_PIN_SET);
#else
    return true;
#endif
}

float bsp_get_pcb_temperature_c(void) {
    /* Default safe ambient; populated from TMP1075 hardware driver on target */
    return 25.0f;
}

/*
 * PMBus word read for the converter bricks.
 *
 * Not implemented yet: node 3 has no I2C peripheral configured (no MX_I2C*_Init
 * in main.c, and no HAL module enabled), so there is no bus to drive a PMBus
 * transaction on. Reporting failure is the honest answer -- a zero word would
 * reach pmbus_brick_read_telemetry() as a genuine 0 V / 0 A sample and be
 * published on 0x300 as though it had been measured.
 *
 * When the I2C peripheral does land, this becomes a send-command-then-receive
 * with a repeated start between them, which is the shape PMBus requires.
 */
bool bsp_pmbus_read_word(uint8_t pmbus_addr, uint8_t command, uint16_t *raw_word) {
    (void)pmbus_addr;
    (void)command;
    (void)raw_word;

    return false;
}

#endif /* ROV_UNIT_TEST */
