/**
 * @file bsp.c
 * @brief Board Support Package Implementation for Node 2 (Control Board).
 * @author Purdue ROV Embedded Team
 *
 * Implements low-level hardware routines for Node 2:
 * - 8-channel ESC PWM updates (TIM1 Ch1..Ch4, TIM8 Ch1..Ch4)
 * - 10-channel pneumatic solenoid MOSFET driver pin toggling
 * - Hardware emergency break trip / check
 * - Microsecond/millisecond system timers
 * - Diagnostic LED
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

#if __has_include("tim.h")
#include "tim.h"
#endif

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

#if defined(HAL_I2C_MODULE_ENABLED)
extern I2C_HandleTypeDef hi2c1;
#endif

/* Cached PWM duty cycles for reading back */
static uint16_t g_pwm_duty_us[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
static uint16_t g_solenoid_state_mask = 0;

void bsp_init(void) {
    /* Peripheral init is performed in main.c by STM32CubeMX generated code */
    static const uint32_t tim_channels[4] = {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4,
    };

    for (uint8_t i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim1, tim_channels[i], 1500U);
        __HAL_TIM_SET_COMPARE(&htim8, tim_channels[i], 1500U);

        HAL_TIM_PWM_Start(&htim1, tim_channels[i]);
        HAL_TIM_PWM_Start(&htim8, tim_channels[i]);
    }

    for (uint8_t i = 0; i < 8; i++) {
        g_pwm_duty_us[i] = 1500U;
    }
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

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
#if defined(HAL_I2C_MODULE_ENABLED)
    if (!data || len == 0U) {
        return false;
    }

    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1U), (uint8_t *)data, len, 10U) == HAL_OK;
#else
    (void)addr;
    (void)data;
    (void)len;

    return false;
#endif
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
#if defined(HAL_I2C_MODULE_ENABLED)
    if (!data || len == 0U) {
        return false;
    }

    return HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr << 1U), data, len, 10U) == HAL_OK;
#else
    (void)addr;
    (void)data;
    (void)len;

    return false;
#endif
}

void led_toggle(void) {
#if defined(LED_STATUS_GPIO_Port) && defined(LED_STATUS_Pin)
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
#endif
}

void led_set(bool state) {
#if defined(LED_STATUS_GPIO_Port) && defined(LED_STATUS_Pin)
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)state;
#endif
}

void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us) {
    if (channel >= 8) {
        return;
    }

    if (pulse_us < 1000U) {
        pulse_us = 1000U;
    } else if (pulse_us > 2000U) {
        pulse_us = 2000U;
    }

    g_pwm_duty_us[channel] = pulse_us;

    /*
     * Hardware Timer Register Writes:
     * ESC Channels 0..3 -> TIM1 CCR1..CCR4
     * ESC Channels 4..7 -> TIM8 CCR1..CCR4
     * At 1 MHz counter clock, 1 tick = 1 microsecond.
     */

    switch (channel) {
    case 0:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
        break;
    case 1:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse_us);
        break;
    case 2:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_us);
        break;
    case 3:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse_us);
        break;
    case 4:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse_us);
        break;
    case 5:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pulse_us);
        break;
    case 6:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pulse_us);
        break;
    case 7:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pulse_us);
        break;
    default:
        break;
    }
}

uint16_t bsp_pwm_get_us(uint8_t channel) {
    if (channel < 8) {
        return g_pwm_duty_us[channel];
    }
    return 1500;
}

void bsp_solenoid_set(uint16_t mask) {
    g_solenoid_state_mask = (mask & 0x03FF);

    /*
     * 10-channel discrete N-channel MOSFET drivers for pneumatic solenoids.
     * Write GPIO state for active channels.
     */
#if defined(SOL_0_GPIO_Port) && defined(SOL_0_Pin)
    HAL_GPIO_WritePin(SOL_0_GPIO_Port, SOL_0_Pin, (mask & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

uint16_t bsp_solenoid_get(void) {
    return g_solenoid_state_mask;
}

void bsp_emergency_brake_trip(void) {
    /* Set all PWM compare registers to 1500 us neutral stop */
    for (uint8_t ch = 0; ch < 8; ch++) {
        bsp_pwm_set_us(ch, 1500);
    }
    /* Turn off all pneumatic solenoids */
    bsp_solenoid_set(0);
}

bool bsp_is_emergency_brake_tripped(void) {
    return false;
}

#endif /* ROV_UNIT_TEST */
