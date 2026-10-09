/**
 * @file platform_pwm.c
 * @brief NUCLEO-G474 platform 8-channel ESC PWM driver (TIM1/TIM8).
 */

#include "bsp.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

static uint16_t g_pwm_duty_us[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

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
