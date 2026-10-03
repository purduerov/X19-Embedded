/**
 * @file bsp_g474.c
 * @brief Board Support Package setup and safety coordinator for NUCLEO-G474.
 */

#include "bsp.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

void bsp_init(void) {
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
        bsp_pwm_set_us(i, 1500U);
    }
}

void bsp_emergency_brake_trip(void) {
    for (uint8_t ch = 0; ch < 8; ch++) {
        bsp_pwm_set_us(ch, 1500U);
    }
    bsp_solenoid_set(0);
}

bool bsp_is_emergency_brake_tripped(void) {
    return false;
}
