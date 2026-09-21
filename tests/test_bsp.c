/**
 * @file test_bsp.c
 * @brief Unit tests for BSP PWM behavior.
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "mocks/mock_bsp.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>

static void setup(void) {
    mock_bsp_reset();
}

void test_bsp_pwm_clamping(void) {
    setup();

    bsp_pwm_set_us(0, 999);
    assert(mock_bsp_get_pwm_us(0) == ROV_PWM_MIN_US);

    bsp_pwm_set_us(1, 1000);
    assert(mock_bsp_get_pwm_us(1) == ROV_PWM_MIN_US);

    bsp_pwm_set_us(2, 1500);
    assert(mock_bsp_get_pwm_us(2) == ROV_PWM_STOP_US);

    bsp_pwm_set_us(3, 2000);
    assert(mock_bsp_get_pwm_us(3) == ROV_PWM_MAX_US);

    bsp_pwm_set_us(4, 2001);
    assert(mock_bsp_get_pwm_us(4) == ROV_PWM_MAX_US);

    printf("[PASS] test_bsp_pwm_clamping\n");
}

void test_bsp_pwm_channel_mapping(void) {
    setup();

    for (uint8_t channel = 0; channel < ROV_NUM_THRUSTERS; channel++) {
        uint16_t pulse_us = (uint16_t)(1100U + (channel * 100U));
        bsp_pwm_set_us(channel, pulse_us);
    }

    for (uint8_t channel = 0; channel < ROV_NUM_THRUSTERS; channel++) {
        uint16_t expected = (uint16_t)(1100U + (channel * 100U));
        assert(mock_bsp_get_pwm_us(channel) == expected);
    }

    printf("[PASS] test_bsp_pwm_channel_mapping\n");
}

int main(void) {
    test_bsp_pwm_clamping();
    test_bsp_pwm_channel_mapping();

    printf("All BSP tests passed.\n");
    return 0;
}
