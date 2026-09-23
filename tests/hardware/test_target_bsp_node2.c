#include "bsp.h"
#include "main.h"
#include <stdio.h>

static unsigned s_failures;

#define CHECK(condition, description)                                                                                   \
    do {                                                                                                                \
        if (!(condition)) {                                                                                             \
            printf("[FAIL] %s\n", description);                                                                        \
            s_failures++;                                                                                              \
        }                                                                                                               \
    } while (0)

static uint16_t expected_compare(uint8_t channel) {
    return channel < 4U ? fake_htim1.compare[channel] : fake_htim8.compare[channel - 4U];
}

int main(void) {
    fake_hal_reset();
    bsp_init();

    CHECK(fake_hal_can_init_count() == 1U, "bsp_init must initialize the node CAN controller");
    CHECK(fake_htim1.pwm_started_mask == 0x0FU, "bsp_init must start all four TIM1 ESC outputs");
    CHECK(fake_htim8.pwm_started_mask == 0x0FU, "bsp_init must start all four TIM8 ESC outputs");
    for (uint8_t ch = 0; ch < 8U; ch++) {
        CHECK(expected_compare(ch) == 1500U, "Every ESC output must initialize at neutral pulse width");
    }

    for (uint8_t ch = 0; ch < 8U; ch++) {
        uint16_t pulse = (uint16_t)(1200U + (ch * 50U));
        bsp_pwm_set_us(ch, pulse);
        CHECK(bsp_pwm_get_us(ch) == pulse, "PWM readback must match its channel command");
        CHECK(expected_compare(ch) == pulse, "Each PWM channel must update its mapped timer compare register");
    }
    bsp_pwm_set_us(8U, 1800U);
    CHECK(bsp_pwm_get_us(8U) == 1500U, "Invalid PWM channels must not expose an output");

    CHECK(fake_hal_gpio_was_configured(GPIOB, SOL_0_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL),
          "Solenoid outputs must be initialized as GPIO outputs");
    for (uint8_t ch = 0; ch < 10U; ch++) {
        bsp_solenoid_set((uint16_t)(1U << ch));
        CHECK(fake_hal_get_output(GPIOB, (uint16_t)(1U << ch)) == GPIO_PIN_SET,
              "Every solenoid bit must energize its own GPIO output");
        CHECK(bsp_solenoid_get() == (uint16_t)(1U << ch), "Solenoid readback must match the active mask");
    }

    bsp_emergency_brake_trip();
    CHECK(fake_hal_get_output(GPIOC, EMERGENCY_CUTOFF_Pin) == GPIO_PIN_SET,
          "Emergency brake must assert the physical hardware cutoff");
    CHECK(bsp_is_emergency_brake_tripped(), "Emergency brake status must expose the hardware latch");
    CHECK(bsp_solenoid_get() == 0U, "Emergency brake must de-energize all solenoids");
    for (uint8_t ch = 0; ch < 8U; ch++) {
        CHECK(bsp_pwm_get_us(ch) == 1500U, "Emergency brake must return every ESC output to neutral");
    }

    if (s_failures != 0U) {
        printf("Node 2 target BSP readiness: %u failure(s)\n", s_failures);
        return 1;
    }
    puts("Node 2 target BSP readiness passed");
    return 0;
}
