#include "bsp.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

static unsigned s_failures;

#define CHECK(condition, description)                                                                                   \
    do {                                                                                                                \
        if (!(condition)) {                                                                                             \
            printf("[FAIL] %s\n", description);                                                                        \
            s_failures++;                                                                                              \
        }                                                                                                               \
    } while (0)

int main(void) {
    fake_hal_reset();
    fake_hal_set_ina237(5.07f, 1.25f, true);
    fake_hal_set_tmp1075(37.5f, true);
    bsp_init();

    CHECK(fake_hal_can_init_count() == 1U, "bsp_init must initialize the node CAN controller");
    CHECK(bsp_get_logic_voltage_mv() == 5070U, "Logic voltage must come from INA237 data, not a nominal constant");
    CHECK(fabsf(bsp_get_pcb_temperature_c() - 37.5f) < 0.1f,
          "PCB temperature must come from TMP1075 data, not a nominal constant");
    CHECK(fake_hal_gpio_was_configured(GPIOB, BRICK1_EN_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL),
          "Power brick enable pins must be initialized as outputs");
    CHECK(fake_hal_gpio_was_configured(GPIOA, LM74700_STAT_Pin, GPIO_MODE_INPUT, GPIO_PULLUP),
          "Ideal diode status must be initialized as a pulled-up input");

    const uint16_t brick_pins[4] = {BRICK1_EN_Pin, BRICK2_EN_Pin, BRICK3_EN_Pin, BRICK4_EN_Pin};
    for (uint8_t brick = 0; brick < 4U; brick++) {
        bsp_power_brick_enable(brick);
        CHECK(fake_hal_get_output(GPIOB, brick_pins[brick]) == GPIO_PIN_SET,
              "Each converter enable command must reach its mapped GPIO");
    }
    bsp_power_brick_disable_all();
    for (uint8_t brick = 0; brick < 4U; brick++) {
        CHECK(fake_hal_get_output(GPIOB, brick_pins[brick]) == GPIO_PIN_RESET,
              "Global converter shutdown must de-energize every enable pin");
    }

    fake_hal_set_input(GPIOA, LM74700_STAT_Pin, GPIO_PIN_SET);
    CHECK(bsp_lm74700_status_ok(), "High ideal-diode status must report healthy");
    fake_hal_set_input(GPIOA, LM74700_STAT_Pin, GPIO_PIN_RESET);
    CHECK(!bsp_lm74700_status_ok(), "Low ideal-diode status must report a fault");

    if (s_failures != 0U) {
        printf("Node 3 target BSP readiness: %u failure(s)\n", s_failures);
        return 1;
    }
    puts("Node 3 target BSP readiness passed");
    return 0;
}
