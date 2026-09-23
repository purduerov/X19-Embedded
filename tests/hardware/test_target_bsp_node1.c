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

int main(void) {
    fake_hal_reset();
    bsp_init();

    CHECK(fake_hal_can_init_count() == 1U, "bsp_init must initialize the node CAN controller");
    CHECK(fake_hal_gpio_was_configured(GPIOA, LEAK_PROBE0_Pin, GPIO_MODE_INPUT, GPIO_PULLUP),
          "Leak probe 0 must be configured as a pulled-up input");
    CHECK(fake_hal_gpio_was_configured(GPIOA, LEAK_PROBE1_Pin, GPIO_MODE_INPUT, GPIO_PULLUP),
          "Leak probe 1 must be configured as a pulled-up input");
    CHECK(fake_hal_gpio_was_configured(GPIOC, EMERGENCY_CUTOFF_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL),
          "Emergency cutoff must be configured as an output");
    CHECK(fake_hal_get_output(GPIOC, EMERGENCY_CUTOFF_Pin) == GPIO_PIN_RESET,
          "Emergency cutoff must boot deasserted");

    fake_hal_set_input(GPIOA, LEAK_PROBE0_Pin, GPIO_PIN_SET);
    fake_hal_set_input(GPIOA, LEAK_PROBE1_Pin, GPIO_PIN_SET);
    CHECK(!bsp_leak_probe_read(0), "A dry probe must report no leak");
    fake_hal_set_input(GPIOA, LEAK_PROBE0_Pin, GPIO_PIN_RESET);
    CHECK(bsp_leak_probe_read(0), "An active-low wet probe must report a leak");
    fake_hal_set_input(GPIOA, LEAK_PROBE1_Pin, GPIO_PIN_RESET);
    CHECK(bsp_leak_probe_read(1), "Probe 1 must be read independently");
    CHECK(!bsp_leak_probe_read(2), "An invalid probe index must fail safe as dry");

    bsp_emergency_brake_trip();
    CHECK(fake_hal_get_output(GPIOC, EMERGENCY_CUTOFF_Pin) == GPIO_PIN_SET,
          "Emergency trip must assert the physical cutoff output");
    CHECK(bsp_is_emergency_brake_tripped(), "Emergency status must reflect the hardware cutoff state");

    if (s_failures != 0U) {
        printf("Node 1 target BSP readiness: %u failure(s)\n", s_failures);
        return 1;
    }
    puts("Node 1 target BSP readiness passed");
    return 0;
}
