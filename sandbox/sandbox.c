/**
 * @file sandbox.c
 * @brief Zero-boilerplate Developer Scratchpad / Rapid Prototyping Entrypoint.
 *
 * Write any test, hardware experiment, or scratch code here.
 * Compile, flash, and monitor on any dev board with:
 *   python rov.py sandbox -b f411                  (STM32F411 Nucleo)
 *   python rov.py sandbox -b g474                  (STM32G474 Nucleo)
 *   python rov.py sandbox -f my_experiment.c -b f411 (Custom scratch file)
 *   python rov.py sandbox -b host                  (Host SIL simulation)
 */

#include "bsp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void app_main(void) {
    /* 1. Initialize board hardware, clocks, and debug UART printf */
    bsp_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Developer Sandbox / Rapid Prototyping    \r\n");
    printf("======================================================\r\n");
    printf("Running on target hardware. Edit sandbox/sandbox.c or pass\r\n");
    printf("'-f <file.c>' with 'python rov.py run' to test custom code.\r\n\r\n");

    /* 2. Main loop */
    uint32_t counter = 0;
    while (1) {
        led_toggle();
        delay_ms(500);
        counter++;
        if (counter % 4 == 0) {
            printf("[Sandbox] Uptime: %lu ms | Heartbeat alive\r\n", (unsigned long)time_get_ms());
        }
    }
}
