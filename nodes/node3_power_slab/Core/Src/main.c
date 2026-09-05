/**
 * @file main.c
 * @brief Power Slab Main Application (Node 3 - STM32G4).
 * Handles PMBus telemetry for 5 converter bricks (4x 12V 300W + 1x 5.2V 50W),
 * PCB copper thermal monitoring, LM74700 diode status, and 20 Hz Power Telemetry CAN stream.
 * @organization Purdue ROV
 */

#include "main.h"
#include "tcan1044.h"
#include "tps25990.h"

static x19_safety_state_t g_safety_state;
static x19_power_telemetry_t g_power_telemetry;
static tps25990_dev_t g_pmbus_bricks[5];

int main(void) {
    x19_safety_init(&g_safety_state);

    /* Initialize PMBus monitoring for 5 bricks */
    for (int i = 0; i < 5; i++) {
        tps25990_init(&g_pmbus_bricks[i], 0x40 + i);
    }

    while (1) {
        (void)g_power_telemetry;
        (void)g_pmbus_bricks;
    }
}

void Error_Handler(void) {
    while (1) {
    }
}
