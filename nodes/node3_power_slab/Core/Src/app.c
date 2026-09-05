/**
 * @file app.c
 * @brief Node 3 (Power Slab) Application Layer Logic.
 *
 * Handles PMBus telemetry for 5 converter bricks (4x 12V 300W + 1x 5.2V 50W),
 * PCB copper thermal monitoring, LM74700 diode status, and 20 Hz Power Telemetry CAN stream.
 * Pure application logic with zero vendor ST HAL calls.
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include "tps25990.h"
#include "x19_can_protocol.h"
#include "x19_safety.h"

static x19_safety_state_t g_safety_state;
static x19_power_telemetry_t g_power_telemetry;
static tps25990_dev_t g_pmbus_bricks[5];

void app_main(void) {
    bsp_init();
    x19_safety_init(&g_safety_state);

    /* Initialize PMBus monitoring for 5 bricks */
    for (int i = 0; i < 5; i++) {
        tps25990_init(&g_pmbus_bricks[i], 0x40 + i);
    }

    uint32_t last_power_time = 0;

    while (1) {
        /* 20 Hz Power Telemetry stream (CAN ID 0x300) */
        if (time_get_ms() - last_power_time >= 50) {
            last_power_time = time_get_ms();

            /* Stream 0x300 Power Telemetry over CAN FD */
            can_send(X19_CAN_ID_POWER_TELEMETRY, (const uint8_t *)&g_power_telemetry, sizeof(g_power_telemetry));
            led_toggle();
        }

        delay_ms(5);
    }
}
