/**
 * @file app.c
 * @brief Node 3 (Power Slab) Application Layer Logic.
 *
 * Handles PMBus telemetry for 5 converter bricks (4x 12V 300W + 1x 5.2V 50W),
 * PCB copper thermal monitoring, LM74700 diode status, 20 Hz Power Telemetry CAN stream,
 * and 0x005 eFuse Fault Alert broadcast upon overcurrent / overtemp trip.
 * Pure application logic with zero vendor ST HAL calls.
 * @organization Purdue ROV
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include "pmbus_brick.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"
#include <string.h>

static rov_safety_state_t g_safety_state;
static rov_power_telemetry_t g_power_telemetry;
static pmbus_brick_dev_t g_pmbus_bricks[5];
static uint32_t g_last_power_time = 0;

void node3_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);

    for (int i = 0; i < 5; i++) {
        pmbus_brick_init(&g_pmbus_bricks[i], (uint8_t)(0x40 + i));
    }

    memset(&g_power_telemetry, 0, sizeof(g_power_telemetry));
    g_power_telemetry.tether_voltage_mv = 48000;
    g_power_telemetry.v5_voltage_mv = 5200;
    g_last_power_time = 0;
}

void node3_app_step(void) {
    uint32_t current_time = time_get_ms();

    /* 20 Hz Power Telemetry and Protection Loop */
    if (current_time - g_last_power_time >= (1000 / ROV_POWER_TELEMETRY_FREQ_HZ)) {
        g_last_power_time = current_time;

        float total_tether_current_a = 0.0f;
        bool fault_detected = false;
        int16_t max_temp_c_tenths = 250;

        for (int i = 0; i < 5; i++) {
            pmbus_brick_read_telemetry(&g_pmbus_bricks[i]);

            /* Brick 0 is 5.2V logic; Bricks 1..4 are 12V 300W thruster bricks */
            if (i == 0) {
                g_power_telemetry.v5_voltage_mv = (uint16_t)(g_pmbus_bricks[i].output_voltage_v * 1000.0f);
                g_power_telemetry.v5_current_ma = (uint16_t)(g_pmbus_bricks[i].output_current_a * 1000.0f);
                total_tether_current_a +=
                    (g_pmbus_bricks[i].output_voltage_v * g_pmbus_bricks[i].output_current_a) / 48.0f;
            } else {
                g_power_telemetry.v12_current_ma[i - 1] = (uint16_t)(g_pmbus_bricks[i].output_current_a * 1000.0f);
                total_tether_current_a +=
                    (g_pmbus_bricks[i].output_voltage_v * g_pmbus_bricks[i].output_current_a) / 48.0f;

                /* Overcurrent protection: 25A max per brick */
                if (g_pmbus_bricks[i].output_current_a > ROV_BRICK_MAX_CURRENT_A) {
                    fault_detected = true;
                }
            }

            int16_t brick_temp_tenths = (int16_t)(g_pmbus_bricks[i].temperature_c * 10.0f);
            if (brick_temp_tenths > max_temp_c_tenths) {
                max_temp_c_tenths = brick_temp_tenths;
            }
            if (g_pmbus_bricks[i].temperature_c > ROV_PCB_MAX_SAFE_TEMP_C) {
                fault_detected = true;
            }
        }

        g_power_telemetry.tether_voltage_mv = 48000;
        g_power_telemetry.tether_current_ma = (uint16_t)(total_tether_current_a * 1000.0f);
        g_power_telemetry.pcb_temp_c = max_temp_c_tenths;

        if (fault_detected) {
            g_power_telemetry.status_flags |= 0x0001; /* Fault bit */
            g_safety_state.overtemperature_tripped = true;

            /* Broadcast Priority 0 eFuse Fault Alert (0x005) */
            uint8_t alert[8] = {0xEF, 0x01, (uint8_t)(g_power_telemetry.status_flags & 0xFF), 0, 0, 0, 0, 0};
            can_send(ROV_CAN_ID_EFUSE_FAULT_ALERT, alert, sizeof(alert));
        }

        /* Stream 0x300 Power Telemetry over CAN FD */
        uint8_t tx_buf[64];
        size_t packed_len = 0;
        if (rov_can_pack_power_telemetry(&g_power_telemetry, tx_buf, sizeof(tx_buf), &packed_len) == ROV_OK) {
            can_send(ROV_CAN_ID_POWER_TELEMETRY, tx_buf, (uint8_t)packed_len);
        }

        led_toggle();
    }

    delay_ms(5);
}

#ifndef ROV_UNIT_TEST
void app_main(void) {
    node3_app_init();
    while (1) {
        node3_app_step();
    }
}
#endif
