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
#include <math.h>
#include <string.h>

static rov_safety_state_t g_safety_state;
static rov_power_telemetry_t g_power_telemetry;
static pmbus_brick_dev_t g_pmbus_bricks[5];
static uint32_t g_last_power_time = 0;
static bool g_can_ready = false;
static bool g_fault_latched = false;
static bool g_fault_alert_latched = false;

static bool node3_u16_value_valid(float value) {
    return isfinite(value) && value >= 0.0f && value <= (float)UINT16_MAX;
}

static bool node3_i16_value_valid(float value) {
    return isfinite(value) && value >= -32768.0f && value <= (float)INT16_MAX;
}

static uint16_t node3_to_u16(float value) {
    return node3_u16_value_valid(value) ? (uint16_t)value : 0U;
}

static int16_t node3_to_i16(float value) {
    return node3_i16_value_valid(value) ? (int16_t)value : 0;
}

void node3_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);

    g_can_ready = can_init();

    for (int i = 0; i < 5; i++) {
        pmbus_brick_init(&g_pmbus_bricks[i], (uint8_t)(0x40 + i));
    }

    memset(&g_power_telemetry, 0, sizeof(g_power_telemetry));
    g_power_telemetry.tether_voltage_mv = 48000;
    g_power_telemetry.v5_voltage_mv = 5200;
    g_last_power_time = 0;
    g_fault_latched = !g_can_ready;
    g_fault_alert_latched = false;
    if (!g_can_ready) {
        bsp_power_brick_disable_all();
        g_power_telemetry.status_flags = 0x0001U;
    }
}

void node3_app_step(void) {
    uint32_t current_time = time_get_ms();

    if (!g_can_ready) {
        bsp_power_brick_disable_all();
        delay_ms(5);
        return;
    }

    /* 20 Hz Power Telemetry and Protection Loop */
    if (current_time - g_last_power_time >= (1000 / ROV_POWER_TELEMETRY_FREQ_HZ)) {
        g_last_power_time = current_time;

        float total_tether_power_w = 0.0f;
        float total_tether_current_a = 0.0f;
        float measured_tether_voltage_v = 0.0f;
        bool fault_detected = false;
        bool thermal_fault = false;
        bool overcurrent_fault = false;
        int16_t max_temp_c_tenths = 250;

        for (int i = 0; i < 5; i++) {
            rov_status_t status = pmbus_brick_read_telemetry(&g_pmbus_bricks[i]);
            if (status != ROV_OK || !isfinite(g_pmbus_bricks[i].input_voltage_v) ||
                !isfinite(g_pmbus_bricks[i].output_voltage_v) || !isfinite(g_pmbus_bricks[i].output_current_a) ||
                !isfinite(g_pmbus_bricks[i].temperature_c)) {
                fault_detected = true;
                continue;
            }

            if (i == 0) {
                measured_tether_voltage_v = g_pmbus_bricks[i].input_voltage_v;
                float v5_voltage_mv = g_pmbus_bricks[i].output_voltage_v * 1000.0f;
                float v5_current_ma = g_pmbus_bricks[i].output_current_a * 1000.0f;
                if (!node3_u16_value_valid(v5_voltage_mv) || !node3_u16_value_valid(v5_current_ma)) {
                    fault_detected = true;
                }
                g_power_telemetry.v5_voltage_mv = node3_to_u16(v5_voltage_mv);
                g_power_telemetry.v5_current_ma = node3_to_u16(v5_current_ma);
                if (g_pmbus_bricks[i].output_current_a > ROV_LOGIC_RAIL_MAX_CURRENT_A) {
                    overcurrent_fault = true;
                }
            } else {
                float v12_current_ma = g_pmbus_bricks[i].output_current_a * 1000.0f;
                if (!node3_u16_value_valid(v12_current_ma)) {
                    fault_detected = true;
                }
                g_power_telemetry.v12_current_ma[i - 1] = node3_to_u16(v12_current_ma);
                if (g_pmbus_bricks[i].output_current_a > ROV_BRICK_MAX_CURRENT_A) {
                    overcurrent_fault = true;
                }
            }

            total_tether_power_w += g_pmbus_bricks[i].output_voltage_v * g_pmbus_bricks[i].output_current_a;

            float brick_temp_tenths_value = g_pmbus_bricks[i].temperature_c * 10.0f;
            if (!node3_i16_value_valid(brick_temp_tenths_value)) {
                fault_detected = true;
            }
            int16_t brick_temp_tenths = node3_to_i16(brick_temp_tenths_value);
            if (brick_temp_tenths > max_temp_c_tenths) {
                max_temp_c_tenths = brick_temp_tenths;
            }
            if (g_pmbus_bricks[i].temperature_c > ROV_PCB_MAX_SAFE_TEMP_C) {
                thermal_fault = true;
            }
            if (g_pmbus_bricks[i].status_word != 0U) {
                fault_detected = true;
            }
        }

        float pcb_temperature_c = bsp_get_pcb_temperature_c();
        if (!isfinite(pcb_temperature_c)) {
            fault_detected = true;
        } else {
            float pcb_temp_tenths_value = pcb_temperature_c * 10.0f;
            if (!node3_i16_value_valid(pcb_temp_tenths_value)) {
                fault_detected = true;
            }
            int16_t pcb_temp_tenths = node3_to_i16(pcb_temp_tenths_value);
            if (pcb_temp_tenths > max_temp_c_tenths) {
                max_temp_c_tenths = pcb_temp_tenths;
            }
            if (pcb_temperature_c > ROV_PCB_MAX_SAFE_TEMP_C) {
                thermal_fault = true;
            }
        }

        if (!bsp_lm74700_status_ok() || bsp_get_logic_voltage_mv() == 0U) {
            fault_detected = true;
        }

        if (isfinite(total_tether_power_w) && measured_tether_voltage_v > 0.0f) {
            total_tether_current_a = total_tether_power_w / measured_tether_voltage_v;
        } else {
            fault_detected = true;
        }
        if (!isfinite(total_tether_current_a) || total_tether_power_w > ROV_TETHER_MAX_POWER_W ||
            total_tether_current_a > ROV_TETHER_MAX_CURRENT_A) {
            overcurrent_fault = true;
        }

        fault_detected = fault_detected || thermal_fault || overcurrent_fault;
        if (fault_detected) {
            g_fault_latched = true;
        }
        fault_detected = g_fault_latched;
        g_safety_state.overtemperature_tripped = thermal_fault;
        float tether_voltage_mv = measured_tether_voltage_v * 1000.0f;
        float tether_current_ma = total_tether_current_a * 1000.0f;
        if (!node3_u16_value_valid(tether_voltage_mv) || !node3_u16_value_valid(tether_current_ma)) {
            fault_detected = true;
        }
        g_power_telemetry.tether_voltage_mv = node3_to_u16(tether_voltage_mv);
        g_power_telemetry.tether_current_ma = node3_to_u16(tether_current_ma);
        g_power_telemetry.pcb_temp_c = max_temp_c_tenths;

        if (fault_detected) {
            g_power_telemetry.status_flags |= 0x0001; /* Fault bit */
            bsp_power_brick_disable_all();

            /* Broadcast Priority 0 eFuse Fault Alert (0x005) once per fault. */
            if (!g_fault_alert_latched) {
                uint8_t alert[8] = {0xEF, 0x01, (uint8_t)(g_power_telemetry.status_flags & 0xFF), 0, 0, 0, 0, 0};
                if (can_send_emergency(ROV_CAN_ID_EFUSE_FAULT_ALERT, alert, sizeof(alert))) {
                    g_fault_alert_latched = true;
                }
            }
        } else {
            g_power_telemetry.status_flags &= (uint16_t)~0x0001U;
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
