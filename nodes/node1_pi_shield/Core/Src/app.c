/**
 * @file app.c
 * @brief Node 1 (Pi Shield) Application Layer Logic.
 *
 * Handles BME280 sealed enclosure vacuum/humidity leak testing, INA226 5V monitor,
 * GPIO floor leak traces, 9-clock I2C recovery, 10 Hz Leak Telemetry CAN stream,
 * and immediate broadcast of Emergency Break (0x001) upon leak detection.
 * Pure application logic with zero vendor ST HAL calls.
 * @organization Purdue ROV
 */

#include "app.h"
#include "bme280.h"
#include "bsp.h"
#include "can_interface.h"
#include "ina226.h"
#include "x19_can_protocol.h"
#include "x19_parameters.h"
#include "x19_safety.h"

static x19_safety_state_t g_safety_state;
static x19_env_telemetry_t g_env_telemetry;
static bme280_dev_t g_bme280_dev;
static ina226_dev_t g_ina226_dev;
static uint32_t g_last_telemetry_time = 0;
static float g_baseline_pressure_hpa = 0.0f;

void node1_app_init(void) {
    bsp_init();
    x19_safety_init(&g_safety_state);

    bme280_init(&g_bme280_dev);
    ina226_init(&g_ina226_dev, 0x40, 0.002f);

    g_last_telemetry_time = 0;
    g_baseline_pressure_hpa = 0.0f;
    g_env_telemetry.pressure_hpa = 1013.25f;
    g_env_telemetry.humidity_pct = 30.0f;
    g_env_telemetry.temperature_c = 25.0f;
    g_env_telemetry.leak_flags = 0;
}

void node1_app_step(void) {
    uint32_t current_time = time_get_ms();

    /* Sample sensors and evaluate leak state at 10 Hz */
    if (current_time - g_last_telemetry_time >= (1000 / X19_ENV_TELEMETRY_FREQ_HZ)) {
        g_last_telemetry_time = current_time;

        bme280_read_all(&g_bme280_dev);
        ina226_read_power(&g_ina226_dev);

        /* Set baseline pressure on first valid sample */
        if (g_baseline_pressure_hpa <= 0.0f && g_bme280_dev.pressure_hpa > 0.0f) {
            g_baseline_pressure_hpa = g_bme280_dev.pressure_hpa;
        }

        uint8_t leak_bits = 0;

        /* Check BME280 humidity threshold (> 80%) */
        if (g_bme280_dev.humidity_pct >= X19_LEAK_HUMIDITY_MAX_PCT) {
            leak_bits |= 0x01;
        }

        /* Check vacuum decay: if enclosure was pulled to vacuum (<900 hPa) and rises toward atmosphere */
        if (g_baseline_pressure_hpa > 0.0f &&
            (g_bme280_dev.pressure_hpa - g_baseline_pressure_hpa) >= X19_LEAK_PRESSURE_DROP_THRESHOLD_HPA) {
            leak_bits |= 0x01;
        }

        /* Check physical floor leak probes */
        if (bsp_leak_probe_read(0)) {
            leak_bits |= 0x02;
        }
        if (bsp_leak_probe_read(1)) {
            leak_bits |= 0x04;
        }

        g_env_telemetry.pressure_hpa = g_bme280_dev.pressure_hpa;
        g_env_telemetry.humidity_pct = g_bme280_dev.humidity_pct;
        g_env_telemetry.temperature_c = g_bme280_dev.temperature_c;
        g_env_telemetry.leak_flags = leak_bits;

        if (leak_bits != 0) {
            g_safety_state.leak_detected = true;
            x19_safety_trigger_emergency_break(&g_safety_state);
            bsp_emergency_brake_trip();

            /* Immediately broadcast Priority 0 Emergency Break (0x001) */
            uint8_t alert_payload[8] = {0xAA, 0x55, leak_bits, 0x00, 0x00, 0x00, 0x00, 0x00};
            can_send(X19_CAN_ID_EMERGENCY_BREAK, alert_payload, sizeof(alert_payload));
        }

        /* Stream 0x210 Environment & Leak Telemetry over CAN FD */
        uint8_t tx_buf[64];
        size_t packed_len = 0;
        if (x19_can_pack_env_telemetry(&g_env_telemetry, tx_buf, sizeof(tx_buf), &packed_len) == X19_OK) {
            can_send(X19_CAN_ID_ENV_TELEMETRY, tx_buf, (uint8_t)packed_len);
        }

        led_toggle();
    }

    delay_ms(5);
}

#ifndef X19_UNIT_TEST
void app_main(void) {
    node1_app_init();
    while (1) {
        node1_app_step();
    }
}
#endif
