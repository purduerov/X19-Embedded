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
#include "ina237.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"
#include <math.h>

static rov_safety_state_t g_safety_state;
static rov_env_telemetry_t g_env_telemetry;
static bme280_dev_t g_bme280_dev;
static ina237_dev_t g_ina237_dev;

static uint32_t g_last_telemetry_time = 0;
static float g_baseline_pressure_hpa = 0.0f;
static bool g_bme280_valid = false;
static bool g_ina237_valid = false;
static bool g_can_ready = false;

/*
 * Once an emergency has been transmitted, latch the state so that the
 * 10 Hz backup polling path does not continuously flood CAN ID 0x001.
 */
static volatile bool g_emergency_latched = false;

/**
 * @brief Read both physical floor leak probes and construct the protocol
 *        leak bitmask.
 *
 * Bit 1 (0x02): floor leak probe 0
 * Bit 2 (0x04): floor leak probe 1
 *
 * bsp_leak_probe_read() returns true when water is detected, so the
 * application layer does not need to know the electrical polarity of
 * the physical GPIO.
 *
 * @return Floor leak bitmask.
 */
static uint8_t node1_get_floor_leak_bits(void) {
    uint8_t leak_bits = 0;

    if (bsp_leak_probe_read(0)) {
        leak_bits |= 0x02;
    }

    if (bsp_leak_probe_read(1)) {
        leak_bits |= 0x04;
    }

    return leak_bits;
}

/**
 * @brief Execute the Node 1 emergency leak response.
 *
 * The local hardware emergency brake is tripped first so that the local
 * safety action does not depend on CAN bus availability.
 *
 * The Emergency Break frame must use CAN ID 0x001 and contain the
 * authorization signature 0xAA, 0x55 expected by the Control Board.
 *
 * @param leak_bits Leak source bitmask.
 */
static void node1_trigger_emergency(uint8_t leak_bits) {
    uint8_t alert_payload[8] = {0xAA, 0x55, leak_bits, 0x00, 0x00, 0x00, 0x00, 0x00};

    /*
     * Trip the local hardware safety path immediately.
     * This must not depend on successful CAN transmission.
     */
    bsp_emergency_brake_trip();

    /*
     * Update the software safety state.
     */
    g_safety_state.leak_detected = true;
    rov_safety_trigger_emergency_break(&g_safety_state);

    /*
     * Safety-critical CAN transmission.
     *
     * The real STM32 implementation of can_send_emergency() must bypass
     * normal telemetry software queues and submit directly to the FDCAN
     * hardware transmit resource.
     */
    g_emergency_latched = true;
    if (!can_send_emergency(ROV_CAN_ID_EMERGENCY_BREAK, alert_payload, sizeof(alert_payload))) {
        g_emergency_latched = false;
    }
}

/**
 * @brief Immediate physical floor-leak interrupt entry point.
 *
 * The STM32 BSP/EXTI layer calls this function when PA4 or PA5 generates
 * the configured leak-detection edge.
 *
 * No debounce delay is intentionally used here. A detected water event
 * must immediately trigger the emergency path.
 */
void node1_leak_irq_handler(void) {
    uint8_t leak_bits = node1_get_floor_leak_bits();

    if (leak_bits != 0u && !g_emergency_latched) {
        node1_trigger_emergency(leak_bits);
    }
}

void node1_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);

    g_can_ready = can_init();
    if (!g_can_ready) {
        /* A node without a verified CAN transport must not report healthy. */
        bsp_emergency_brake_trip();
        g_safety_state.emergency_break_active = true;
        g_emergency_latched = true;
    }

    bme280_init(&g_bme280_dev);
    ina237_init(&g_ina237_dev, 0x40, 0.001f);
    g_bme280_valid = false;
    g_ina237_valid = false;

    g_last_telemetry_time = 0;
    g_baseline_pressure_hpa = 0.0f;
    g_emergency_latched = false;

    g_env_telemetry.pressure_hpa = 1013.25f;
    g_env_telemetry.humidity_pct = 30.0f;
    g_env_telemetry.temperature_c = 25.0f;
    g_env_telemetry.leak_flags = 0;
}

void node1_app_step(void) {
    uint32_t current_time = time_get_ms();

    if (!g_can_ready) {
        bsp_emergency_brake_trip();
        delay_ms(5);
        return;
    }

    /* Sample sensors and evaluate leak state at 10 Hz */
    if (current_time - g_last_telemetry_time >= (1000 / ROV_ENV_TELEMETRY_FREQ_HZ)) {

        g_last_telemetry_time = current_time;

        g_bme280_valid = bme280_read_all(&g_bme280_dev) == ROV_OK;
        g_ina237_valid = ina237_read_power(&g_ina237_dev) == ROV_OK;
        bool sensor_fault = !g_bme280_valid || !g_ina237_valid || !isfinite(g_bme280_dev.pressure_hpa) ||
                            !isfinite(g_bme280_dev.humidity_pct) || !isfinite(g_bme280_dev.temperature_c) ||
                            !isfinite(g_ina237_dev.bus_voltage_v) || !isfinite(g_ina237_dev.shunt_current_a);

        /* Track the lowest valid pressure so a vacuum applied after startup is detectable. */
        if (g_bme280_valid && g_bme280_dev.pressure_hpa > 0.0f &&
            (g_baseline_pressure_hpa <= 0.0f || g_bme280_dev.pressure_hpa < g_baseline_pressure_hpa)) {
            g_baseline_pressure_hpa = g_bme280_dev.pressure_hpa;
        }

        uint8_t leak_bits = sensor_fault ? 0x01u : 0u;

        /*
         * Check BME280 humidity threshold (> 80%).
         *
         * Environmental leak detection uses bit 0.
         */
        if (g_bme280_dev.humidity_pct >= ROV_LEAK_HUMIDITY_MAX_PCT) {

            leak_bits |= 0x01;
        }

        /*
         * Check vacuum decay.
         *
         * If the enclosure was pulled to vacuum and pressure rises by
         * more than the configured threshold, report an environmental
         * leak using bit 0.
         */
        if (g_baseline_pressure_hpa > 0.0f &&
            (g_bme280_dev.pressure_hpa - g_baseline_pressure_hpa) >= ROV_LEAK_PRESSURE_DROP_THRESHOLD_HPA) {

            leak_bits |= 0x01;
        }

        /*
         * Redundant polling backup for physical floor leak probes.
         *
         * The EXTI interrupt is the primary fast path. This 10 Hz read
         * ensures a physical leak is still detected if an interrupt is
         * missed or disabled.
         */
        leak_bits |= node1_get_floor_leak_bits();

        g_env_telemetry.pressure_hpa = g_bme280_valid ? g_bme280_dev.pressure_hpa : 0.0f;

        g_env_telemetry.humidity_pct = g_bme280_valid ? g_bme280_dev.humidity_pct : 0.0f;

        g_env_telemetry.temperature_c = g_bme280_valid ? g_bme280_dev.temperature_c : 0.0f;

        g_env_telemetry.leak_flags = leak_bits;

        /*
         * Trigger the emergency response for any detected leak.
         *
         * g_emergency_latched prevents the 10 Hz loop from repeatedly
         * flooding CAN ID 0x001 after the initial emergency frame.
         */
        if (leak_bits != 0u && !g_emergency_latched) {
            node1_trigger_emergency(leak_bits);
        }

        /*
         * Stream 0x210 Environment & Leak Telemetry over CAN FD.
         */
        uint8_t tx_buf[64];
        size_t packed_len = 0;

        if (rov_can_pack_env_telemetry(&g_env_telemetry, tx_buf, sizeof(tx_buf), &packed_len) == ROV_OK) {

            can_send(ROV_CAN_ID_ENV_TELEMETRY, tx_buf, (uint8_t)packed_len);
        }

        led_toggle();
    }

    delay_ms(5);
}

#ifndef ROV_UNIT_TEST
void app_main(void) {
    node1_app_init();

    while (1) {
        node1_app_step();
    }
}
#endif
