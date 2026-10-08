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
#include <stdio.h>

static rov_safety_state_t g_safety_state;
static rov_env_telemetry_t g_env_telemetry;
static bme280_dev_t g_bme280_dev;
static ina237_dev_t g_ina237_dev;

static uint32_t g_last_telemetry_time = 0;
static bool g_bme280_valid = false;
static bool g_ina237_valid = false;
static bool g_can_ready = false;

/*
 * Once an emergency has been transmitted, latch the state so that the
 * 10 Hz backup polling path does not continuously flood CAN ID 0x001.
 */
static volatile bool g_emergency_latched = false;

/**
 * @brief Vacuum Decay States 
 * 
 */
typedef enum{
    VAC_STATE_ATMOSPHERIC, 
    VAC_STATE_PUMPING, 
    VAC_STATE_TESTING, 
    VAC_STATE_PASSED, 
    VAC_STATE_FAILED 
} vacuum_state_t; 

static vacuum_state_t g_vac_state = VAC_STATE_ATMOSPHERIC; 

static float g_ambient_pressure_hpa = 0.0f; // pressure before vacuum is applied 
static float g_previous_pressure_hpa = 0.0f; // pressure used for stabilization / rate calculation 

static uint32_t g_stable_start_time = 0; // when pressure first become stable 
static uint32_t g_test_start_time = 0; // when the 10-min vacuum test started 
static uint32_t g_last_pressure_check_time = 0; 
static uint32_t g_last_decay_check_time = 0; // last time the 10-second decay calculation performed 

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

uint8_t node1_i2c_scan(void) {
    uint8_t count = 0;
    printf("\r\n========================================\r\n");
    printf("   Node 1 Pi Shield I2C Bus Scanner\r\n");
    printf("========================================\r\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    for (uint8_t row = 0; row < 128; row += 16) {
        printf("%02X: ", row);
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t addr = row + col;
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
            } else if (bsp_i2c_probe(addr)) {
                printf("%02X ", addr);
                count++;
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    printf("----------------------------------------\r\n");
    printf("Scan complete: found %u device(s).\r\n", count);

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (bsp_i2c_probe(addr)) {
            const char *desc = "Unknown device";
            if (addr == 0x76 || addr == 0x77) {
                desc = "Bosch BME280 (Pressure / Humidity / Temp)";
            } else if (addr >= 0x40 && addr <= 0x47) {
                desc = "TI INA237 / INA226 (Current / Voltage / Power Monitor)";
            } else if (addr >= 0x48 && addr <= 0x4F) {
                desc = "TI TMP1075 / BNO086 (Temperature / IMU Sensor)";
            }
            printf("  -> [0x%02X] %s\r\n", addr, desc);
        }
    }
    printf("========================================\r\n\r\n");
    return count;
}

void node1_app_init(void) {
    bsp_init();
    rov_safety_init(&g_safety_state);

    /* Run I2C bus scan at startup to verify all sensors are reachable */
    node1_i2c_scan();

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
    g_emergency_latched = false;

    g_env_telemetry.pressure_hpa = 1013.25f;
    g_env_telemetry.humidity_pct = 30.0f;
    g_env_telemetry.temperature_c = 25.0f;
    g_env_telemetry.leak_flags = 0;

    // Vacuum Decay State Machine Reset 
    g_vac_state = VAC_STATE_ATMOSPHERIC;
    g_ambient_pressure_hpa = 0.0f;
    g_previous_pressure_hpa = 0.0f;

    g_stable_start_time = 0U;
    g_test_start_time = 0U;
    g_last_pressure_check_time = 0U;
    g_last_decay_check_time = 0U;
}

void node1_update_vacuum_decay(
    float pressure_hpa, // current pressure reading 
    uint32_t current_time 
){
    switch(g_vac_state){
        case VAC_STATE_ATMOSPHERIC: {
            if(g_ambient_pressure_hpa <= 0.0f){
                g_ambient_pressure_hpa = pressure_hpa; 
            }
            if (g_ambient_pressure_hpa - pressure_hpa >= 100.0f) {
                g_vac_state = VAC_STATE_PUMPING;
                g_previous_pressure_hpa = pressure_hpa;
                g_last_pressure_check_time = current_time;
                g_stable_start_time = 0U;
            }
            break; 
        }
        case VAC_STATE_PUMPING: {
            if(current_time - g_last_pressure_check_time >= 1000U){ // time elapsed at least 1 second 
                float delta_pressure = fabsf(pressure_hpa - g_previous_pressure_hpa); 
                float rate_hpa_per_second = delta_pressure / ((float)(current_time - g_last_pressure_check_time) / 1000.0f); 

                g_previous_pressure_hpa = pressure_hpa; 
                g_last_pressure_check_time = current_time; 

                if(rate_hpa_per_second < 0.1f){
                    if(g_stable_start_time == 0U){
                        g_stable_start_time = current_time; 
                    }
                    if(current_time - g_stable_start_time >= 15000U){
                        g_vac_state = VAC_STATE_TESTING; 

                        g_test_start_time = current_time; 
                        g_last_decay_check_time = current_time; 
                        g_previous_pressure_hpa = pressure_hpa; 
                    }
                } else{
                    g_stable_start_time = 0U; 
                }
            }
            break; 
        }
        case VAC_STATE_TESTING: {
            if (current_time - g_last_decay_check_time >= 10000U) {
                uint32_t elapsed_ms =
                    current_time - g_last_decay_check_time;

                float delta_pressure =
                    pressure_hpa - g_previous_pressure_hpa;

                float elapsed_minutes =
                    (float)elapsed_ms / 60000.0f;

                float decay_rate_pa_per_min =
                    (delta_pressure * 100.0f) / elapsed_minutes;

                g_previous_pressure_hpa = pressure_hpa;
                g_last_decay_check_time = current_time;

                if (decay_rate_pa_per_min > 170.0f) {
                    g_vac_state = VAC_STATE_FAILED;
                    break;
                }
            }

            if (current_time - g_test_start_time >= 600000U) {
                g_vac_state = VAC_STATE_PASSED;
            }

            break;
        }
        case VAC_STATE_FAILED: {
            break; 
        }
        case VAC_STATE_PASSED: {
            break; 
        }
    }
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

        g_bme280_valid = (bme280_read_all(&g_bme280_dev) == ROV_OK) && isfinite(g_bme280_dev.pressure_hpa) &&
                         isfinite(g_bme280_dev.humidity_pct) && isfinite(g_bme280_dev.temperature_c);
        g_ina237_valid = (ina237_read_power(&g_ina237_dev) == ROV_OK) && isfinite(g_ina237_dev.bus_voltage_v) &&
                         isfinite(g_ina237_dev.shunt_current_a);

        if(g_bme280_valid){
            node1_update_vacuum_decay(g_bme280_dev.pressure_hpa, current_time); 
        }

        uint8_t leak_bits = 0;

        /*
         * Check BME280 humidity threshold (> 80%).
         *
         * Environmental leak detection uses bit 0.
         */
        if (g_bme280_valid && g_bme280_dev.humidity_pct >= ROV_LEAK_HUMIDITY_MAX_PCT) {
            leak_bits |= 0x01;
        }

        /*
         * Check vacuum decay.
         *
         * If the enclosure was pulled to vacuum and pressure rises by
         * more than the configured threshold, report an environmental
         * leak using bit 0.
         */
        if(g_vac_state == VAC_STATE_FAILED){
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
__attribute__((weak)) void app_main(void) {
    node1_app_init();

    while (1) {
        node1_app_step();
    }
}
#endif
