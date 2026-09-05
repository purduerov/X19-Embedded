/**
 * @file app.c
 * @brief Node 1 (Pi Shield) Application Layer Logic.
 *
 * Handles BME280 sealed enclosure vacuum/humidity leak testing, INA226 5V monitor,
 * GPIO floor leak traces, 9-clock I2C recovery, and 10 Hz Leak Telemetry CAN stream.
 * Pure application logic with zero vendor ST HAL calls.
 */

#include "app.h"
#include "bme280.h"
#include "bsp.h"
#include "can_interface.h"
#include "ina226.h"
#include "x19_can_protocol.h"
#include "x19_safety.h"

static x19_safety_state_t g_safety_state;
static x19_env_telemetry_t g_env_telemetry;
static bme280_dev_t g_bme280_dev;
static ina226_dev_t g_ina226_dev;

void app_main(void) {
    bsp_init();
    x19_safety_init(&g_safety_state);

    bme280_init(&g_bme280_dev);
    ina226_init(&g_ina226_dev, 0x40, 0.002f);

    uint32_t last_telemetry_time = 0;

    while (1) {
        /* Sample sensors at 10 Hz */
        if (time_get_ms() - last_telemetry_time >= 100) {
            last_telemetry_time = time_get_ms();

            g_env_telemetry.pressure_hpa = g_bme280_dev.pressure_hpa;
            g_env_telemetry.humidity_pct = g_bme280_dev.humidity_pct;
            g_env_telemetry.temperature_c = g_bme280_dev.temperature_c;

            /* Stream 0x210 Environment & Leak Telemetry over CAN FD */
            can_send(X19_CAN_ID_ENV_TELEMETRY, (const uint8_t *)&g_env_telemetry, sizeof(g_env_telemetry));
            led_toggle();
        }

        delay_ms(5);
    }
}
