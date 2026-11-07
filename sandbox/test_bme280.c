/**
 * @file test_bme280.c
 * @brief Standalone Hardware Diagnostic & Live Monitor for Bosch BME280.
 *
 * Runs on NUCLEO-F411RE or NUCLEO-G474RE bench to diagnose I2C communication,
 * calibration trimming readouts, and live temperature, pressure, and humidity
 * compensation.
 *
 * Usage:
 *   python rov.py run sandbox/test_bme280.c -b f411
 *   python rov.py run sandbox/test_bme280.c -b g474
 */

#include "bme280.h"
#include "bsp.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void app_main(void) {
    bsp_init();
    bsp_i2c_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Bosch BME280 Live Hardware Diagnostic    \r\n");
    printf("======================================================\r\n");

    bme280_dev_t dev;
    rov_status_t status = bme280_init(&dev, bsp_i2c_bus());

    if (status != ROV_OK) {
        printf("[ERROR] Failed to initialize BME280 (status: %d).\r\n", status);
        printf("Check I2C wiring (SCL/SDA, 3.3V, GND, address 0x76 or 0x77).\r\n");
        while (1) {
            led_toggle();
            delay_ms(250);
        }
    }

    printf("[OK] BME280 detected and initialized successfully at address 0x%02X!\r\n", dev.i2c_addr);
    printf("Calibration Summary:\r\n");
    printf("  T1=%u, T2=%d, T3=%d\r\n", dev.calib.dig_T1, dev.calib.dig_T2, dev.calib.dig_T3);
    printf("  H1=%u, H2=%d, H3=%u, H4=%d, H5=%d, H6=%d\r\n", dev.calib.dig_H1, dev.calib.dig_H2, dev.calib.dig_H3,
           dev.calib.dig_H4, dev.calib.dig_H5, dev.calib.dig_H6);
    printf("------------------------------------------------------\r\n");
    printf("  Sample |  Temp (C)  | Pressure (hPa) | Humidity (%%RH) \r\n");
    printf("------------------------------------------------------\r\n");

    uint32_t sample = 0;
    while (1) {
        status = bme280_read_all(&dev);
        if (status == ROV_OK) {
            printf("  [%5lu] |  %7.2f C |   %8.2f hPa  |   %6.2f %%RH\r\n", (unsigned long)sample++,
                   (double)dev.temperature_c, (double)dev.pressure_hpa, (double)dev.humidity_pct);
        } else {
            printf("  [%5lu] | [I2C READ ERROR status=%d]\r\n", (unsigned long)sample++, status);
        }

        led_toggle();
        delay_ms(500);
    }
}
