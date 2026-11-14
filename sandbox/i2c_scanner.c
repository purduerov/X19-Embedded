/**
 * @file i2c_scanner.c
 * @brief Generic platform-agnostic I2C bus scanner diagnostic.
 *
 * Scans standard 7-bit addresses (0x08 to 0x77) using bsp_i2c_probe() and prints
 * an interactive hexadecimal matrix and decoded sensor descriptions over UART.
 * Runs seamlessly across all boards (NUCLEO-G474, NUCLEO-F411, STM32C5, and Host SIL).
 */

#include "bsp.h"
#include "ina237.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void app_main(void) {
    bsp_init();
    bsp_i2c_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - Hardware Diagnostic I2C Bus Scanner      \r\n");
    printf("======================================================\r\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\r\n");

    uint8_t count = 0;
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

    printf("------------------------------------------------------\r\n");
    if (count == 0) {
        printf("Scan complete: 0 devices found.\r\n");
        printf("Note: If I2C devices are connected, verify:\r\n");
        printf("  1. Pull-up resistors (2.2k - 4.7k to 3.3V) on SCL and SDA.\r\n");
        printf("  2. Pinout: PB8 (SCL), PB9 (SDA) on Nucleo Arduino header (D15/D14).\r\n");
        printf("  3. Sensor VCC (3.3V) and GND connections.\r\n");
    } else {
        printf("Scan complete: found %u device(s).\r\n", count);

        for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
            if (bsp_i2c_probe(addr)) {
                const char *desc = "Unknown device";
                if (addr == 0x76) {
                    desc = "MS5837 Depth Sensor / Bosch BME280 (SDO=GND)";
                } else if (addr == 0x77) {
                    desc = "Murata DBE DC-DC Brick (SA0/SA1 open default 119d) / BME280 / MS5837";
                } else if (addr >= 0x40 && addr <= 0x47) {
                    desc = "TI INA237 / INA226 Power Monitor / PMBus Device";
                } else if (addr >= 0x48 && addr <= 0x4F) {
                    desc = "TI TMP1075 / BNO086 IMU";
                } else if (addr == 0x68 || addr == 0x69) {
                    desc = "BMI270 / LSM6DSOX IMU";
                } else if (addr <= 0x3F) {
                    desc = "PMBus / I2C Device (e.g. Murata DBE configured address)";
                }
                printf("  -> [0x%02X] %s\r\n", addr, desc);
            }
        }
    }
    printf("======================================================\r\n\r\n");

#if !defined(ROV_UNIT_TEST) && !defined(X19_UNIT_TEST)
    ina237_dev_t ina237;
    rov_status_t ina_status = ina237_init(&ina237, bsp_i2c_bus(), 0x40u, 0.001f);

    if (ina_status != ROV_OK) {
        printf("INA237 initialization FAILED (status=%d)\r\n", (int)ina_status);
        while (1) {
            led_toggle();
            delay_ms(1000);
        }
    }

    printf("INA237 initialized at 0x40. Reading rail telemetry...\r\n\r\n");

    while (1) {
        ina_status = ina237_read_power(&ina237);
        if (ina_status == ROV_OK) {
            printf("INA237 | VBUS: %.3f V | Current: %.3f A | Power: %.3f W | Die temp: %.2f C\r\n",
                   (double)ina237.bus_voltage_v, (double)ina237.shunt_current_a, (double)ina237.power_w,
                   (double)ina237.die_temp_c);
        } else {
            printf("INA237 read FAILED (status=%d)\r\n", (int)ina_status);
        }
        led_toggle();
        delay_ms(1000);
    }
#endif
}
