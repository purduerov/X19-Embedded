/**
 * @file diagnostic_i2c_scan.c
 * @brief Standalone I2C bus scanner diagnostic entrypoint.
 * Allows bench testing and hardware address discovery on dev boards without modifying app.c.
 */

#include "app.h"
#include "bsp.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#if defined(HAL_I2C_MODULE_ENABLED)
extern I2C_HandleTypeDef hi2c1;

static bool i2c_probe_device(uint8_t addr) {
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1U), 2, 5) == HAL_OK;
}
#else
static bool i2c_probe_device(uint8_t addr) {
    (void)addr;
    return false;
}
#endif

void app_main(void) {
    bsp_init();

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
            } else if (i2c_probe_device(addr)) {
                printf("%02X ", addr);
                count++;
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    printf("------------------------------------------------------\r\n");
    printf("Scan complete: found %u device(s).\r\n", count);

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_probe_device(addr)) {
            const char *desc = "Unknown device";
            if (addr == 0x76) {
                desc = "MS5837 Depth Sensor / Bosch BME280";
            } else if (addr == 0x77) {
                desc = "Bosch BME280 Enclosure Sensor";
            } else if (addr >= 0x40 && addr <= 0x47) {
                desc = "TI INA237 / INA226 Power Monitor";
            } else if (addr >= 0x48 && addr <= 0x4F) {
                desc = "TI TMP1075 / BNO086 IMU";
            }
            printf("  -> 0x%02X: %s\r\n", addr, desc);
        }
    }
    printf("======================================================\r\n\r\n");

    while (1) {
        led_toggle();
        delay_ms(1000);
    }
}
