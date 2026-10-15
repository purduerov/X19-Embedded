/**
 * @file test_ms5837.c
 * @brief Standalone Hardware Diagnostic & Live Monitor for TE MS5837 (Bar30 / Bar02).
 *
 * Runs on NUCLEO-G474RE bench to diagnose I2C communication, PROM calibration,
 * CRC verification, and raw D1/D2 conversion timing.
 *
 * Target: python rov.py sandbox -f sandbox/test_ms5837.c -b g474
 */

#include "bsp.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MS5837_ADDR           0x76U
#define MS5837_RESET          0x1EU
#define MS5837_ADC_READ       0x00U
#define MS5837_PROM_READ_BASE 0xA0U
#define MS5837_CONVERT_D1     0x48U /* OSR 4096 pressure */
#define MS5837_CONVERT_D2     0x58U /* OSR 4096 temperature */

#define MS5837_02BA_30BA_SEPARATION 37000U
#define MS5837_ATMOSPHERIC_MBAR     1013.25f

static uint8_t calc_crc4(const uint16_t n_prom[8]) {
    uint16_t prom_copy[8];
    uint16_t n_rem = 0;

    for (int i = 0; i < 8; i++) {
        prom_copy[i] = n_prom[i];
    }
    prom_copy[0] &= 0x0FFF;
    prom_copy[7] = 0;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) {
            n_rem ^= (uint16_t)(prom_copy[cnt >> 1] & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(prom_copy[cnt >> 1] >> 8);
        }

        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem <<= 1;
            }
        }
    }
    return (uint8_t)((n_rem >> 12) & 0x0F);
}

void app_main(void) {
    bsp_init();
    bsp_i2c_init();

    printf("\r\n======================================================\r\n");
    printf("   X19 ROV - MS5837 Depth Sensor Hardware Diagnostic  \r\n");
    printf("======================================================\r\n");

    /* Step 1: Probe I2C Bus */
    printf("[1/5] Probing MS5837 at 0x%02X... ", MS5837_ADDR);
    if (!bsp_i2c_probe(MS5837_ADDR)) {
        printf("FAILED (No response / NACK)\r\n");
        printf("Troubleshooting:\r\n");
        printf("  - Verify wiring: PA15 -> SCL, PB7 -> SDA, 3.3V -> Vin, GND -> GND\r\n");
        printf("  - Check that pull-up resistors are active or present\r\n");
        while (1) {
            led_toggle();
            delay_ms(500);
        }
    }
    printf("ACK received!\r\n");

    /* Step 2: Reset Sensor */
    printf("[2/5] Sending RESET command (0x%02X)... ", MS5837_RESET);
    uint8_t cmd_reset = MS5837_RESET;
    if (!bsp_i2c_write(MS5837_ADDR, &cmd_reset, 1)) {
        printf("FAILED to send RESET!\r\n");
        while (1) {
            delay_ms(500);
        }
    }
    delay_ms(15); /* 15 ms wait after reset */
    printf("OK!\r\n");

    /* Step 3: Read PROM Calibration Coefficients */
    printf("[3/5] Reading PROM calibration words:\r\n");
    uint16_t prom[8] = {0};
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t cmd_prom = MS5837_PROM_READ_BASE + (i * 2);
        uint8_t data[2] = {0};
        if (!bsp_i2c_write(MS5837_ADDR, &cmd_prom, 1) || !bsp_i2c_read(MS5837_ADDR, data, 2)) {
            printf("  FAILED to read PROM word C[%u]!\r\n", i);
            while (1) {
                delay_ms(500);
            }
        }
        prom[i] = ((uint16_t)data[0] << 8) | data[1];
        printf("  -> C[%u] = 0x%04X (%5u)\r\n", i, prom[i], prom[i]);
    }

    /* Step 4: CRC4 Verification */
    uint8_t crc_expected = (uint8_t)(prom[0] >> 12);
    uint8_t crc_calc = calc_crc4(prom);
    printf("[4/5] CRC4 Check: Expected=0x%X, Calculated=0x%X -> %s\r\n", crc_expected, crc_calc,
           crc_expected == crc_calc ? "MATCH (VALID)" : "MISMATCH (CORRUPT)");

    /* Step 5: Identify Sensor Model */
    bool is_02ba = (prom[1] > MS5837_02BA_30BA_SEPARATION);
    printf("[5/5] Detected Model: %s (C[1] = %u)\r\n",
           is_02ba ? "MS5837-02BA (Bar02, 0-2 bar)" : "MS5837-30BA (Bar30, 0-30 bar)", prom[1]);

    printf("\r\n--- Entering Live Measurement Loop (2 Hz) ---\r\n\r\n");

    uint32_t sample_num = 0;
    while (1) {
        sample_num++;

        /* 1. Request D1 (Pressure) conversion */
        uint8_t cmd_d1 = MS5837_CONVERT_D1;
        if (!bsp_i2c_write(MS5837_ADDR, &cmd_d1, 1)) {
            printf("[%lu] ERR: Failed to write CONVERT_D1\r\n", (unsigned long)sample_num);
            delay_ms(500);
            continue;
        }
        delay_ms(20); /* Ensure ADC completes (OSR 4096 is max 9.04 ms) */

        /* 2. Read D1 ADC result (24 bits) */
        uint8_t cmd_adc = MS5837_ADC_READ;
        uint8_t d1_buf[3] = {0};
        if (!bsp_i2c_write(MS5837_ADDR, &cmd_adc, 1) || !bsp_i2c_read(MS5837_ADDR, d1_buf, 3)) {
            printf("[%lu] ERR: Failed to read D1 ADC bytes\r\n", (unsigned long)sample_num);
            delay_ms(500);
            continue;
        }
        uint32_t raw_d1 = ((uint32_t)d1_buf[0] << 16) | ((uint32_t)d1_buf[1] << 8) | d1_buf[2];

        /* 3. Request D2 (Temperature) conversion */
        uint8_t cmd_d2 = MS5837_CONVERT_D2;
        if (!bsp_i2c_write(MS5837_ADDR, &cmd_d2, 1)) {
            printf("[%lu] ERR: Failed to write CONVERT_D2\r\n", (unsigned long)sample_num);
            delay_ms(500);
            continue;
        }
        delay_ms(20); /* Ensure ADC completes */

        /* 4. Read D2 ADC result (24 bits) */
        uint8_t d2_buf[3] = {0};
        if (!bsp_i2c_write(MS5837_ADDR, &cmd_adc, 1) || !bsp_i2c_read(MS5837_ADDR, d2_buf, 3)) {
            printf("[%lu] ERR: Failed to read D2 ADC bytes\r\n", (unsigned long)sample_num);
            delay_ms(500);
            continue;
        }
        uint32_t raw_d2 = ((uint32_t)d2_buf[0] << 16) | ((uint32_t)d2_buf[1] << 8) | d2_buf[2];

        if (raw_d1 == 0 || raw_d2 == 0) {
            printf("[%lu] WARN: ADC returned 0 (raw_d1=0x%06lX, raw_d2=0x%06lX)\r\n", (unsigned long)sample_num,
                   (unsigned long)raw_d1, (unsigned long)raw_d2);
            delay_ms(500);
            continue;
        }

        /* 5. Compensation calculations */
        int32_t dT = (int32_t)raw_d2 - ((int32_t)prom[5] << 8);
        int32_t temp_c_x100 = 2000 + (int32_t)(((int64_t)dT * (int64_t)prom[6]) >> 23);

        int64_t off, sens;
        if (is_02ba) {
            off = ((int64_t)prom[2] << 17) + (((int64_t)prom[4] * (int64_t)dT) >> 6);
            sens = ((int64_t)prom[1] << 16) + (((int64_t)prom[3] * (int64_t)dT) >> 7);
        } else {
            off = ((int64_t)prom[2] << 16) + (((int64_t)prom[4] * (int64_t)dT) >> 7);
            sens = ((int64_t)prom[1] << 15) + (((int64_t)prom[3] * (int64_t)dT) >> 8);
        }

        /* Second-order temperature compensation */
        int64_t ti = 0, off2 = 0, sens2 = 0;
        int64_t dt_sq = (int64_t)dT * (int64_t)dT;

        if (is_02ba) {
            if (temp_c_x100 < 2000) {
                int64_t t_delta = (int64_t)temp_c_x100 - 2000;
                int64_t t_delta_sq = t_delta * t_delta;
                ti = (11LL * dt_sq) >> 35;
                off2 = (31LL * t_delta_sq) >> 3;
                sens2 = (63LL * t_delta_sq) >> 5;
            }
        } else {
            if (temp_c_x100 < 2000) {
                int64_t t_delta = (int64_t)temp_c_x100 - 2000;
                int64_t t_delta_sq = t_delta * t_delta;
                ti = (3LL * dt_sq) >> 33;
                off2 = (3LL * t_delta_sq) >> 1;
                sens2 = (5LL * t_delta_sq) >> 3;
                if (temp_c_x100 < -1500) {
                    int64_t vl_delta = (int64_t)temp_c_x100 + 1500;
                    int64_t vl_delta_sq = vl_delta * vl_delta;
                    off2 += 7LL * vl_delta_sq;
                    sens2 += 4LL * vl_delta_sq;
                }
            } else {
                ti = (2LL * dt_sq) >> 37;
                int64_t t_delta = (int64_t)temp_c_x100 - 2000;
                off2 = (t_delta * t_delta) >> 4;
                sens2 = 0;
            }
        }

        temp_c_x100 -= (int32_t)ti;
        off -= off2;
        sens -= sens2;

        float pressure_mbar;
        if (is_02ba) {
            int32_t p = (int32_t)(((((int64_t)raw_d1 * sens) >> 21) - off) >> 15);
            pressure_mbar = (float)p / 100.0f;
        } else {
            int32_t p = (int32_t)(((((int64_t)raw_d1 * sens) >> 21) - off) >> 13);
            pressure_mbar = (float)p / 10.0f;
        }

        float temperature_c = (float)temp_c_x100 / 100.0f;
        float depth_m = (pressure_mbar - MS5837_ATMOSPHERIC_MBAR) * 100.0f / (1000.0f * 9.80665f);
        if (depth_m < 0.0f) {
            depth_m = 0.0f;
        }

        printf("[%4lu] Temp: %5.2f C | Press: %7.2f mbar | Depth: %5.3f m | D1=0x%06lX D2=0x%06lX\r\n",
               (unsigned long)sample_num, temperature_c, pressure_mbar, depth_m, (unsigned long)raw_d1,
               (unsigned long)raw_d2);

        led_toggle();
        delay_ms(500);
    }
}
