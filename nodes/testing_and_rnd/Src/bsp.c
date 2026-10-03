/**
 * @file bsp.c
 * @brief Board Support Package Implementation for NUCLEO-F446RE.
 * Encapsulates STM32 HAL peripheral calls away from application logic.
 */

#include "bsp.h"
#include "can_interface.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately to UART */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize and start CAN hardware and filters */
    can_init();
}

uint32_t time_get_ms(void) {
    return HAL_GetTick();
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void led_set(bool state) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ===========================================================================
 * I2C Driver on NUCLEO Pins PB8 (D15 - SCL) and PB9 (D14 - SDA)
 * =========================================================================== */

void bsp_i2c_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Idle bus: SCL and SDA high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

static inline void i2c_delay(void) {
    for (volatile int i = 0; i < 40; i++) {
        __NOP();
    }
}

static inline void i2c_scl(bool high) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

static inline void i2c_sda(bool high) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    i2c_delay();
}

static inline bool i2c_read_sda(void) {
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET);
}

bool bsp_i2c_probe(uint8_t addr) {
    /* START condition */
    i2c_sda(true);
    i2c_scl(true);
    i2c_sda(false);
    i2c_scl(false);

    /* Send 7-bit address + Write bit (0) */
    uint8_t byte = (addr << 1);
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }

    /* Release SDA and read ACK (LOW = ACK) */
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);

    /* STOP condition */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);

    return ack;
}

uint8_t bsp_i2c_scan(void) {
    uint8_t count = 0;
    printf("\r\n========================================\r\n");
    printf("   I2C Bus Scanner (PB8:SCL, PB9:SDA)   \r\n");
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

bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len) {
    if (!data && len > 0)
        return false;

    /* START */
    i2c_sda(true);
    i2c_scl(true);
    i2c_sda(false);
    i2c_scl(false);

    /* Address + Write */
    uint8_t byte = (addr << 1);
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);
    if (!ack) {
        i2c_sda(false);
        i2c_scl(true);
        i2c_sda(true);
        return false;
    }

    for (uint16_t b = 0; b < len; b++) {
        byte = data[b];
        for (int i = 7; i >= 0; i--) {
            i2c_sda((byte >> i) & 1);
            i2c_scl(true);
            i2c_scl(false);
        }
        i2c_sda(true);
        i2c_scl(true);
        ack = !i2c_read_sda();
        i2c_scl(false);
        if (!ack) {
            i2c_sda(false);
            i2c_scl(true);
            i2c_sda(true);
            return false;
        }
    }

    /* STOP */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);
    return true;
}

bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len) {
    if (!data && len > 0)
        return false;

    /* START */
    i2c_sda(true);
    i2c_scl(true);
    i2c_sda(false);
    i2c_scl(false);

    /* Address + Read */
    uint8_t byte = (addr << 1) | 0x01;
    for (int i = 7; i >= 0; i--) {
        i2c_sda((byte >> i) & 1);
        i2c_scl(true);
        i2c_scl(false);
    }
    i2c_sda(true);
    i2c_scl(true);
    bool ack = !i2c_read_sda();
    i2c_scl(false);
    if (!ack) {
        i2c_sda(false);
        i2c_scl(true);
        i2c_sda(true);
        return false;
    }

    for (uint16_t b = 0; b < len; b++) {
        uint8_t received = 0;
        i2c_sda(true);
        for (int i = 7; i >= 0; i--) {
            i2c_scl(true);
            if (i2c_read_sda()) {
                received |= (1 << i);
            }
            i2c_scl(false);
        }
        data[b] = received;

        if (b + 1 < len) {
            i2c_sda(false); /* ACK */
        } else {
            i2c_sda(true); /* NACK on last */
        }
        i2c_scl(true);
        i2c_scl(false);
    }

    /* STOP */
    i2c_sda(false);
    i2c_scl(true);
    i2c_sda(true);
    return true;
}

rov_status_t bsp_i2c_mem_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U) {
        return ROV_ERR_INVALID_ARG;
    }

    /*
     * Tell the device which register we want to read.
     *
     * For example, the BME280 chip-ID register is 0xD0.
     */
    if (!bsp_i2c_write(addr, &reg, 1U)) {
        return ROV_ERROR;
    }

    /*
     * Now read the requested bytes starting from that register.
     */
    if (!bsp_i2c_read(addr, data, len)) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

rov_status_t bsp_i2c_mem_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    if (data == NULL || len == 0U || len > 32U) {
        return ROV_ERR_INVALID_ARG;
    }

    uint8_t tx_data[33];
    tx_data[0] = reg;
    memcpy(&tx_data[1], data, len);

    if (!bsp_i2c_write(addr, tx_data, (uint16_t)(len + 1U))) {
        return ROV_ERROR;
    }

    return ROV_OK;
}

uint64_t time_get_us(void) {
    return (uint64_t)HAL_GetTick() * 1000U;
}

static uint16_t g_bench_pwm[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
static uint16_t g_bench_solenoids = 0;
static bool g_bench_emergency_tripped = false;
static uint8_t g_bench_brick_mask = 0;

void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us) {
    if (channel < 8) {
        g_bench_pwm[channel] = pulse_us;
    }
}

uint16_t bsp_pwm_get_us(uint8_t channel) {
    return (channel < 8) ? g_bench_pwm[channel] : 1500U;
}

void bsp_solenoid_set(uint16_t mask) {
    g_bench_solenoids = mask;
}

uint16_t bsp_solenoid_get(void) {
    return g_bench_solenoids;
}

bool bsp_leak_probe_read(uint8_t probe_idx) {
    (void)probe_idx;
    return false;
}

void bsp_emergency_brake_trip(void) {
    g_bench_emergency_tripped = true;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

bool bsp_is_emergency_brake_tripped(void) {
    return g_bench_emergency_tripped;
}

void bsp_power_brick_enable(uint8_t brick_idx) {
    if (brick_idx < 4) {
        g_bench_brick_mask |= (1 << brick_idx);
    }
}

void bsp_power_brick_disable_all(void) {
    g_bench_brick_mask = 0;
}

uint32_t bsp_get_logic_voltage_mv(void) {
    return 5000U;
}

bool bsp_lm74700_status_ok(void) {
    return true;
}

float bsp_get_pcb_temperature_c(void) {
    return 24.5f;
}
