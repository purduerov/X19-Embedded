#include "x19_types.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

typedef void (*x19_gpio_write_fn)(bool high);
typedef bool (*x19_gpio_read_fn)(void);
typedef void (*x19_delay_us_fn)(uint32_t us);

extern x19_status_t x19_i2c_recover_bus(x19_gpio_write_fn scl_write, x19_gpio_read_fn sda_read,
                                        x19_gpio_write_fn sda_write, x19_delay_us_fn delay_us);

static bool g_mock_sda = false;
static int g_clock_toggle_count = 0;

static void mock_scl_write(bool high) {
    if (high) {
        g_clock_toggle_count++;
        // Simulate slave releasing SDA on 3rd clock cycle
        if (g_clock_toggle_count >= 3) {
            g_mock_sda = true;
        }
    }
}

static bool mock_sda_read(void) {
    return g_mock_sda;
}

static void mock_sda_write(bool high) {
    g_mock_sda = high;
}

static void mock_delay_us(uint32_t us) {
    (void)us;
}

static void mock_scl_write_locked(bool high) {
    if (high) {
        g_clock_toggle_count++;
    }
}

static void mock_sda_write_locked(bool high) {
    (void)high;
    g_mock_sda = false;  // SDA is held low by external device
}

void test_i2c_bus_recovery_success(void) {
    g_mock_sda = false;  // Initially locked LOW by slave
    g_clock_toggle_count = 0;

    x19_status_t status = x19_i2c_recover_bus(mock_scl_write, mock_sda_read, mock_sda_write, mock_delay_us);
    assert(status == X19_OK);
    assert(g_clock_toggle_count >= 3);
    printf("[PASS] test_i2c_bus_recovery_success\n");
}

void test_i2c_bus_recovery_locked(void) {
    g_mock_sda = false;  // Initially locked LOW by slave and never releases
    g_clock_toggle_count = 0;

    x19_status_t status =
        x19_i2c_recover_bus(mock_scl_write_locked, mock_sda_read, mock_sda_write_locked, mock_delay_us);
    assert(status == X19_ERR_BUS_LOCKED);
    assert(g_clock_toggle_count == 10);
    printf("[PASS] test_i2c_bus_recovery_locked\n");
}

int main(void) {
    printf("Running I2C Bus Recovery Unit Tests...\n");
    test_i2c_bus_recovery_success();
    test_i2c_bus_recovery_locked();
    printf("All I2C Recovery Tests Passed Successfully!\n");
    return 0;
}
