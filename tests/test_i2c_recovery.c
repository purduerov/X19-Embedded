#include "rov_i2c_recovery.h"
#include "rov_types.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

static bool g_mock_sda = false;
static int g_clock_toggle_count = 0;
static bool g_simulate_bus_locked = false;

static void mock_scl_write(bool high) {
    if (high) {
        g_clock_toggle_count++;
        // Simulate slave releasing SDA on 3rd clock cycle
        if (!g_simulate_bus_locked && g_clock_toggle_count >= 3) {
            g_mock_sda = true;
        }
    }
}

static bool mock_sda_read(void) {
    if (g_simulate_bus_locked) {
        return false;
    }
    return g_mock_sda;
}

static void mock_sda_write(bool high) {
    g_mock_sda = high;
}

static void mock_delay_us(uint32_t us) {
    (void)us;
}

void test_i2c_bus_recovery_success(void) {
    g_mock_sda = false;  // Initially locked LOW by slave
    g_clock_toggle_count = 0;
    g_simulate_bus_locked = false;

    rov_status_t status = rov_i2c_recover_bus(mock_scl_write, mock_sda_read, mock_sda_write, mock_delay_us);
    assert(status == ROV_OK);
    assert(g_clock_toggle_count >= 3);
    printf("[PASS] test_i2c_bus_recovery_success\n");
}

void test_i2c_bus_recovery_locked(void) {
    g_mock_sda = false;  // Initially locked LOW by slave
    g_clock_toggle_count = 0;
    g_simulate_bus_locked = true;

    rov_status_t status = rov_i2c_recover_bus(mock_scl_write, mock_sda_read, mock_sda_write, mock_delay_us);
    assert(status == ROV_ERR_BUS_LOCKED);
    assert(g_clock_toggle_count == 10);  // 9 cycles in loop + 1 at the end
    printf("[PASS] test_i2c_bus_recovery_locked\n");

    g_simulate_bus_locked = false;  // Reset for other tests if any
}

int main(void) {
    printf("Running I2C Bus Recovery Unit Tests...\n");
    test_i2c_bus_recovery_success();
    test_i2c_bus_recovery_locked();
    printf("All I2C Recovery Tests Passed Successfully!\n");
    return 0;
}
