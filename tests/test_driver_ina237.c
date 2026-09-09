/**
 * @file test_driver_ina237.c
 * @brief Unit tests for TI INA237 High-Precision Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina237.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <stdio.h>

void test_ina237_driver(void) {
    ina237_dev_t dev;

    /* Negative test: NULL pointer and invalid shunt resistance */
    assert(ina237_init(NULL, 0x40, 0.001f) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, 0x40, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, 0x40, -0.01f) == ROV_ERR_INVALID_ARG);
    assert(ina237_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization */
    assert(ina237_init(&dev, 0x40, 0.001f) == ROV_OK);
    assert(dev.i2c_addr == 0x40);

    /* Test mock power calculation: 5.2V @ 3.5A = 18.2W */
    mock_sensors_reset();
    mock_sensors_set_ina226(5.2f, 3.5f);

    assert(ina237_read_power(&dev) == ROV_OK);
    assert(dev.bus_voltage_v > 5.19f && dev.bus_voltage_v < 5.21f);
    assert(dev.shunt_current_a > 3.49f && dev.shunt_current_a < 3.51f);
    assert(dev.power_w > 18.19f && dev.power_w < 18.21f);

    printf("[PASS] test_ina237_driver\n");
}

int main(void) {
    printf("Running INA237 Driver Unit Tests...\n");
    test_ina237_driver();
    printf("All INA237 Driver Tests Passed Successfully!\n");
    return 0;
}
