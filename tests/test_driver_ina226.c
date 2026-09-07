/**
 * @file test_driver_ina226.c
 * @brief Unit tests for TI INA226 Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina226.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <stdio.h>

void test_ina226_driver(void) {
    ina226_dev_t dev;

    /* Negative test: NULL pointer and invalid shunt resistance */
    assert(ina226_init(NULL, 0x40, 0.002f) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, 0x40, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, 0x40, -0.05f) == ROV_ERR_INVALID_ARG);
    assert(ina226_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization */
    assert(ina226_init(&dev, 0x40, 0.002f) == ROV_OK);
    assert(dev.i2c_addr == 0x40);

    /* Test mock power calculation: 5.2V @ 2.5A = 13.0W */
    mock_sensors_reset();
    mock_sensors_set_ina226(5.2f, 2.5f);

    assert(ina226_read_power(&dev) == ROV_OK);
    assert(dev.voltage_v > 5.19f && dev.voltage_v < 5.21f);
    assert(dev.current_a > 2.49f && dev.current_a < 2.51f);
    assert(dev.power_w > 12.99f && dev.power_w < 13.01f);

    printf("[PASS] test_ina226_driver\n");
}

int main(void) {
    printf("Running INA226 Driver Unit Tests...\n");
    test_ina226_driver();
    printf("All INA226 Driver Tests Passed Successfully!\n");
    return 0;
}
