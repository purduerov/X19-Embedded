/**
 * @file test_driver_ina237.c
 * @brief Unit tests for TI INA237 High-Precision Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina237.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_ina237.h"
#include <assert.h>
#include <stdio.h>

#define INA237_TEST_ADDR 0x40u
#define INA237_TEST_SHUNT_OHMS 0.001f

void test_ina237_driver(void) {
    ina237_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers and a non-positive shunt. */
    assert(ina237_init(NULL, bus, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, NULL, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, -0.01f) == ROV_ERR_INVALID_ARG);
    assert(ina237_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization stores the transport, address, and shunt. */
    assert(ina237_init(&dev, bus, INA237_TEST_ADDR, INA237_TEST_SHUNT_OHMS) == ROV_OK);
    assert(dev.i2c_addr == INA237_TEST_ADDR);
    assert(dev.shunt_resistor_ohms > 0.0009f && dev.shunt_resistor_ohms < 0.0011f);

    /*
     * Datasheet round trip at the default ADCRANGE = 0:
     *   VBUS   = 5.2 V  -> 5.2 / 3.125 mV  = 1664 counts
     *   VSHUNT = 3.5 A * 1 mOhm = 3.5 mV -> 3.5 mV / 5 uV = 700 counts
     */
    mock_ina237_set_reading(INA237_TEST_ADDR, 5.2f, 3.5f, INA237_TEST_SHUNT_OHMS);
    mock_ina237_set_die_temperature(INA237_TEST_ADDR, 25.0f);

    assert(ina237_read_power(&dev) == ROV_OK);
    assert(dev.bus_voltage_v > 5.19f && dev.bus_voltage_v < 5.21f);
    assert(dev.shunt_current_a > 3.49f && dev.shunt_current_a < 3.51f);
    assert(dev.power_w > 18.19f && dev.power_w < 18.21f);
    assert(dev.die_temp_c > 24.9f && dev.die_temp_c < 25.1f);

    /* A transport fault propagates and leaves the previous reading intact. */
    float held_voltage = dev.bus_voltage_v;
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(ina237_read_power(&dev) == ROV_TIMEOUT);
    assert(dev.bus_voltage_v == held_voltage);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_ina237_driver\n");
}

int main(void) {
    printf("Running INA237 Driver Unit Tests...\n");
    test_ina237_driver();
    printf("All INA237 Driver Tests Passed Successfully!\n");
    return 0;
}
