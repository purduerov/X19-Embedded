/**
 * @file test_driver_ina226.c
 * @brief Unit tests for TI INA226 Power Monitor Driver.
 * @organization Purdue ROV
 */

#include "ina226.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_ina226.h"
#include <assert.h>
#include <stdio.h>

#define INA226_TEST_ADDR       0x40u
#define INA226_TEST_SHUNT_OHMS 0.002f

void test_ina226_driver(void) {
    ina226_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers and a non-positive shunt. */
    assert(ina226_init(NULL, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, NULL, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, -0.05f) == ROV_ERR_INVALID_ARG);
    assert(ina226_read_power(NULL) == ROV_ERR_INVALID_ARG);

    /* Init probes the die ID; with an empty register file it must fail. */
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) != ROV_OK);

    /* Place the part, then init succeeds and stores the transport. */
    mock_ina226_present(INA226_TEST_ADDR);
    assert(ina226_init(&dev, bus, INA226_TEST_ADDR, INA226_TEST_SHUNT_OHMS) == ROV_OK);
    assert(dev.i2c_addr == INA226_TEST_ADDR);

    /*
     * Datasheet round trip:
     *   VBUS   = 5.2 V            -> 5.2 / 1.25 mV = 4160 counts
     *   VSHUNT = 2.5 A * 2 mOhm   = 5 mV -> 5 mV / 2.5 uV = 2000 counts
     */
    mock_ina226_set_reading(INA226_TEST_ADDR, 5.2f, 2.5f, INA226_TEST_SHUNT_OHMS);

    assert(ina226_read_power(&dev) == ROV_OK);
    assert(dev.voltage_v > 5.19f && dev.voltage_v < 5.21f);
    assert(dev.current_a > 2.49f && dev.current_a < 2.51f);
    assert(dev.power_w > 12.99f && dev.power_w < 13.01f);

    /* A transport fault propagates and leaves the previous reading intact. */
    float held_voltage = dev.voltage_v;
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(ina226_read_power(&dev) == ROV_TIMEOUT);
    assert(dev.voltage_v == held_voltage);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_ina226_driver\n");
}

int main(void) {
    printf("Running INA226 Driver Unit Tests...\n");
    test_ina226_driver();
    printf("All INA226 Driver Tests Passed Successfully!\n");
    return 0;
}
