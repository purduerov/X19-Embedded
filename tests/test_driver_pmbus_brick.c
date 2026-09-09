/**
 * @file test_driver_pmbus_brick.c
 * @brief Unit tests for Mornsun PMBus DC-DC Converter Brick Driver.
 * @organization Purdue ROV
 */

#include "pmbus_brick.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <stdio.h>

void test_pmbus_brick_driver(void) {
    pmbus_brick_dev_t dev;

    /* Negative test: NULL pointer */
    assert(pmbus_brick_init(NULL, 0x40) == ROV_ERR_INVALID_ARG);
    assert(pmbus_brick_read_telemetry(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization */
    assert(pmbus_brick_init(&dev, 0x41) == ROV_OK);
    assert(dev.pmbus_addr == 0x41);

    /* Test mock telemetry injection: 48.0V in, 12.05V out @ 8.5A, 41.5 deg C */
    mock_sensors_reset();
    mock_sensors_set_tps25990(1, 48.0f, 12.05f, 8.5f, 41.5f, 0x0000);

    assert(pmbus_brick_read_telemetry(&dev) == ROV_OK);
    assert(dev.input_voltage_v > 47.9f && dev.input_voltage_v < 48.1f);
    assert(dev.output_voltage_v > 12.04f && dev.output_voltage_v < 12.06f);
    assert(dev.output_current_a > 8.49f && dev.output_current_a < 8.51f);
    assert(dev.temperature_c > 41.4f && dev.temperature_c < 41.6f);

    printf("[PASS] test_pmbus_brick_driver\n");
}

int main(void) {
    printf("Running PMBus Brick Driver Unit Tests...\n");
    test_pmbus_brick_driver();
    printf("All PMBus Brick Driver Tests Passed Successfully!\n");
    return 0;
}
