/**
 * @file test_driver_tps25990.c
 * @brief Unit tests for TI TPS25990 PMBus Driver.
 * @organization Purdue ROV
 */

#include "mocks/mock_sensors.h"
#include "tps25990.h"
#include <assert.h>
#include <stdio.h>

void test_tps25990_driver(void) {
    tps25990_dev_t dev;

    /* Negative tests */
    assert(tps25990_init(NULL, 0x40) == X19_ERR_INVALID_ARG);
    assert(tps25990_read_telemetry(NULL) == X19_ERR_INVALID_ARG);

    /* Initialization */
    assert(tps25990_init(&dev, 0x41) == X19_OK);
    assert(dev.pmbus_addr == 0x41);

    /* Test mock telemetry read */
    mock_sensors_reset();
    mock_sensors_set_tps25990(1, 48.0f, 12.05f, 8.4f, 42.5f, 0x0000);

    assert(tps25990_read_telemetry(&dev) == X19_OK);
    assert(dev.output_voltage_v > 12.0f && dev.output_voltage_v < 12.1f);
    assert(dev.output_current_a > 8.3f && dev.output_current_a < 8.5f);
    assert(dev.temperature_c > 42.0f && dev.temperature_c < 43.0f);
    assert(dev.status_word == 0x0000);

    printf("[PASS] test_tps25990_driver\n");
}

int main(void) {
    printf("Running TPS25990 Driver Unit Tests...\n");
    test_tps25990_driver();
    printf("All TPS25990 Driver Tests Passed Successfully!\n");
    return 0;
}
