/**
 * @file test_driver_bme280.c
 * @brief Unit tests for Bosch BME280 Driver.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include "mocks/mock_sensors.h"
#include <assert.h>
#include <stdio.h>

void test_bme280_driver(void) {
    bme280_dev_t dev;

    /* Negative test: NULL pointer guards */
    assert(bme280_init(NULL) == X19_ERR_INVALID_ARG);
    assert(bme280_read_all(NULL) == X19_ERR_INVALID_ARG);

    /* Initialization */
    assert(bme280_init(&dev) == X19_OK);
    assert(dev.pressure_hpa == 0.0f);

    /* Test mock sensor reading */
    mock_sensors_reset();
    mock_sensors_set_bme280(995.5f, 55.2f, 21.8f);

    assert(bme280_read_all(&dev) == X19_OK);
    assert(dev.pressure_hpa > 995.0f && dev.pressure_hpa < 996.0f);
    assert(dev.humidity_pct > 55.0f && dev.humidity_pct < 56.0f);
    assert(dev.temperature_c > 21.0f && dev.temperature_c < 22.0f);

    printf("[PASS] test_bme280_driver\n");
}

int main(void) {
    printf("Running BME280 Driver Unit Tests...\n");
    test_bme280_driver();
    printf("All BME280 Driver Tests Passed Successfully!\n");
    return 0;
}
