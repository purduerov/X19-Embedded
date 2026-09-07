/**
 * @file test_driver_ms5837.c
 * @brief Unit tests for MS5837-30BA Hydrostatic Depth Driver.
 * @organization Purdue ROV
 */

#include "mocks/mock_sensors.h"
#include "ms5837.h"
#include <assert.h>
#include <stdio.h>

void test_ms5837_driver(void) {
    ms5837_dev_t dev;

    /* Negative test: NULL pointer and invalid density */
    assert(ms5837_init(NULL) == ROV_ERR_INVALID_ARG);
    assert(ms5837_read_pressure_depth(NULL, 1000.0f) == ROV_ERR_INVALID_ARG);
    assert(ms5837_init(&dev) == ROV_OK);
    assert(ms5837_read_pressure_depth(&dev, 0.0f) == ROV_ERR_INVALID_ARG);
    assert(ms5837_read_pressure_depth(&dev, -10.0f) == ROV_ERR_INVALID_ARG);

    /* Test surface atmospheric pressure: depth must clamp to 0.0 meters */
    mock_sensors_reset();
    mock_sensors_set_ms5837(1013.25f, 18.0f);
    assert(ms5837_read_pressure_depth(&dev, 1000.0f) == ROV_OK);
    assert(dev.depth_meters == 0.0f);

    /* Test 10.0 meters in freshwater (rho = 1000 kg/m^3):
     * Delta_P = rho * g * h = 1000 * 9.80665 * 10 = 98066.5 Pa = 980.665 mbar
     * Total P = 1013.25 + 980.665 = 1993.915 mbar
     */
    mock_sensors_set_ms5837(1993.915f, 15.0f);
    assert(ms5837_read_pressure_depth(&dev, 1000.0f) == ROV_OK);
    assert(dev.depth_meters > 9.99f && dev.depth_meters < 10.01f);

    printf("[PASS] test_ms5837_driver\n");
}

int main(void) {
    printf("Running MS5837 Driver Unit Tests...\n");
    test_ms5837_driver();
    printf("All MS5837 Driver Tests Passed Successfully!\n");
    return 0;
}
