/**
 * @file test_driver_tmp1075.c
 * @brief Unit tests for TI TMP1075 Temperature Sensor Driver.
 * @organization Purdue ROV
 */

#include "mocks/mock_sensors.h"
#include "tmp1075.h"
#include <assert.h>
#include <stdio.h>

void test_tmp1075_driver(void) {
    tmp1075_dev_t dev;

    /* Negative test: NULL pointer */
    assert(tmp1075_init(NULL, 0x48) == ROV_ERR_INVALID_ARG);
    assert(tmp1075_read_temperature(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization */
    assert(tmp1075_init(&dev, 0x48) == ROV_OK);
    assert(dev.i2c_addr == 0x48);

    /* Test mock temperature injection: 42.5 deg C */
    mock_sensors_reset();
    mock_sensors_set_tmp1075(42.5f);

    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    assert(dev.temperature_c > 42.4f && dev.temperature_c < 42.6f);

    printf("[PASS] test_tmp1075_driver\n");
}

int main(void) {
    printf("Running TMP1075 Driver Unit Tests...\n");
    test_tmp1075_driver();
    printf("All TMP1075 Driver Tests Passed Successfully!\n");
    return 0;
}
