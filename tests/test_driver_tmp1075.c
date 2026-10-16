/**
 * @file test_driver_tmp1075.c
 * @brief Unit tests for TI TMP1075 Temperature Sensor Driver.
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_tmp1075.h"
#include "tmp1075.h"
#include <assert.h>
#include <stdio.h>

void test_tmp1075_driver(void) {
    tmp1075_dev_t dev;
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();

    /* Negative test: NULL pointers. */
    assert(tmp1075_init(NULL, bus, 0x48) == ROV_ERR_INVALID_ARG);
    assert(tmp1075_init(&dev, NULL, 0x48) == ROV_ERR_INVALID_ARG);
    assert(tmp1075_read_temperature(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization stores the transport and address. */
    assert(tmp1075_init(&dev, bus, 0x48) == ROV_OK);
    assert(dev.i2c_addr == 0x48);

    /* Positive temperature round-trips through the datasheet conversion. */
    mock_tmp1075_set_temperature(0x48, 42.5f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    assert(dev.temperature_c > 42.4f && dev.temperature_c < 42.6f);

    /*
     * Negative temperature: the 12-bit two's complement field must sign-extend.
     * 12-bit two's complement of -10.0 C is 0xD80; left-justified that is
     * 0xD800, which is a negative int16_t. Dropping the pad bits after the
     * sign-extension gives -10 C, not +245 C.
     */
    mock_tmp1075_set_temperature(0x48, -10.0f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    assert(dev.temperature_c > -10.1f && dev.temperature_c < -9.9f);

    /* A transport fault propagates and leaves the previous reading intact. */
    mock_tmp1075_set_temperature(0x48, 25.0f);
    assert(tmp1075_read_temperature(&dev) == ROV_OK);
    float held = dev.temperature_c;

    mock_bsp_i2c_set_fault(ROV_TIMEOUT);
    assert(tmp1075_read_temperature(&dev) == ROV_TIMEOUT);
    assert(dev.temperature_c == held);
    mock_bsp_i2c_set_fault(ROV_OK);

    printf("[PASS] test_tmp1075_driver\n");
}

int main(void) {
    printf("Running TMP1075 Driver Unit Tests...\n");
    test_tmp1075_driver();
    printf("All TMP1075 Driver Tests Passed Successfully!\n");
    return 0;
}
