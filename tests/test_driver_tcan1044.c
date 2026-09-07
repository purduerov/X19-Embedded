/**
 * @file test_driver_tcan1044.c
 * @brief Unit tests for TI TCAN1044 CAN FD Transceiver Driver.
 * @organization Purdue ROV
 */

#include "tcan1044.h"
#include <assert.h>
#include <stdio.h>

void test_tcan1044_driver(void) {
    tcan1044_dev_t dev;

    /* Negative tests: NULL pointer guards */
    assert(tcan1044_init(NULL) == ROV_ERR_INVALID_ARG);
    assert(tcan1044_set_standby(NULL, true) == ROV_ERR_INVALID_ARG);

    /* Initialization */
    assert(tcan1044_init(&dev) == ROV_OK);
    assert(!dev.standby_mode);

    /* Set Standby Mode */
    assert(tcan1044_set_standby(&dev, true) == ROV_OK);
    assert(dev.standby_mode == true);

    /* Wake up */
    assert(tcan1044_set_standby(&dev, false) == ROV_OK);
    assert(dev.standby_mode == false);

    printf("[PASS] test_tcan1044_driver\n");
}

int main(void) {
    printf("Running TCAN1044 Driver Unit Tests...\n");
    test_tcan1044_driver();
    printf("All TCAN1044 Driver Tests Passed Successfully!\n");
    return 0;
}
