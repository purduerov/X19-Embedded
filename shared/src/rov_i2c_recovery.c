/**
 * @file rov_i2c_recovery.c
 * @brief Automated 9-Clock I2C Clear-Bus Recovery Handler.
 * @organization Purdue ROV
 */

#include "rov_i2c_recovery.h"
#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

rov_status_t rov_i2c_recover_bus(rov_gpio_write_fn scl_write, rov_gpio_read_fn sda_read, rov_gpio_write_fn sda_write,
                                 rov_delay_us_fn delay_us) {
    if (!scl_write || !sda_read || !sda_write || !delay_us) {
        return ROV_ERR_INVALID_ARG;
    }

    if (sda_read()) {
        return ROV_OK;
    }

    for (int i = 0; i < 9; i++) {
        scl_write(false);
        delay_us(5);
        scl_write(true);
        delay_us(5);

        if (sda_read()) {
            break;
        }
    }

    sda_write(false);
    delay_us(5);
    scl_write(true);
    delay_us(5);
    sda_write(true);
    delay_us(5);

    if (sda_read()) {
        return ROV_OK;
    }

    return ROV_ERR_BUS_LOCKED;
}

/* ========================================================================== */
/* Backward Compatibility Export Symbols                                      */
/* ========================================================================== */
#undef x19_i2c_recover_bus

rov_status_t x19_i2c_recover_bus(rov_gpio_write_fn scl_write, rov_gpio_read_fn sda_read, rov_gpio_write_fn sda_write,
                                 rov_delay_us_fn delay_us) {
    return rov_i2c_recover_bus(scl_write, sda_read, sda_write, delay_us);
}
