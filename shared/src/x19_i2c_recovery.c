/**
 * @file x19_i2c_recovery.c
 * @brief Automated 9-Clock I2C Clear-Bus Recovery Handler.
 * @organization Purdue ROV
 */

#include "x19_types.h"
#include <stdint.h>
#include <stdbool.h>

typedef void (*x19_gpio_write_fn)(bool high);
typedef bool (*x19_gpio_read_fn)(void);
typedef void (*x19_delay_us_fn)(uint32_t us);

x19_status_t x19_i2c_recover_bus(x19_gpio_write_fn scl_write,
                                x19_gpio_read_fn sda_read,
                                x19_gpio_write_fn sda_write,
                                x19_delay_us_fn delay_us) {
    if (!scl_write || !sda_read || !sda_write || !delay_us) {
        return X19_ERR_INVALID_ARG;
    }

    if (sda_read()) {
        return X19_OK;
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
        return X19_OK;
    }

    return X19_ERR_BUS_LOCKED;
}
