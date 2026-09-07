/**
 * @file rov_i2c_recovery.h
 * @brief Automated 9-Clock I2C Clear-Bus Recovery Handler.
 * @organization Purdue ROV
 */

#ifndef ROV_I2C_RECOVERY_H
#define ROV_I2C_RECOVERY_H

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*rov_gpio_write_fn)(bool high);
typedef bool (*rov_gpio_read_fn)(void);
typedef void (*rov_delay_us_fn)(uint32_t us);

rov_status_t rov_i2c_recover_bus(rov_gpio_write_fn scl_write, rov_gpio_read_fn sda_read, rov_gpio_write_fn sda_write,
                                 rov_delay_us_fn delay_us);

/* ========================================================================== */
/* BACKWARD COMPATIBILITY ALIASES (X19 Vehicle Profile)                       */
/* ========================================================================== */
typedef rov_gpio_write_fn x19_gpio_write_fn;
typedef rov_gpio_read_fn x19_gpio_read_fn;
typedef rov_delay_us_fn x19_delay_us_fn;
#define x19_i2c_recover_bus rov_i2c_recover_bus

#ifdef __cplusplus
}
#endif

#endif /* ROV_I2C_RECOVERY_H */
