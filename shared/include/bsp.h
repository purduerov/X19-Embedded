/**
 * @file bsp.h
 * @brief Board Support Package (BSP) Interface for X19 Subsea Nodes.
 * @author Purdue ROV Embedded Team
 *
 * Provides platform-independent timing, delay, and board-level hardware
 * utilities. Application code (app.c) calls these functions rather than
 * vendor ST HAL functions.
 */

#ifndef X19_BSP_H
#define X19_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize low-level board hardware, clocks, peripherals, and debug UART.
 */
void bsp_init(void);

/**
 * @brief Get elapsed system time in milliseconds since boot.
 * @return Milliseconds elapsed.
 */
uint32_t time_get_ms(void);

/**
 * @brief Get elapsed system time in microseconds since boot.
 * @return Microseconds elapsed.
 */
uint64_t time_get_us(void);

/**
 * @brief Non-preemptive millisecond delay.
 * @param ms Duration to delay in milliseconds.
 */
void delay_ms(uint32_t ms);

/**
 * @brief Write bytes to a 7-bit I2C device address.
 * @param addr 7-bit I2C address.
 * @param data Data buffer to transmit.
 * @param len Number of bytes to transmit.
 * @return true on success, false on failure.
 */
bool bsp_i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief Read bytes from a 7-bit I2C device address.
 * @param addr 7-bit I2C address.
 * @param data Destination buffer.
 * @param len Number of bytes to read.
 * @return true on success, false on failure.
 */
bool bsp_i2c_read(uint8_t addr, uint8_t *data, uint16_t len);

/**
 * @brief Probe whether an I2C device with the specified 7-bit address responds with ACK.
 * @param addr 7-bit I2C address (0x08 to 0x77).
 * @return true if device acknowledges, false otherwise.
 */
bool bsp_i2c_probe(uint8_t addr);

/**
 * @brief Initialize low-level I2C hardware bus and pull-up GPIOs.
 */
void bsp_i2c_init(void);

/**
 * @brief Scan the I2C bus (0x08 to 0x77) and log detected devices over UART.
 * @return Number of detected devices.
 */
uint8_t bsp_i2c_scan(void);

/**
 * @brief Toggle the board heartbeat / diagnostic indicator LED.
 */
void led_toggle(void);

/**
 * @brief Set the board indicator LED state.
 * @param state true = ON, false = OFF.
 */
void led_set(bool state);

/**
 * @brief Output pulse width to an ESC channel (0 to 7).
 * @param channel ESC index (0 to 7).
 * @param pulse_us Pulse width in microseconds (1000 to 2000 us).
 */
void bsp_pwm_set_us(uint8_t channel, uint16_t pulse_us);

/**
 * @brief Get the last commanded pulse width on an ESC channel.
 * @param channel ESC index (0 to 7).
 * @return Pulse width in microseconds.
 */
uint16_t bsp_pwm_get_us(uint8_t channel);

/**
 * @brief Set pneumatic solenoid driver states.
 * @param mask 10-bit bitmask of energized solenoids.
 */
void bsp_solenoid_set(uint16_t mask);

/**
 * @brief Get active pneumatic solenoid driver bitmask.
 * @return 10-bit bitmask.
 */
uint16_t bsp_solenoid_get(void);

/**
 * @brief Read floor leak probe sensor input.
 * @param probe_idx Probe index (0 or 1).
 * @return true if water contact detected (wet), false if dry.
 */
bool bsp_leak_probe_read(uint8_t probe_idx);

/**
 * @brief Hardware Emergency Brake trigger (TIMx_BDTR BKIN cutoff / latch).
 */
void bsp_emergency_brake_trip(void);

/**
 * @brief Check if hardware emergency brake is tripped.
 * @return true if tripped, false if normal.
 */
bool bsp_is_emergency_brake_tripped(void);

/**
 * @brief Enable a specific DC-DC converter brick on Node 3 (0 to 3).
 * @param brick_idx Converter brick index (0 to 3).
 */
void bsp_power_brick_enable(uint8_t brick_idx);

/**
 * @brief Disable all 4 DC-DC converter bricks immediately.
 */
void bsp_power_brick_disable_all(void);

/**
 * @brief Get 5.2V logic rail voltage in millivolts (measured via INA237).
 * @return Voltage in millivolts.
 */
uint32_t bsp_get_logic_voltage_mv(void);

/**
 * @brief Query TI LM74700-Q1 ideal diode status.
 * @return true if diode status indicates normal operation (no reverse current / fault), false otherwise.
 */
bool bsp_lm74700_status_ok(void);

/**
 * @brief Read one 16-bit PMBus command word from a converter brick.
 *
 * A PMBus slave is write-addressed rather than register-addressed: the master
 * sends the command code and the slave answers with a word. This is therefore a
 * different shape from bsp_i2c_read(), which resumes from whatever pointer the
 * last write left behind.
 *
 * @param pmbus_addr 7-bit PMBus slave address.
 * @param command PMBus command code.
 * @param raw_word Destination for the 16-bit response.
 * @return true if the transaction succeeded.
 */
bool bsp_pmbus_read_word(uint8_t pmbus_addr, uint8_t command, uint16_t *raw_word);

/**
 * @brief Get PCB temperature in degrees Celsius from onboard sensor (TMP1075).
 * @return Temperature in degrees C.
 */
float bsp_get_pcb_temperature_c(void);

/**
 * @brief Read registers from an I2C peripheral.
 *
 * @param addr 7-bit I2C device address.
 * @param reg Starting register address.
 * @param data Destination buffer.
 * @param len Number of bytes to read.
 * @return ROV_OK on success or an error status.
 *
 */
rov_status_t bsp_i2c_mem_read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Write registers to an I2C peripheral.
 *
 * @param addr 7-bit I2C device address.
 * @param reg Starting register address.
 * @param data Source buffer.
 * @param len Number of bytes to write.
 * @return ROV_OK on success or an error status.
 *
 */
rov_status_t bsp_i2c_mem_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* X19_BSP_H */
