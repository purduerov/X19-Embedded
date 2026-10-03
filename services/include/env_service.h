/**
 * @file env_service.h
 * @brief High-level environmental sensor service (BME280 / ambient telemetry).
 * @author Purdue ROV Embedded Team
 *
 * Provides domain-level environmental telemetry reading for application code.
 * Encapsulates sensor initialization, bus transfers, and compensation math.
 */

#ifndef X19_ENV_SERVICE_H
#define X19_ENV_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the environmental sensor service.
 * @return ROV_OK on success, error code otherwise.
 */
rov_status_t env_service_init(void);

/**
 * @brief Read all environmental sensor values in engineering units.
 * @param temp_c Pointer to store temperature in degrees Celsius (may be NULL).
 * @param press_hpa Pointer to store barometric pressure in hPa (may be NULL).
 * @param hum_pct Pointer to store relative humidity in %RH (may be NULL).
 * @return ROV_OK if readings are valid, error code otherwise.
 */
rov_status_t env_read_all(float *temp_c, float *press_hpa, float *hum_pct);

/**
 * @brief Get latest cached ambient temperature.
 * @return Temperature in degrees Celsius.
 */
float env_get_temperature(void);

/**
 * @brief Get latest cached enclosure barometric pressure.
 * @return Pressure in hPa.
 */
float env_get_pressure(void);

/**
 * @brief Get latest cached enclosure relative humidity.
 * @return Relative humidity in %RH.
 */
float env_get_humidity(void);

/**
 * @brief Probe if an I2C device acknowledges at the given address.
 * @param addr 7-bit I2C device address.
 * @return true if device responded, false otherwise.
 */
bool env_probe_i2c(uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* X19_ENV_SERVICE_H */
