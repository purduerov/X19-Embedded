/**
 * @file safety_service.h
 * @brief High-level safety, leak detection, and emergency trip service.
 * @author Purdue ROV Embedded Team
 *
 * Provides domain-level safety operations for application code.
 * Encapsulates probe reading, hardware break latching, and emergency cutoff.
 */

#ifndef X19_SAFETY_SERVICE_H
#define X19_SAFETY_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the safety monitoring service.
 */
void safety_service_init(void);

/**
 * @brief Check if a physical leak probe detects water contact.
 * @param probe_idx Leak probe index (0 or 1).
 * @return true if water contact detected (wet), false if dry.
 */
bool leak_probe_is_wet(uint8_t probe_idx);

/**
 * @brief Immediately trip the vehicle emergency brake and latch shutdown.
 */
void safety_emergency_trip(void);

/**
 * @brief Check if the safety emergency brake is currently tripped / latched.
 * @return true if emergency brake is active, false if nominal.
 */
bool safety_is_tripped(void);

/**
 * @brief Reset the safety emergency trip latch if all interlocks are clear.
 * @return true if cleared, false if fault condition persists.
 */
bool safety_emergency_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_SAFETY_SERVICE_H */
