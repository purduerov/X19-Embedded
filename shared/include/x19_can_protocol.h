/**
 * @file x19_can_protocol.h
 * @brief CAN FD Message Arbitration IDs, Frame Serialization, and Deserialization.
 * @organization Purdue ROV
 */

#ifndef X19_CAN_PROTOCOL_H
#define X19_CAN_PROTOCOL_H

#include "x19_types.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* CAN ARBITRATION ID ALLOCATION (11-bit Standard IDs)                        */
/* ========================================================================== */
#define X19_CAN_ID_EMERGENCY_BREAK          (0x001) /**< Priority 0: Leak/E-Stop Cutoff */
#define X19_CAN_ID_EFUSE_FAULT_ALERT        (0x005) /**< Priority 0: Power Slab Fault */
#define X19_CAN_ID_THRUSTER_CMD             (0x100) /**< Priority 1: 8x Thruster PWM */
#define X19_CAN_ID_SOLENOID_CMD             (0x110) /**< Priority 1: Pneumatic Solenoids */
#define X19_CAN_ID_NAV_TELEMETRY            (0x200) /**< Priority 2: 100 Hz Nav (IMU+Depth) */
#define X19_CAN_ID_ENV_TELEMETRY            (0x210) /**< Priority 2: 10 Hz Leak & Temp */
#define X19_CAN_ID_POWER_TELEMETRY          (0x300) /**< Priority 3: 20 Hz Power Slab */
#define X19_CAN_ID_USB_HUB_TELEMETRY        (0x310) /**< Priority 3: 5 Hz USB Hub */
#define X19_CAN_ID_BOOTLOADER_CMD           (0x700) /**< Priority 7: Bootloader Control */
#define X19_CAN_ID_BOOTLOADER_DATA          (0x701) /**< Priority 7: Bootloader Data Chunk */

/* ========================================================================== */
/* SERIALIZATION & DESERIALIZATION API                                       */
/* ========================================================================== */

x19_status_t x19_can_pack_thruster_cmd(const x19_thruster_cmd_t *cmd, uint8_t *buffer, size_t *len);
x19_status_t x19_can_unpack_thruster_cmd(const uint8_t *buffer, size_t len, x19_thruster_cmd_t *cmd);

x19_status_t x19_can_pack_nav_telemetry(const x19_nav_telemetry_t *nav, uint8_t *buffer, size_t *len);
x19_status_t x19_can_unpack_nav_telemetry(const uint8_t *buffer, size_t len, x19_nav_telemetry_t *nav);

x19_status_t x19_can_pack_env_telemetry(const x19_env_telemetry_t *env, uint8_t *buffer, size_t *len);
x19_status_t x19_can_unpack_env_telemetry(const uint8_t *buffer, size_t len, x19_env_telemetry_t *env);

x19_status_t x19_can_pack_power_telemetry(const x19_power_telemetry_t *power, uint8_t *buffer, size_t *len);
x19_status_t x19_can_unpack_power_telemetry(const uint8_t *buffer, size_t len, x19_power_telemetry_t *power);

#ifdef __cplusplus
}
#endif

#endif /* X19_CAN_PROTOCOL_H */
