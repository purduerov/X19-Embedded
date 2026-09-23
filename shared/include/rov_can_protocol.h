/**
 * @file rov_can_protocol.h
 * @brief CAN FD Message Arbitration IDs, Frame Serialization, and Deserialization.
 * @organization Purdue ROV
 */

#ifndef ROV_CAN_PROTOCOL_H
#define ROV_CAN_PROTOCOL_H

#include "rov_types.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* CAN ARBITRATION ID ALLOCATION (11-bit Standard IDs)                        */
/* ========================================================================== */
#define ROV_CAN_ID_EMERGENCY_BREAK   (0x001) /**< Priority 0: Leak/E-Stop Cutoff */
#define ROV_CAN_ID_EFUSE_FAULT_ALERT (0x005) /**< Priority 0: Power Slab Fault */
#define ROV_CAN_ID_TIME_SYNC_MASTER  (0x010) /**< Priority 0: 10 Hz Master Time Broadcast */
#define ROV_CAN_ID_TIME_SYNC_REQ     (0x011) /**< Priority 0: Delay / Latency Request */
#define ROV_CAN_ID_TIME_SYNC_RESP    (0x012) /**< Priority 0: Delay / Latency Response */
#define ROV_CAN_ID_THRUSTER_CMD      (0x100) /**< Priority 1: 8x Thruster PWM */
#define ROV_CAN_ID_SOLENOID_CMD      (0x110) /**< Priority 1: Pneumatic Solenoids */
#define ROV_CAN_ID_NAV_TELEMETRY     (0x200) /**< Priority 2: 100 Hz Nav (IMU+Depth) */
#define ROV_CAN_ID_ENV_TELEMETRY     (0x210) /**< Priority 2: 10 Hz Leak & Temp */
#define ROV_CAN_ID_POWER_TELEMETRY   (0x300) /**< Priority 3: 20 Hz Power Slab */
#define ROV_CAN_ID_USB_HUB_TELEMETRY (0x310) /**< Priority 3: 5 Hz USB Hub */
#define ROV_CAN_ID_BOOTLOADER_CMD    (0x700) /**< Priority 7: Bootloader Control */
#define ROV_CAN_ID_BOOTLOADER_DATA   (0x701) /**< Priority 7: Bootloader Data Chunk */

/* ========================================================================== */
/* SERIALIZATION & DESERIALIZATION API                                       */
/* ========================================================================== */

rov_status_t rov_can_pack_time_sync_master(const rov_time_sync_master_t *sync, uint8_t *buffer, size_t max_len,
                                           size_t *packed_len);
rov_status_t rov_can_unpack_time_sync_master(const uint8_t *buffer, size_t len, rov_time_sync_master_t *sync);

rov_status_t rov_can_pack_time_sync_req(const rov_time_sync_req_t *req, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len);
rov_status_t rov_can_unpack_time_sync_req(const uint8_t *buffer, size_t len, rov_time_sync_req_t *req);

rov_status_t rov_can_pack_time_sync_resp(const rov_time_sync_resp_t *resp, uint8_t *buffer, size_t max_len,
                                         size_t *packed_len);
rov_status_t rov_can_unpack_time_sync_resp(const uint8_t *buffer, size_t len, rov_time_sync_resp_t *resp);

rov_status_t rov_can_pack_thruster_cmd(const rov_thruster_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len);
rov_status_t rov_can_unpack_thruster_cmd(const uint8_t *buffer, size_t len, rov_thruster_cmd_t *cmd);

rov_status_t rov_can_pack_solenoid_cmd(const rov_solenoid_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len);
rov_status_t rov_can_unpack_solenoid_cmd(const uint8_t *buffer, size_t len, rov_solenoid_cmd_t *cmd);

rov_status_t rov_can_pack_nav_telemetry(const rov_nav_telemetry_t *nav, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len);
rov_status_t rov_can_unpack_nav_telemetry(const uint8_t *buffer, size_t len, rov_nav_telemetry_t *nav);

rov_status_t rov_can_pack_env_telemetry(const rov_env_telemetry_t *env, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len);
rov_status_t rov_can_unpack_env_telemetry(const uint8_t *buffer, size_t len, rov_env_telemetry_t *env);

rov_status_t rov_can_pack_power_telemetry(const rov_power_telemetry_t *power, uint8_t *buffer, size_t max_len,
                                          size_t *packed_len);
rov_status_t rov_can_unpack_power_telemetry(const uint8_t *buffer, size_t len, rov_power_telemetry_t *power);

/* ========================================================================== */
/* BACKWARD COMPATIBILITY ALIASES (X19 Vehicle Profile)                       */
/* ========================================================================== */
#define X19_CAN_ID_EMERGENCY_BREAK   ROV_CAN_ID_EMERGENCY_BREAK
#define X19_CAN_ID_EFUSE_FAULT_ALERT ROV_CAN_ID_EFUSE_FAULT_ALERT
#define X19_CAN_ID_TIME_SYNC_MASTER  ROV_CAN_ID_TIME_SYNC_MASTER
#define X19_CAN_ID_TIME_SYNC_REQ     ROV_CAN_ID_TIME_SYNC_REQ
#define X19_CAN_ID_TIME_SYNC_RESP    ROV_CAN_ID_TIME_SYNC_RESP
#define X19_CAN_ID_THRUSTER_CMD      ROV_CAN_ID_THRUSTER_CMD
#define X19_CAN_ID_SOLENOID_CMD      ROV_CAN_ID_SOLENOID_CMD
#define X19_CAN_ID_NAV_TELEMETRY     ROV_CAN_ID_NAV_TELEMETRY
#define X19_CAN_ID_ENV_TELEMETRY     ROV_CAN_ID_ENV_TELEMETRY
#define X19_CAN_ID_POWER_TELEMETRY   ROV_CAN_ID_POWER_TELEMETRY
#define X19_CAN_ID_USB_HUB_TELEMETRY ROV_CAN_ID_USB_HUB_TELEMETRY
#define X19_CAN_ID_BOOTLOADER_CMD    ROV_CAN_ID_BOOTLOADER_CMD
#define X19_CAN_ID_BOOTLOADER_DATA   ROV_CAN_ID_BOOTLOADER_DATA

#define x19_can_pack_time_sync_master   rov_can_pack_time_sync_master
#define x19_can_unpack_time_sync_master rov_can_unpack_time_sync_master
#define x19_can_pack_time_sync_req      rov_can_pack_time_sync_req
#define x19_can_unpack_time_sync_req    rov_can_unpack_time_sync_req
#define x19_can_pack_time_sync_resp     rov_can_pack_time_sync_resp
#define x19_can_unpack_time_sync_resp   rov_can_unpack_time_sync_resp
#define x19_can_pack_thruster_cmd       rov_can_pack_thruster_cmd
#define x19_can_unpack_thruster_cmd     rov_can_unpack_thruster_cmd
#define x19_can_pack_solenoid_cmd       rov_can_pack_solenoid_cmd
#define x19_can_unpack_solenoid_cmd     rov_can_unpack_solenoid_cmd
#define x19_can_pack_nav_telemetry      rov_can_pack_nav_telemetry
#define x19_can_unpack_nav_telemetry    rov_can_unpack_nav_telemetry
#define x19_can_pack_env_telemetry      rov_can_pack_env_telemetry
#define x19_can_unpack_env_telemetry    rov_can_unpack_env_telemetry
#define x19_can_pack_power_telemetry    rov_can_pack_power_telemetry
#define x19_can_unpack_power_telemetry  rov_can_unpack_power_telemetry

#ifdef __cplusplus
}
#endif

#endif /* ROV_CAN_PROTOCOL_H */
