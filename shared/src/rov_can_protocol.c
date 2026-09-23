/**
 * @file rov_can_protocol.c
 * @brief CAN FD serialization and deserialization implementations.
 * @organization Purdue ROV
 */

#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <string.h>

rov_status_t rov_can_pack_time_sync_master(const rov_time_sync_master_t *sync, uint8_t *buffer, size_t max_len,
                                           size_t *packed_len) {
    if (!sync || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_time_sync_master_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, sync, sizeof(rov_time_sync_master_t));
    *packed_len = sizeof(rov_time_sync_master_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_time_sync_master(const uint8_t *buffer, size_t len, rov_time_sync_master_t *sync) {
    if (!buffer || !sync)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_time_sync_master_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(sync, buffer, sizeof(rov_time_sync_master_t));
    return ROV_OK;
}

rov_status_t rov_can_pack_time_sync_req(const rov_time_sync_req_t *req, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    if (!req || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_time_sync_req_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, req, sizeof(rov_time_sync_req_t));
    *packed_len = sizeof(rov_time_sync_req_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_time_sync_req(const uint8_t *buffer, size_t len, rov_time_sync_req_t *req) {
    if (!buffer || !req)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_time_sync_req_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(req, buffer, sizeof(rov_time_sync_req_t));
    return ROV_OK;
}

rov_status_t rov_can_pack_time_sync_resp(const rov_time_sync_resp_t *resp, uint8_t *buffer, size_t max_len,
                                         size_t *packed_len) {
    if (!resp || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_time_sync_resp_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, resp, sizeof(rov_time_sync_resp_t));
    *packed_len = sizeof(rov_time_sync_resp_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_time_sync_resp(const uint8_t *buffer, size_t len, rov_time_sync_resp_t *resp) {
    if (!buffer || !resp)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_time_sync_resp_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(resp, buffer, sizeof(rov_time_sync_resp_t));
    return ROV_OK;
}

rov_status_t rov_can_pack_thruster_cmd(const rov_thruster_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len) {
    if (!cmd || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_thruster_cmd_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, cmd, sizeof(rov_thruster_cmd_t));
    *packed_len = sizeof(rov_thruster_cmd_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_thruster_cmd(const uint8_t *buffer, size_t len, rov_thruster_cmd_t *cmd) {
    if (!buffer || !cmd)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_thruster_cmd_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(cmd, buffer, sizeof(rov_thruster_cmd_t));

    /* Security enhancement: Validate PWM bounds (Sentinel) */
    for (int i = 0; i < 8; i++) {
        if (cmd->pwm_us[i] < ROV_PWM_MIN_US || cmd->pwm_us[i] > ROV_PWM_MAX_US) {
            memset(cmd, 0, sizeof(rov_thruster_cmd_t));
            return ROV_ERR_INVALID_ARG;
        }
    }

    return ROV_OK;
}

rov_status_t rov_can_pack_solenoid_cmd(const rov_solenoid_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len) {
    if (!cmd || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_solenoid_cmd_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, cmd, sizeof(rov_solenoid_cmd_t));
    *packed_len = sizeof(rov_solenoid_cmd_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_solenoid_cmd(const uint8_t *buffer, size_t len, rov_solenoid_cmd_t *cmd) {
    if (!buffer || !cmd)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_solenoid_cmd_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(cmd, buffer, sizeof(rov_solenoid_cmd_t));
    /* Mask to 10 valid channels (bits 0..9) */
    cmd->solenoid_mask &= 0x03FF;
    /* Each double-acting valve has opposing coils; never accept both at once. */
    for (uint8_t valve = 0; valve < ROV_NUM_SOLENOIDS; valve++) {
        uint16_t pair_mask = (uint16_t)(0x3u << (valve * 2u));
        if ((cmd->solenoid_mask & pair_mask) == pair_mask) {
            cmd->solenoid_mask = 0;
            return ROV_ERR_INVALID_ARG;
        }
    }
    return ROV_OK;
}

rov_status_t rov_can_pack_nav_telemetry(const rov_nav_telemetry_t *nav, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    if (!nav || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_nav_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, nav, sizeof(rov_nav_telemetry_t));
    *packed_len = sizeof(rov_nav_telemetry_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_nav_telemetry(const uint8_t *buffer, size_t len, rov_nav_telemetry_t *nav) {
    if (!buffer || !nav)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_nav_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(nav, buffer, sizeof(rov_nav_telemetry_t));
    return ROV_OK;
}

rov_status_t rov_can_pack_env_telemetry(const rov_env_telemetry_t *env, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    if (!env || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_env_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, env, sizeof(rov_env_telemetry_t));
    *packed_len = sizeof(rov_env_telemetry_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_env_telemetry(const uint8_t *buffer, size_t len, rov_env_telemetry_t *env) {
    if (!buffer || !env)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_env_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(env, buffer, sizeof(rov_env_telemetry_t));
    return ROV_OK;
}

rov_status_t rov_can_pack_power_telemetry(const rov_power_telemetry_t *power, uint8_t *buffer, size_t max_len,
                                          size_t *packed_len) {
    if (!power || !buffer || !packed_len)
        return ROV_ERR_INVALID_ARG;
    if (max_len < sizeof(rov_power_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(buffer, power, sizeof(rov_power_telemetry_t));
    *packed_len = sizeof(rov_power_telemetry_t);
    return ROV_OK;
}

rov_status_t rov_can_unpack_power_telemetry(const uint8_t *buffer, size_t len, rov_power_telemetry_t *power) {
    if (!buffer || !power)
        return ROV_ERR_INVALID_ARG;
    if (len < sizeof(rov_power_telemetry_t))
        return ROV_ERR_INVALID_ARG;
    memcpy(power, buffer, sizeof(rov_power_telemetry_t));
    return ROV_OK;
}

/* ========================================================================== */
/* Backward Compatibility Export Symbols                                      */
/* ========================================================================== */
#undef x19_can_pack_thruster_cmd
#undef x19_can_unpack_thruster_cmd
#undef x19_can_pack_solenoid_cmd
#undef x19_can_unpack_solenoid_cmd
#undef x19_can_pack_nav_telemetry
#undef x19_can_unpack_nav_telemetry
#undef x19_can_pack_env_telemetry
#undef x19_can_unpack_env_telemetry
#undef x19_can_pack_power_telemetry
#undef x19_can_unpack_power_telemetry

rov_status_t x19_can_pack_thruster_cmd(const rov_thruster_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len) {
    return rov_can_pack_thruster_cmd(cmd, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_thruster_cmd(const uint8_t *buffer, size_t len, rov_thruster_cmd_t *cmd) {
    return rov_can_unpack_thruster_cmd(buffer, len, cmd);
}

rov_status_t x19_can_pack_solenoid_cmd(const rov_solenoid_cmd_t *cmd, uint8_t *buffer, size_t max_len,
                                       size_t *packed_len) {
    return rov_can_pack_solenoid_cmd(cmd, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_solenoid_cmd(const uint8_t *buffer, size_t len, rov_solenoid_cmd_t *cmd) {
    return rov_can_unpack_solenoid_cmd(buffer, len, cmd);
}

rov_status_t x19_can_pack_nav_telemetry(const rov_nav_telemetry_t *nav, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    return rov_can_pack_nav_telemetry(nav, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_nav_telemetry(const uint8_t *buffer, size_t len, rov_nav_telemetry_t *nav) {
    return rov_can_unpack_nav_telemetry(buffer, len, nav);
}

rov_status_t x19_can_pack_env_telemetry(const rov_env_telemetry_t *env, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    return rov_can_pack_env_telemetry(env, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_env_telemetry(const uint8_t *buffer, size_t len, rov_env_telemetry_t *env) {
    return rov_can_unpack_env_telemetry(buffer, len, env);
}

rov_status_t x19_can_pack_power_telemetry(const rov_power_telemetry_t *power, uint8_t *buffer, size_t max_len,
                                          size_t *packed_len) {
    return rov_can_pack_power_telemetry(power, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_power_telemetry(const uint8_t *buffer, size_t len, rov_power_telemetry_t *power) {
    return rov_can_unpack_power_telemetry(buffer, len, power);
}

#undef x19_can_pack_time_sync_master
#undef x19_can_unpack_time_sync_master
#undef x19_can_pack_time_sync_req
#undef x19_can_unpack_time_sync_req
#undef x19_can_pack_time_sync_resp
#undef x19_can_unpack_time_sync_resp

rov_status_t x19_can_pack_time_sync_master(const rov_time_sync_master_t *sync, uint8_t *buffer, size_t max_len,
                                           size_t *packed_len) {
    return rov_can_pack_time_sync_master(sync, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_time_sync_master(const uint8_t *buffer, size_t len, rov_time_sync_master_t *sync) {
    return rov_can_unpack_time_sync_master(buffer, len, sync);
}

rov_status_t x19_can_pack_time_sync_req(const rov_time_sync_req_t *req, uint8_t *buffer, size_t max_len,
                                        size_t *packed_len) {
    return rov_can_pack_time_sync_req(req, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_time_sync_req(const uint8_t *buffer, size_t len, rov_time_sync_req_t *req) {
    return rov_can_unpack_time_sync_req(buffer, len, req);
}

rov_status_t x19_can_pack_time_sync_resp(const rov_time_sync_resp_t *resp, uint8_t *buffer, size_t max_len,
                                         size_t *packed_len) {
    return rov_can_pack_time_sync_resp(resp, buffer, max_len, packed_len);
}

rov_status_t x19_can_unpack_time_sync_resp(const uint8_t *buffer, size_t len, rov_time_sync_resp_t *resp) {
    return rov_can_unpack_time_sync_resp(buffer, len, resp);
}
