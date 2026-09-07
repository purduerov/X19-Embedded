/**
 * @file rov_can_protocol.c
 * @brief CAN FD serialization and deserialization implementations.
 * @organization Purdue ROV
 */

#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <string.h>

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
