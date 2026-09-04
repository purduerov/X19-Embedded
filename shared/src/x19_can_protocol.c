/**
 * @file x19_can_protocol.c
 * @brief CAN FD serialization and deserialization implementations.
 * @organization Purdue ROV
 */

#include "x19_can_protocol.h"
#include <string.h>

x19_status_t x19_can_pack_thruster_cmd(const x19_thruster_cmd_t *cmd, uint8_t *buffer, size_t *len) {
    if (!cmd || !buffer || !len)
        return X19_ERR_INVALID_ARG;
    if (*len < sizeof(x19_thruster_cmd_t))
        return X19_ERR_INVALID_ARG;
    memcpy(buffer, cmd, sizeof(x19_thruster_cmd_t));
    *len = sizeof(x19_thruster_cmd_t);
    return X19_OK;
}

x19_status_t x19_can_unpack_thruster_cmd(const uint8_t *buffer, size_t len, x19_thruster_cmd_t *cmd) {
    if (!buffer || !cmd)
        return X19_ERR_INVALID_ARG;
    if (len < sizeof(x19_thruster_cmd_t))
        return X19_ERR_INVALID_ARG;
    memcpy(cmd, buffer, sizeof(x19_thruster_cmd_t));
    return X19_OK;
}

x19_status_t x19_can_pack_nav_telemetry(const x19_nav_telemetry_t *nav, uint8_t *buffer, size_t *len) {
    if (!nav || !buffer || !len)
        return X19_ERR_INVALID_ARG;
    if (*len < sizeof(x19_nav_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(buffer, nav, sizeof(x19_nav_telemetry_t));
    *len = sizeof(x19_nav_telemetry_t);
    return X19_OK;
}

x19_status_t x19_can_unpack_nav_telemetry(const uint8_t *buffer, size_t len, x19_nav_telemetry_t *nav) {
    if (!buffer || !nav)
        return X19_ERR_INVALID_ARG;
    if (len < sizeof(x19_nav_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(nav, buffer, sizeof(x19_nav_telemetry_t));
    return X19_OK;
}

x19_status_t x19_can_pack_env_telemetry(const x19_env_telemetry_t *env, uint8_t *buffer, size_t *len) {
    if (!env || !buffer || !len)
        return X19_ERR_INVALID_ARG;
    if (*len < sizeof(x19_env_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(buffer, env, sizeof(x19_env_telemetry_t));
    *len = sizeof(x19_env_telemetry_t);
    return X19_OK;
}

x19_status_t x19_can_unpack_env_telemetry(const uint8_t *buffer, size_t len, x19_env_telemetry_t *env) {
    if (!buffer || !env)
        return X19_ERR_INVALID_ARG;
    if (len < sizeof(x19_env_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(env, buffer, sizeof(x19_env_telemetry_t));
    return X19_OK;
}

x19_status_t x19_can_pack_power_telemetry(const x19_power_telemetry_t *power, uint8_t *buffer, size_t *len) {
    if (!power || !buffer || !len)
        return X19_ERR_INVALID_ARG;
    if (*len < sizeof(x19_power_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(buffer, power, sizeof(x19_power_telemetry_t));
    *len = sizeof(x19_power_telemetry_t);
    return X19_OK;
}

x19_status_t x19_can_unpack_power_telemetry(const uint8_t *buffer, size_t len, x19_power_telemetry_t *power) {
    if (!buffer || !power)
        return X19_ERR_INVALID_ARG;
    if (len < sizeof(x19_power_telemetry_t))
        return X19_ERR_INVALID_ARG;
    memcpy(power, buffer, sizeof(x19_power_telemetry_t));
    return X19_OK;
}
