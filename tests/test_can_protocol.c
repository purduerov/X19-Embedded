#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* In Release builds, assert() is stripped by -DNDEBUG.
 * Variables only read inside assert() become unused.
 * Use (void)var after each test block to silence -Wunused-but-set-variable. */

void test_thruster_cmd_pack_unpack(void) {
    rov_thruster_cmd_t original;
    for (int i = 0; i < 8; i++) {
        original.pwm_us[i] = 1000 + (i * 125);
    }

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_thruster_cmd(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);
    assert(len == sizeof(rov_thruster_cmd_t));

    rov_thruster_cmd_t decoded;
    status = rov_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == ROV_OK);

    for (int i = 0; i < 8; i++) {
        assert(decoded.pwm_us[i] == original.pwm_us[i]);
    }
    (void)status;
    (void)len;
    (void)decoded;
    printf("[PASS] test_thruster_cmd_pack_unpack\n");
}

void test_nav_telemetry_pack_unpack(void) {
    rov_nav_telemetry_t original = {.timestamp_us = 1726340000123456ULL,
                                    .q_w = 0.7071f,
                                    .q_x = 0.0f,
                                    .q_y = 0.7071f,
                                    .q_z = 0.0f,
                                    .gyro_x_rad_s = 0.05f,
                                    .gyro_y_rad_s = -0.12f,
                                    .gyro_z_rad_s = 0.01f,
                                    .depth_meters = 12.45f,
                                    .imu_status = 3};

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_nav_telemetry(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);
    assert(len == sizeof(rov_nav_telemetry_t));
    assert(len == 41);

    rov_nav_telemetry_t decoded;
    status = rov_can_unpack_nav_telemetry(buffer, len, &decoded);
    assert(status == ROV_OK);
    assert(decoded.timestamp_us == original.timestamp_us);
    assert(decoded.depth_meters == original.depth_meters);
    assert(decoded.imu_status == 3);
    (void)status;
    (void)len;
    (void)decoded;
    printf("[PASS] test_nav_telemetry_pack_unpack\n");
}

void test_time_sync_pack_unpack(void) {
    /* 1. Master Time Sync frame (0x010) */
    rov_time_sync_master_t master_orig = {
        .master_time_us = 1726345000000000ULL,
        .sync_seq = 42,
        .flags = 0x03,
        .reserved = {0, 0, 0}
    };
    uint8_t buffer[64];
    size_t len = 0;
    assert(rov_can_pack_time_sync_master(&master_orig, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(len == sizeof(rov_time_sync_master_t));
    assert(len == 16);

    rov_time_sync_master_t master_dec;
    assert(rov_can_unpack_time_sync_master(buffer, len, &master_dec) == ROV_OK);
    assert(master_dec.master_time_us == master_orig.master_time_us);
    assert(master_dec.sync_seq == 42);
    assert(master_dec.flags == 0x03);

    /* 2. Time Sync Delay Request frame (0x011) */
    rov_time_sync_req_t req_orig = {
        .target_node_id = ROV_NODE_CONTROL_BOARD,
        .seq = 7,
        .reserved = 0,
        .flags = 1,
        .t1_us = 1726345000000100ULL
    };
    assert(rov_can_pack_time_sync_req(&req_orig, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(len == sizeof(rov_time_sync_req_t));
    assert(len == 16);

    rov_time_sync_req_t req_dec;
    assert(rov_can_unpack_time_sync_req(buffer, len, &req_dec) == ROV_OK);
    assert(req_dec.target_node_id == ROV_NODE_CONTROL_BOARD);
    assert(req_dec.seq == 7);
    assert(req_dec.t1_us == req_orig.t1_us);

    /* 3. Time Sync Delay Response frame (0x012) */
    rov_time_sync_resp_t resp_orig = {
        .responder_node_id = ROV_NODE_CONTROL_BOARD,
        .seq = 7,
        .reserved = 0,
        .status = 1,
        .t1_us = 1726345000000100ULL,
        .t2_us = 1726345000000145ULL,
        .t3_us = 1726345000000155ULL
    };
    assert(rov_can_pack_time_sync_resp(&resp_orig, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(len == sizeof(rov_time_sync_resp_t));
    assert(len == 32);

    rov_time_sync_resp_t resp_dec;
    assert(rov_can_unpack_time_sync_resp(buffer, len, &resp_dec) == ROV_OK);
    assert(resp_dec.responder_node_id == ROV_NODE_CONTROL_BOARD);
    assert(resp_dec.seq == 7);
    assert(resp_dec.t1_us == resp_orig.t1_us);
    assert(resp_dec.t2_us == resp_orig.t2_us);
    assert(resp_dec.t3_us == resp_orig.t3_us);

    printf("[PASS] test_time_sync_pack_unpack\n");
}

void test_env_telemetry_pack_unpack(void) {
    rov_env_telemetry_t original = {
        .pressure_hpa = 1013.25f, .humidity_pct = 45.5f, .temperature_c = 22.1f, .leak_flags = 5};

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_env_telemetry(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);
    assert(len == sizeof(rov_env_telemetry_t));

    rov_env_telemetry_t decoded;
    status = rov_can_unpack_env_telemetry(buffer, len, &decoded);
    assert(status == ROV_OK);
    assert(decoded.pressure_hpa == original.pressure_hpa);
    assert(decoded.humidity_pct == original.humidity_pct);
    assert(decoded.temperature_c == original.temperature_c);
    assert(decoded.leak_flags == original.leak_flags);
    (void)status;
    (void)len;
    (void)decoded;
    printf("[PASS] test_env_telemetry_pack_unpack\n");
}

void test_power_telemetry_pack_unpack(void) {
    rov_power_telemetry_t original = {.tether_voltage_mv = 48000,
                                      .tether_current_ma = 1500,
                                      .v5_voltage_mv = 5200,
                                      .v5_current_ma = 500,
                                      .v12_current_ma = {100, 200, 300, 400},
                                      .pcb_temp_c = 450,
                                      .status_flags = 0x01};

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_power_telemetry(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);
    assert(len == sizeof(rov_power_telemetry_t));

    rov_power_telemetry_t decoded;
    status = rov_can_unpack_power_telemetry(buffer, len, &decoded);
    assert(status == ROV_OK);
    assert(decoded.tether_voltage_mv == original.tether_voltage_mv);
    assert(decoded.tether_current_ma == original.tether_current_ma);
    assert(decoded.v5_voltage_mv == original.v5_voltage_mv);
    assert(decoded.v5_current_ma == original.v5_current_ma);
    for (int i = 0; i < 4; i++) {
        assert(decoded.v12_current_ma[i] == original.v12_current_ma[i]);
    }
    assert(decoded.pcb_temp_c == original.pcb_temp_c);
    assert(decoded.status_flags == original.status_flags);
    (void)status;
    (void)len;
    (void)decoded;
    printf("[PASS] test_power_telemetry_pack_unpack\n");
}

void test_invalid_arguments(void) {
    uint8_t buffer[10];
    rov_thruster_cmd_t cmd;
    size_t len = 0;

    assert(rov_can_pack_thruster_cmd(NULL, buffer, sizeof(buffer), &len) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_thruster_cmd(buffer, 5, &cmd) == ROV_ERR_INVALID_ARG);

    /* Negative test cases: Buffer capacity too small */
    assert(rov_can_pack_thruster_cmd(&cmd, buffer, 5, &len) == ROV_ERR_INVALID_ARG);

    rov_nav_telemetry_t nav;
    assert(rov_can_pack_nav_telemetry(&nav, buffer, 5, &len) == ROV_ERR_INVALID_ARG);

    rov_env_telemetry_t env;
    assert(rov_can_pack_env_telemetry(&env, buffer, 5, &len) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_env_telemetry(buffer, 5, &env) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_env_telemetry(NULL, sizeof(buffer), &env) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_env_telemetry(buffer, sizeof(buffer), NULL) == ROV_ERR_INVALID_ARG);

    rov_power_telemetry_t power;
    assert(rov_can_pack_power_telemetry(NULL, buffer, sizeof(buffer), &len) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_power_telemetry(buffer, 5, &power) == ROV_ERR_INVALID_ARG);
    assert(rov_can_pack_power_telemetry(&power, buffer, 5, &len) == ROV_ERR_INVALID_ARG);
    assert(rov_can_unpack_power_telemetry(NULL, sizeof(buffer), &power) == ROV_ERR_INVALID_ARG);

    (void)buffer;
    (void)cmd;
    (void)len;
    (void)nav;
    (void)env;
    (void)power;
    printf("[PASS] test_invalid_arguments\n");
}

void test_security_pwm_bounds(void) {
    rov_thruster_cmd_t original;
    for (int i = 0; i < 8; i++) {
        original.pwm_us[i] = 1500;
    }

    original.pwm_us[3] = 2001;

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_thruster_cmd(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);

    rov_thruster_cmd_t decoded;
    status = rov_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == ROV_ERR_INVALID_ARG);
    for (int i = 0; i < 8; i++) {
        assert(decoded.pwm_us[i] == 0);
    }

    original.pwm_us[3] = 1500;
    original.pwm_us[5] = 999;
    status = rov_can_pack_thruster_cmd(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);

    status = rov_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == ROV_ERR_INVALID_ARG);

    (void)status;
    (void)len;
    (void)decoded;
    printf("[PASS] test_security_pwm_bounds\n");
}

void test_solenoid_cmd_pack_unpack(void) {
    rov_solenoid_cmd_t original = {.solenoid_mask = 0x0255};
    uint8_t buffer[64];
    size_t len = 0;
    assert(rov_can_pack_solenoid_cmd(&original, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(len == sizeof(rov_solenoid_cmd_t));

    rov_solenoid_cmd_t decoded;
    assert(rov_can_unpack_solenoid_cmd(buffer, len, &decoded) == ROV_OK);
    assert(decoded.solenoid_mask == 0x0255);

    /* Test 10-bit mask clamping */
    original.solenoid_mask = 0xFC01;
    assert(rov_can_pack_solenoid_cmd(&original, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(rov_can_unpack_solenoid_cmd(buffer, len, &decoded) == ROV_OK);
    assert(decoded.solenoid_mask == 0x0001);

    /* The two coils for a valve are mutually exclusive at the protocol boundary. */
    original.solenoid_mask = 0x0003;
    assert(rov_can_pack_solenoid_cmd(&original, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(rov_can_unpack_solenoid_cmd(buffer, len, &decoded) == ROV_ERR_INVALID_ARG);
    assert(decoded.solenoid_mask == 0);

    (void)len;
    (void)decoded;
    (void)original;
    printf("[PASS] test_solenoid_cmd_pack_unpack\n");
}

int main(void) {
    printf("Running CAN Protocol Unit Tests...\n");
    test_thruster_cmd_pack_unpack();
    test_solenoid_cmd_pack_unpack();
    test_nav_telemetry_pack_unpack();
    test_time_sync_pack_unpack();
    test_env_telemetry_pack_unpack();
    test_power_telemetry_pack_unpack();
    test_invalid_arguments();
    test_security_pwm_bounds();
    printf("All CAN Protocol Tests Passed Successfully!\n");
    return 0;
}
