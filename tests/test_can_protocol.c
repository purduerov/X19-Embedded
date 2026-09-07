#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

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
    printf("[PASS] test_thruster_cmd_pack_unpack\n");
}

void test_nav_telemetry_pack_unpack(void) {
    rov_nav_telemetry_t original = {.q_w = 0.7071f,
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

    rov_nav_telemetry_t decoded;
    status = rov_can_unpack_nav_telemetry(buffer, len, &decoded);
    assert(status == ROV_OK);
    assert(decoded.depth_meters == original.depth_meters);
    assert(decoded.imu_status == 3);
    printf("[PASS] test_nav_telemetry_pack_unpack\n");
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

    printf("[PASS] test_invalid_arguments\n");
}

void test_security_pwm_bounds(void) {
    rov_thruster_cmd_t original;
    for (int i = 0; i < 8; i++) {
        original.pwm_us[i] = 1500;  // Safe value
    }

    // Set one value out of bounds (too high)
    original.pwm_us[3] = 2001;

    uint8_t buffer[64];
    size_t len = 0;
    rov_status_t status = rov_can_pack_thruster_cmd(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);

    rov_thruster_cmd_t decoded;
    status = rov_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == ROV_ERR_INVALID_ARG);
    // Ensure it fails securely (cleared to zeros)
    for (int i = 0; i < 8; i++) {
        assert(decoded.pwm_us[i] == 0);
    }

    // Test out of bounds (too low)
    original.pwm_us[3] = 1500;
    original.pwm_us[5] = 999;
    status = rov_can_pack_thruster_cmd(&original, buffer, sizeof(buffer), &len);
    assert(status == ROV_OK);

    status = rov_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == ROV_ERR_INVALID_ARG);

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
    original.solenoid_mask = 0xFFFF;
    assert(rov_can_pack_solenoid_cmd(&original, buffer, sizeof(buffer), &len) == ROV_OK);
    assert(rov_can_unpack_solenoid_cmd(buffer, len, &decoded) == ROV_OK);
    assert(decoded.solenoid_mask == 0x03FF);

    printf("[PASS] test_solenoid_cmd_pack_unpack\n");
}

int main(void) {
    printf("Running CAN Protocol Unit Tests...\n");
    test_thruster_cmd_pack_unpack();
    test_solenoid_cmd_pack_unpack();
    test_nav_telemetry_pack_unpack();
    test_invalid_arguments();
    test_security_pwm_bounds();
    printf("All CAN Protocol Tests Passed Successfully!\n");
    return 0;
}
