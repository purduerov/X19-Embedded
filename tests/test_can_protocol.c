#include "x19_can_protocol.h"
#include "x19_parameters.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>

void test_thruster_cmd_pack_unpack(void) {
    x19_thruster_cmd_t original;
    for (int i = 0; i < 8; i++) {
        original.pwm_us[i] = 1000 + (i * 125);
    }

    uint8_t buffer[64];
    size_t len = 0;
    x19_status_t status = x19_can_pack_thruster_cmd(&original, buffer, &len);
    assert(status == X19_OK);
    assert(len == sizeof(x19_thruster_cmd_t));

    x19_thruster_cmd_t decoded;
    status = x19_can_unpack_thruster_cmd(buffer, len, &decoded);
    assert(status == X19_OK);

    for (int i = 0; i < 8; i++) {
        assert(decoded.pwm_us[i] == original.pwm_us[i]);
    }
    printf("[PASS] test_thruster_cmd_pack_unpack\n");
}

void test_nav_telemetry_pack_unpack(void) {
    x19_nav_telemetry_t original = {
        .q_w = 0.7071f,
        .q_x = 0.0f,
        .q_y = 0.7071f,
        .q_z = 0.0f,
        .gyro_x_rad_s = 0.05f,
        .gyro_y_rad_s = -0.12f,
        .gyro_z_rad_s = 0.01f,
        .depth_meters = 12.45f,
        .imu_status = 3
    };

    uint8_t buffer[64];
    size_t len = 0;
    x19_status_t status = x19_can_pack_nav_telemetry(&original, buffer, &len);
    assert(status == X19_OK);
    assert(len == sizeof(x19_nav_telemetry_t));

    x19_nav_telemetry_t decoded;
    status = x19_can_unpack_nav_telemetry(buffer, len, &decoded);
    assert(status == X19_OK);
    assert(decoded.depth_meters == original.depth_meters);
    assert(decoded.imu_status == 3);
    printf("[PASS] test_nav_telemetry_pack_unpack\n");
}

void test_invalid_arguments(void) {
    uint8_t buffer[10];
    x19_thruster_cmd_t cmd;
    size_t len = 0;

    assert(x19_can_pack_thruster_cmd(NULL, buffer, &len) == X19_ERR_INVALID_ARG);
    assert(x19_can_unpack_thruster_cmd(buffer, 5, &cmd) == X19_ERR_INVALID_ARG);
    printf("[PASS] test_invalid_arguments\n");
}

int main(void) {
    printf("Running CAN Protocol Unit Tests...\n");
    test_thruster_cmd_pack_unpack();
    test_nav_telemetry_pack_unpack();
    test_invalid_arguments();
    printf("All CAN Protocol Tests Passed Successfully!\n");
    return 0;
}
