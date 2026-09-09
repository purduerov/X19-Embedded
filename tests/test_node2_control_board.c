/**
 * @file test_node2_control_board.c
 * @brief Host-Native SIL Test for Node 2 (Control Board) Application Logic.
 * @organization Purdue ROV
 */

#include "app.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void test_node2_boot_state(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    node2_app_init();

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }
    assert(mock_bsp_get_solenoid_mask() == 0);
    assert(!mock_bsp_is_emergency_brake_tripped());

    printf("[PASS] test_node2_boot_state\n");
}

void test_node2_thruster_ramping(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    node2_app_init();

    /* Command Thruster 0 to 1800 us */
    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = (i == 0) ? 1800 : ROV_PWM_STOP_US;
    }

    uint8_t buffer[64];
    size_t packed_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, buffer, sizeof(buffer), &packed_len) == ROV_OK);
    mock_can_inject_rx(ROV_CAN_ID_THRUSTER_CMD, buffer, (uint8_t)packed_len);

    /* Step 1: Receives command, time delta = 10 ms => ramps by 20 us to 1520 us */
    mock_bsp_advance_time_ms(10);
    node2_app_step();
    assert(mock_bsp_get_pwm_us(0) == 1520);

    /* Step across 140 ms more with continuous heartbeat (14 x 10 ms steps) */
    for (int step = 0; step < 14; step++) {
        mock_bsp_advance_time_ms(10);
        mock_can_inject_rx(ROV_CAN_ID_THRUSTER_CMD, buffer, (uint8_t)packed_len);
        node2_app_step();
    }

    /* Total ramp: 1500 + 150 * 2 = 1800 us target reached */
    assert(mock_bsp_get_pwm_us(0) == 1800);

    printf("[PASS] test_node2_thruster_ramping\n");
}

void test_node2_heartbeat_timeout_failsafe(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    node2_app_init();

    /* Ramp thruster 0 up to 1600 us */
    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = (i == 0) ? 1600 : ROV_PWM_STOP_US;
    }
    uint8_t buffer[64];
    size_t packed_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, buffer, sizeof(buffer), &packed_len) == ROV_OK);

    for (int step = 0; step < 5; step++) {
        mock_bsp_advance_time_ms(10);
        mock_can_inject_rx(ROV_CAN_ID_THRUSTER_CMD, buffer, (uint8_t)packed_len);
        node2_app_step();
    }
    assert(mock_bsp_get_pwm_us(0) == 1600);

    /* Cease transmitting thruster commands. Advance past 100 ms timeout */
    mock_bsp_advance_time_ms(110);
    node2_app_step();

    /* Target drops to 1500 us and begins ramping down */
    assert(mock_bsp_get_pwm_us(0) < 1600);

    /* After another 50 ms, reaches neutral 1500 us */
    mock_bsp_advance_time_ms(50);
    node2_app_step();
    assert(mock_bsp_get_pwm_us(0) == ROV_PWM_STOP_US);

    printf("[PASS] test_node2_heartbeat_timeout_failsafe\n");
}

void test_node2_emergency_break_cutoff(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    node2_app_init();

    /* Command all thrusters to 1700 us */
    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1700;
    }
    uint8_t buffer[64];
    size_t packed_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, buffer, sizeof(buffer), &packed_len) == ROV_OK);

    for (int step = 0; step < 10; step++) {
        mock_bsp_advance_time_ms(10);
        mock_can_inject_rx(ROV_CAN_ID_THRUSTER_CMD, buffer, (uint8_t)packed_len);
        node2_app_step();
    }
    assert(mock_bsp_get_pwm_us(0) == 1700);

    /* Inject spoofed/invalid Emergency Break (wrong magic bytes) */
    uint8_t spoofed_alert[8] = {0xDE, 0xAD, 0x01, 0, 0, 0, 0, 0};
    mock_can_inject_rx(ROV_CAN_ID_EMERGENCY_BREAK, spoofed_alert, sizeof(spoofed_alert));
    mock_bsp_advance_time_ms(1);
    node2_app_step();

    /* Thrusters should remain active, emergency break not tripped */
    assert(mock_bsp_get_pwm_us(0) == 1700);
    assert(!mock_bsp_is_emergency_brake_tripped());

    /* Inject Valid Priority 0 Emergency Break (0x001) */
    uint8_t alert[8] = {0xAA, 0x55, 0x01, 0, 0, 0, 0, 0};
    mock_can_inject_rx(ROV_CAN_ID_EMERGENCY_BREAK, alert, sizeof(alert));

    mock_bsp_advance_time_ms(1);
    node2_app_step();

    /* Every thruster must immediately drop to 1500 us neutral */
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }
    assert(mock_bsp_is_emergency_brake_tripped());

    /* Further thruster commands must be ignored */
    mock_can_inject_rx(ROV_CAN_ID_THRUSTER_CMD, buffer, (uint8_t)packed_len);
    mock_bsp_advance_time_ms(20);
    node2_app_step();
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }

    printf("[PASS] test_node2_emergency_break_cutoff\n");
}

void test_node2_solenoid_command(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    node2_app_init();

    rov_solenoid_cmd_t sol = {.solenoid_mask = 0x02A5};
    uint8_t buffer[64];
    size_t packed_len = 0;
    assert(rov_can_pack_solenoid_cmd(&sol, buffer, sizeof(buffer), &packed_len) == ROV_OK);
    mock_can_inject_rx(ROV_CAN_ID_SOLENOID_CMD, buffer, (uint8_t)packed_len);

    mock_bsp_advance_time_ms(1);
    node2_app_step();

    assert(mock_bsp_get_solenoid_mask() == 0x02A5);

    printf("[PASS] test_node2_solenoid_command\n");
}

void test_node2_nav_telemetry_stream(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);

    /* Set synthetic IMU orientation and depth (12.5 meters in seawater) */
    imu_data_t imu_data = {.qw = 0.7071f,
                           .qx = 0.0f,
                           .qy = 0.7071f,
                           .qz = 0.0f,
                           .gx_dps = 2.86f,
                           .gy_dps = -1.15f,
                           .gz_dps = 5.73f,
                           .status = 3};
    mock_sensors_set_imu(&imu_data);
    /* 12.5 meters in seawater (~1000 kg/m^3) corresponds to approx 2239 mbar absolute */
    mock_sensors_set_ms5837(2239.0f, 15.0f);

    node2_app_init();

    /* Advance 10 ms to trigger 100 Hz Nav telemetry */
    mock_bsp_advance_time_ms(10);
    node2_app_step();

    assert(mock_can_get_tx_count() >= 1);
    uint8_t tx_data[64];
    uint8_t tx_len = 0;
    assert(mock_can_find_latest_tx(ROV_CAN_ID_NAV_TELEMETRY, tx_data, &tx_len));

    rov_nav_telemetry_t nav;
    assert(rov_can_unpack_nav_telemetry(tx_data, tx_len, &nav) == ROV_OK);
    assert(nav.q_w > 0.70f && nav.q_w < 0.71f);
    assert(nav.gyro_z_rad_s > 0.09f && nav.gyro_z_rad_s < 0.11f);
    assert(nav.depth_meters > 12.0f && nav.depth_meters < 13.0f);
    assert(nav.imu_status == 3);

    printf("[PASS] test_node2_nav_telemetry_stream\n");
}

int main(void) {
    printf("Running Node 2 (Control Board) SIL Unit Tests...\n");
    test_node2_boot_state();
    test_node2_thruster_ramping();
    test_node2_heartbeat_timeout_failsafe();
    test_node2_emergency_break_cutoff();
    test_node2_solenoid_command();
    test_node2_nav_telemetry_stream();
    printf("All Node 2 SIL Tests Passed Successfully!\n");
    return 0;
}
