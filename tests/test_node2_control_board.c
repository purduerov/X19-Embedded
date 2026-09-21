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
#include "rov_timesync.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void setup(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
}

static void complete_esc_arming(void)  // this will simulate the ESC arming process by advancing time and calling node2_app_step
{
    mock_bsp_set_time_ms(3000U);
    node2_app_step();
}

void test_node2_boot_state(void) {
    setup();
    node2_app_init();

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }
    assert(mock_bsp_get_solenoid_mask() == 0);
    assert(!mock_bsp_is_emergency_brake_tripped());

    printf("[PASS] test_node2_boot_state\n");
}

void test_node2_esc_arming(void) {
    setup();
    node2_app_init();

    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1800U;
    }

    uint8_t buffer[64];
    size_t packed_len = 0;

    assert(rov_can_pack_thruster_cmd(
        &cmd,
        buffer,
        sizeof(buffer),
        &packed_len) == ROV_OK);

    /* Thruster commands must be ignored before arming completes. */
    mock_bsp_set_time_ms(2999U);
    mock_can_inject_rx(
        ROV_CAN_ID_THRUSTER_CMD,
        buffer,
        (uint8_t)packed_len);

    node2_app_step();

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }

    /* At exactly 3000 ms, the ESCs become active and may accept commands. */
    mock_bsp_set_time_ms(3000U);
    mock_can_inject_rx(
        ROV_CAN_ID_THRUSTER_CMD,
        buffer,
        (uint8_t)packed_len);
    node2_app_step();

    /* The accepted target should begin ramping on the next elapsed step. */
    mock_bsp_set_time_ms(3010U);
    node2_app_step();

    assert(mock_bsp_get_pwm_us(0) > ROV_PWM_STOP_US);

    printf("[PASS] test_node2_esc_arming\n");
}

void test_node2_thruster_ramping(void) {
    setup();
    node2_app_init();
    complete_esc_arming(); // simulate ESC arming completion

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
    setup();
    node2_app_init();
    complete_esc_arming(); // simulate ESC arming completion

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
    setup();
    node2_app_init();
    complete_esc_arming();

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
    setup();
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
    setup();

    /* Set synthetic IMU orientation and depth (12.5 meters in seawater) */
    imu_data_t imu_mock = {
        .q_w = 0.7071f,
        .q_x = 0.0f,
        .q_y = 0.7071f,
        .q_z = 0.0f,
        .gyro_x_dps = 2.86f,
        .gyro_y_dps = -1.15f,
        .gyro_z_dps = 5.73f,
        .status = 3
    };
    mock_sensors_set_imu(&imu_mock);
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

void test_node2_time_sync_and_latency_response(void) {
    setup();
    node2_app_init();

    /* 1. Inject Master Time Sync Frame (0x010) */
    rov_time_sync_master_t master = {
        .master_time_us = 1726340000000000ULL,
        .sync_seq = 1,
        .flags = 0x01,
        .reserved = {0, 0, 0}
    };
    uint8_t sync_buf[64];
    size_t sync_len = 0;
    assert(rov_can_pack_time_sync_master(&master, sync_buf, sizeof(sync_buf), &sync_len) == ROV_OK);
    mock_can_inject_rx(ROV_CAN_ID_TIME_SYNC_MASTER, sync_buf, (uint8_t)sync_len);

    /* Advance 10 ms (10,000 us) to trigger step and 100 Hz Nav telemetry */
    mock_bsp_advance_time_ms(10);
    node2_app_step();

    /* Verify Navigation Telemetry contains synchronized timestamp */
    uint8_t tx_data[64];
    uint8_t tx_len = 0;
    assert(mock_can_find_latest_tx(ROV_CAN_ID_NAV_TELEMETRY, tx_data, &tx_len));
    rov_nav_telemetry_t nav;
    assert(rov_can_unpack_nav_telemetry(tx_data, tx_len, &nav) == ROV_OK);
    assert(nav.timestamp_us >= 1726340000000000ULL);

    /* 2. Inject Delay Request Frame (0x011) to measure latency */
    rov_time_sync_req_t req = {
        .target_node_id = ROV_NODE_CONTROL_BOARD,
        .seq = 15,
        .reserved = 0,
        .flags = 0,
        .t1_us = 1726340000000500ULL
    };
    uint8_t req_buf[64];
    size_t req_len = 0;
    assert(rov_can_pack_time_sync_req(&req, req_buf, sizeof(req_buf), &req_len) == ROV_OK);
    mock_can_inject_rx(ROV_CAN_ID_TIME_SYNC_REQ, req_buf, (uint8_t)req_len);

    /* Advance time by 5 us for response processing */
    mock_bsp_advance_time_us(5);
    node2_app_step();

    /* Verify Node 2 transmitted Delay Response (0x012) */
    assert(mock_can_find_latest_tx(ROV_CAN_ID_TIME_SYNC_RESP, tx_data, &tx_len));
    rov_time_sync_resp_t resp;
    assert(rov_can_unpack_time_sync_resp(tx_data, tx_len, &resp) == ROV_OK);
    assert(resp.responder_node_id == ROV_NODE_CONTROL_BOARD);
    assert(resp.seq == 15);
    assert(resp.t1_us == req.t1_us);
    assert(resp.t2_us <= resp.t3_us);

    /* Verify latency calculation */
    uint64_t t4_us = 1726340000000600ULL;
    int64_t offset_us = 0;
    uint32_t rtt_us = 0;
    uint32_t one_way_delay_us = 0;
    assert(rov_timesync_calc_latency(resp.t1_us, resp.t2_us, resp.t3_us, t4_us,
                                     &offset_us, &rtt_us, &one_way_delay_us) == ROV_OK);
    assert(one_way_delay_us > 0);

    printf("[PASS] test_node2_time_sync_and_latency_response\n");
}

int main(void) {
    printf("Running Node 2 (Control Board) SIL Unit Tests...\n");
    test_node2_boot_state();
    test_node2_esc_arming();
    test_node2_thruster_ramping();
    test_node2_heartbeat_timeout_failsafe();
    test_node2_emergency_break_cutoff();
    test_node2_solenoid_command();
    test_node2_nav_telemetry_stream();
    test_node2_time_sync_and_latency_response();
    printf("All Node 2 SIL Tests Passed Successfully!\n");
    return 0;
}
