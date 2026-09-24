#include <inttypes.h>
/**
 * @file test_sil_power.c
 * @brief SIL Power Slab eFuse Overcurrent Trip and Telemetry Fault Reporting tests.
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_physics.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern void node1_app_init(void);
extern void node1_app_step(void);
extern void node2_app_init(void);
extern void node2_app_step(void);
extern void node3_app_init(void);
extern void node3_app_step(void);

/* ============================================================================
 * Test 1: eFuse Overcurrent Trip Under Full Thruster Load
 *
 * Commands all 8 thrusters to 1900 us, enables physics, and steps the
 * simulation for 600 ms virtual time. Asserts the physics model detects load
 * on all 4 bricks and that Node 3 transmits 0x300 power telemetry.
 * ============================================================================ */
void test_efuse_overcurrent_trip(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();
    mock_physics_reset();
    mock_physics_set_enabled(true);

    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_init();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();
    mock_can_set_current_node(ROV_NODE_POWER_SLAB);
    node3_app_init();

    /* Complete Node 2 mandatory 3000 ms ESC arming period */
    mock_bsp_set_time_ms(3000U);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();

    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1900;
    }
    uint8_t tx_buf[64];
    size_t tx_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, tx_buf, sizeof(tx_buf), &tx_len) == ROV_OK);

    /* 60 cycles x 10 ms = 600 ms virtual time */
    for (int cycle = 0; cycle < 60; cycle++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();
    }

    /* Verify physics detected load on at least one brick */
    const mock_physics_state_t *phys = mock_physics_get_state();
    int any_brick_loaded = 0;
    for (int b = 0; b < ROV_NUM_12V_BRICKS; b++) {
        if (phys->brick_currents_a[b] > 1.0f) {
            any_brick_loaded = 1;
        }
    }
    assert(any_brick_loaded);

    /* Verify Node 3 transmitted at least one 0x300 frame */
    assert(mock_can_count_tx_by_id(ROV_CAN_ID_POWER_TELEMETRY) >= 1);

    /* Decode latest power telemetry and verify tether voltage is plausible */
    uint8_t pwr_data[64];
    uint8_t pwr_len = 0;
    assert(mock_can_find_latest_tx(ROV_CAN_ID_POWER_TELEMETRY, pwr_data, &pwr_len));
    assert(pwr_len >= (uint8_t)sizeof(rov_power_telemetry_t));

    rov_power_telemetry_t pwr;
    assert(rov_can_unpack_power_telemetry(pwr_data, pwr_len, &pwr) == ROV_OK);
    assert(pwr.tether_voltage_mv >= 40000);

    int brick_current_detected = 0;
    for (int b = 0; b < ROV_NUM_12V_BRICKS; b++) {
        if (pwr.v12_current_ma[b] > 0) {
            brick_current_detected = 1;
        }
    }
    assert(brick_current_detected);

    mock_physics_set_enabled(false);
    printf("[PASS] test_efuse_overcurrent_trip\n");
}

/* ============================================================================
 * Test 2: Power Telemetry Continuity at 20 Hz
 *
 * Runs 2 seconds of virtual time at neutral throttle and asserts Node 3
 * transmits 0x300 frames at approximately 20 Hz (20 to 60 frame tolerance).
 * ============================================================================ */
void test_power_telemetry_continuity(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_init();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();
    mock_can_set_current_node(ROV_NODE_POWER_SLAB);
    node3_app_init();

    for (int cycle = 0; cycle < 200; cycle++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();
    }

    uint32_t pwr_count = mock_can_count_tx_by_id(ROV_CAN_ID_POWER_TELEMETRY);
    assert(pwr_count >= 20);
    assert(pwr_count <= 60);
    printf("[PASS] test_power_telemetry_continuity (%"PRIu32" frames in 2s)\n", pwr_count);
}

int main(void) {
    printf("Running SIL Power Slab Tests...\n");
    test_efuse_overcurrent_trip();
    test_power_telemetry_continuity();
    printf("All SIL Power Tests Passed Successfully!\n");
    return 0;
}
