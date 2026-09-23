#include <inttypes.h>
/**
 * @file test_sil_safety.c
 * @brief SIL Safety Verification: Tether Watchdog SLA, Emergency Break Latency,
 *        and Pneumatic Solenoid Interlock tests.
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"
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
 * Test 1: Tether Watchdog SLA
 *
 * Verifies that Node 2 (Control Board) ramps all thrusters to neutral stop
 * (1500 us) within the ROV_HEARTBEAT_TIMEOUT_MS window after the last
 * heartbeat frame stops arriving on the CAN bus.
 * ============================================================================ */
void test_tether_watchdog_sla(void) {
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

    /* Complete Node 2 mandatory 3000 ms ESC arming period */
    mock_bsp_set_time_ms(3000U);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();

    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1700;
    }
    uint8_t tx_buf[64];
    size_t tx_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, tx_buf, sizeof(tx_buf), &tx_len) == ROV_OK);

    /* Feed heartbeat (thruster commands) for 100 ms (10 cycles x 10 ms) */
    for (int cycle = 0; cycle < 10; cycle++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
    }

    uint16_t mid_pwm = mock_bsp_get_pwm_us(0);
    assert(mid_pwm > ROV_PWM_STOP_US);

    /* Stop heartbeat and advance past timeout window */
    uint32_t watchdog_ticks = (ROV_HEARTBEAT_TIMEOUT_MS / 10) + 2;
    for (uint32_t t = 0; t < watchdog_ticks; t++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
    }

    /* Allow slew-rate ramp-down to complete */
    uint32_t ramp_down_ticks = ((mid_pwm - ROV_PWM_STOP_US) / ROV_PWM_MAX_SLEW_RATE_US_PER_MS / 10) + 2;
    for (uint32_t t = 0; t < ramp_down_ticks; t++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
    }

    for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
        assert(mock_bsp_get_pwm_us((uint8_t)ch) == ROV_PWM_STOP_US);
    }
    printf("[PASS] test_tether_watchdog_sla\n");
}

/* ============================================================================
 * Test 2: Emergency Break Latency
 *
 * Injects a leak on Node 1. Measures the virtual-time latency from injection
 * to the moment Node 2 receives 0x001 and trips its hardware brake.
 * ============================================================================ */
void test_emergency_break_latency(void) {
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

    /* Warm up 5 ticks */
    for (int i = 0; i < 5; i++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();
    }

    /* Inject leak on floor probe 0 */
    mock_bsp_set_leak_probe(0, true);
    uint32_t leak_inject_time_ms = time_get_ms();

    /* Step Node 1 in 10 ms increments until it processes the leak check (10 Hz = 100 ms period) */
    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    while (mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 0) {
        mock_bsp_advance_time_ms(10);
        node1_app_step();
    }

    /* Node 1 transmitted 0x001 */
    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) >= 1);

    /* Advance one tick and step Node 2: it receives 0x001 and trips brake immediately */
    uint32_t ebreak_tx_time_ms = time_get_ms();
    mock_bsp_advance_time_ms(10);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();

    /* Node 2 reaction latency after 0x001 broadcast must be <= 10 ms (single control tick) */
    uint32_t rx_latency_ms = time_get_ms() - ebreak_tx_time_ms;
    assert(rx_latency_ms <= 10);

    /* Total virtual latency from injection to brake trip must be <= 110 ms (one 10Hz sampling period + 10ms control
     * cycle) */
    uint32_t total_latency_ms = time_get_ms() - leak_inject_time_ms;
    assert(total_latency_ms <= 110);

    for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
        assert(mock_bsp_get_pwm_us((uint8_t)ch) == ROV_PWM_STOP_US);
    }
    assert(mock_bsp_is_emergency_brake_tripped());
    printf("[PASS] test_emergency_break_latency (rx_latency = %"PRIu32" ms, total = %"PRIu32" ms)\n", rx_latency_ms,
           total_latency_ms);
}

/* ============================================================================
 * Test 3: Pneumatic Solenoid Interlock Verification
 *
 * For each of the 5 double-acting solenoid pairs, asserts that both coils
 * are never simultaneously energized (hardware interlock invariant).
 * ============================================================================ */
void test_solenoid_interlock(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();

    assert(mock_bsp_get_solenoid_mask() == 0x0000);

    uint8_t sol_buf[2];
    size_t sol_len = 0;
    rov_solenoid_cmd_t sol_cmd;

    /* Test each of the 5 valve pairs: alternating coil A and coil B */
    for (int valve = 0; valve < ROV_NUM_SOLENOIDS; valve++) {
        int ch_a = valve * 2;
        int ch_b = valve * 2 + 1;

        /* Step 1: Command channel A only */
        sol_cmd.solenoid_mask = (uint16_t)(1u << ch_a);
        assert(rov_can_pack_solenoid_cmd(&sol_cmd, sol_buf, sizeof(sol_buf), &sol_len) == ROV_OK);
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_SOLENOID_CMD, sol_buf, (uint8_t)sol_len);
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        uint16_t mask_a = mock_bsp_get_solenoid_mask();
        assert((mask_a & (1u << ch_a)) != 0);
        assert((mask_a & (1u << ch_b)) == 0);

        /* Step 2: Command channel B only (switching direction) */
        sol_cmd.solenoid_mask = (uint16_t)(1u << ch_b);
        assert(rov_can_pack_solenoid_cmd(&sol_cmd, sol_buf, sizeof(sol_buf), &sol_len) == ROV_OK);
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_SOLENOID_CMD, sol_buf, (uint8_t)sol_len);
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        uint16_t mask_b = mock_bsp_get_solenoid_mask();
        assert((mask_b & (1u << ch_a)) == 0);
        assert((mask_b & (1u << ch_b)) != 0);

        /* Step 3: Deactivate both coils */
        sol_cmd.solenoid_mask = 0x0000;
        assert(rov_can_pack_solenoid_cmd(&sol_cmd, sol_buf, sizeof(sol_buf), &sol_len) == ROV_OK);
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_SOLENOID_CMD, sol_buf, (uint8_t)sol_len);
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        assert((mock_bsp_get_solenoid_mask() & ((1u << ch_a) | (1u << ch_b))) == 0);
    }

    printf("[PASS] test_solenoid_interlock\n");
}

int main(void) {
    printf("Running SIL Safety Verification Tests...\n");
    test_tether_watchdog_sla();
    test_emergency_break_latency();
    test_solenoid_interlock();
    printf("All SIL Safety Tests Passed Successfully!\n");
    return 0;
}
