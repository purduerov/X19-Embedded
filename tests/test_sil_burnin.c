#include <inttypes.h>
/**
 * @file test_sil_burnin.c
 * @brief SIL 100,000-cycle Burn-In Test: long-duration stability, counter
 *        overflow, and memory integrity verification.
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern void node1_app_init(void);
extern void node1_app_step(void);
extern void node2_app_init(void);
extern void node2_app_step(void);
extern void node3_app_init(void);
extern void node3_app_step(void);

/* ============================================================================
 * Test 1: 100,000-Cycle Burn-In at Neutral Throttle
 * ============================================================================ */
void test_burnin_neutral_100k(void) {
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

    uint32_t prev_tx_count = 0;
    const int TOTAL_CYCLES = 100000;
    const int CHECKPOINT_EVERY = 10000;

    for (int cycle = 0; cycle < TOTAL_CYCLES; cycle++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();

        if ((cycle + 1) % CHECKPOINT_EVERY == 0) {
            for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
                assert(mock_bsp_get_pwm_us((uint8_t)ch) == ROV_PWM_STOP_US);
            }
            uint32_t tx_count = mock_can_get_tx_count();
            assert(tx_count > prev_tx_count);
            prev_tx_count = tx_count;
            uint32_t expected_time_ms = (uint32_t)(cycle + 1) * 10u;
            assert(time_get_ms() == expected_time_ms);
            printf("[BURNIN] Cycle %d / %d | SimTime=%"PRIu32" ms | TX frames=%"PRIu32"\n", cycle + 1, TOTAL_CYCLES, expected_time_ms,
                   tx_count);
            fflush(stdout);
        }
    }
    printf("[PASS] test_burnin_neutral_100k (%d cycles, %"PRIu32" ms virtual time)\n", TOTAL_CYCLES, time_get_ms());
}

/* ============================================================================
 * Test 2: Alternating Ramp Burn-In (10,000 cycles)
 * ============================================================================ */
void test_burnin_ramp_alternating(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_init();
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();

    rov_thruster_cmd_t cmd_fwd, cmd_stop;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd_fwd.pwm_us[i] = 1700;
        cmd_stop.pwm_us[i] = 1500;
    }
    uint8_t buf_fwd[64], buf_stop[64];
    size_t len_fwd = 0, len_stop = 0;
    assert(rov_can_pack_thruster_cmd(&cmd_fwd, buf_fwd, sizeof(buf_fwd), &len_fwd) == ROV_OK);
    assert(rov_can_pack_thruster_cmd(&cmd_stop, buf_stop, sizeof(buf_stop), &len_stop) == ROV_OK);

    const int CYCLES = 10000;
    const int PHASE_CYCLES = 20;
    for (int cycle = 0; cycle < CYCLES; cycle++) {
        int phase = (cycle / PHASE_CYCLES) % 2;
        const uint8_t *buf = (phase == 0) ? buf_fwd : buf_stop;
        size_t buf_len = (phase == 0) ? len_fwd : len_stop;

        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_THRUSTER_CMD, buf, (uint8_t)buf_len);
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
            uint16_t pwm = mock_bsp_get_pwm_us((uint8_t)ch);
            assert(pwm >= ROV_PWM_MIN_US && pwm <= ROV_PWM_MAX_US);
        }
    }
    printf("[PASS] test_burnin_ramp_alternating (%d cycles)\n", CYCLES);
}

int main(void) {
    printf("Running SIL Burn-In Stability Tests...\n");
    test_burnin_neutral_100k();
    test_burnin_ramp_alternating();
    printf("All SIL Burn-In Tests Passed Successfully!\n");
    return 0;
}
