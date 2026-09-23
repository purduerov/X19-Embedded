/**
 * @file test_sil_fuzz.c
 * @brief SIL CAN FD Frame Fuzzer: injects malformed frames and verifies firmware
 *        robustness (no crash, no hang, all PWMs remain in valid range).
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

/* Lightweight xorshift32 PRNG for reproducible fuzz sequences */
static uint32_t s_fuzz_seed = 0xDEADBEEFu;
static uint32_t fuzz_rand(void) {
    s_fuzz_seed ^= s_fuzz_seed << 13u;
    s_fuzz_seed ^= s_fuzz_seed >> 17u;
    s_fuzz_seed ^= s_fuzz_seed << 5u;
    return s_fuzz_seed;
}

/* ============================================================================
 * Test 1: CAN FD Frame Fuzzer - Robustness Under Malformed Input
 * ============================================================================ */
void test_can_fuzz_robustness(void) {
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

    static const uint32_t known_ids[] = {0x001, 0x005, 0x100, 0x110, 0x200, 0x210, 0x300, 0x310, 0x700, 0x701};
    const int known_id_count = (int)(sizeof(known_ids) / sizeof(known_ids[0]));
    uint8_t fuzz_data[64];
    uint32_t accepted_frames = 0;
    uint32_t rejected_oversize_frames = 0;

    for (int iter = 0; iter < 1000; iter++) {
        uint32_t r = fuzz_rand();

        rov_node_id_t target_node;
        switch (iter % 3) {
        case 0:
            target_node = ROV_NODE_CONTROL_BOARD;
            break;
        case 1:
            target_node = ROV_NODE_PI_SHIELD;
            break;
        default:
            target_node = ROV_NODE_POWER_SLAB;
            break;
        }

        uint32_t fuzz_id;
        if ((r & 0x3u) == 0) {
            fuzz_id = known_ids[(r >> 4u) % (uint32_t)known_id_count];
        } else {
            fuzz_id = (fuzz_rand() & 0x7FFu);
        }

        uint32_t generated_len = ((r & 0x1Fu) == 0u) ? 65u + (fuzz_rand() % 64u) : fuzz_rand() % 65u;
        uint8_t fuzz_len = (uint8_t)generated_len;
        size_t data_bytes = generated_len < sizeof(fuzz_data) ? generated_len : sizeof(fuzz_data);
        for (size_t b = 0; b < data_bytes; b++) {
            fuzz_data[b] = (uint8_t)(fuzz_rand() & 0xFFu);
        }

        bool injected = mock_can_inject_node_rx(target_node, fuzz_id, fuzz_data, fuzz_len);
        if (generated_len > MOCK_CAN_MAX_FRAME_SIZE) {
            assert(!injected);
            rejected_oversize_frames++;
        } else if (injected) {
            accepted_frames++;
        }

        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();

        for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
            uint16_t pwm = mock_bsp_get_pwm_us((uint8_t)ch);
            assert(pwm >= ROV_PWM_MIN_US && pwm <= ROV_PWM_MAX_US);
        }
    }
    assert(accepted_frames > 0);
    assert(rejected_oversize_frames > 0);
    printf("[PASS] test_can_fuzz_robustness (1000 iterations, %u accepted, %u oversize rejected)\n",
           accepted_frames, rejected_oversize_frames);
}

/* ============================================================================
 * Test 2: Bus-Off Fault Injection
 * ============================================================================ */
void test_bus_off_fault_injection(void) {
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

    /* Complete ESC arming, then establish a non-neutral commanded output. */
    mock_bsp_set_time_ms(3000U);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();

    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1700U;
    }
    uint8_t tx_buf[64];
    size_t tx_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, tx_buf, sizeof(tx_buf), &tx_len) == ROV_OK);
    mock_can_set_current_node(ROV_NODE_PI_CORE);
    assert(can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len));
    mock_bsp_advance_time_ms(10U);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();
    assert(mock_bsp_get_pwm_us(0) > ROV_PWM_STOP_US);

    /* Isolate Node 2's controller; other nodes remain online. */
    mock_can_set_bus_off(true);
    assert(can_is_bus_off());
    uint32_t tx_count_at_fault = mock_can_get_tx_count();

    /* Lost receive traffic must expire the heartbeat and ramp every thruster to neutral. */
    for (int cycle = 0; cycle < 40; cycle++) {
        mock_bsp_advance_time_ms(10);
        node2_app_step();
    }
    assert(mock_can_get_tx_count() == tx_count_at_fault);
    for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
        assert(mock_bsp_get_pwm_us((uint8_t)ch) == ROV_PWM_STOP_US);
    }

    /* Recovery restores the link and allows a fresh command to move the outputs. */
    mock_can_set_bus_off(false);
    assert(!can_is_bus_off());
    mock_can_set_current_node(ROV_NODE_PI_CORE);
    assert(can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len));
    mock_bsp_advance_time_ms(10U);
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();
    assert(mock_bsp_get_pwm_us(0) > ROV_PWM_STOP_US);
    printf("[PASS] test_bus_off_fault_injection (neutral failsafe and recovery)\n");
}

int main(void) {
    printf("Running SIL CAN FD Fuzz and Fault Injection Tests...\n");
    test_can_fuzz_robustness();
    test_bus_off_fault_injection();
    printf("All SIL Fuzz Tests Passed Successfully!\n");
    return 0;
}
