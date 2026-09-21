/**
 * @file test_sil_fuzz.c
 * @brief SIL CAN FD Frame Fuzzer: injects malformed frames and verifies firmware
 *        robustness (no crash, no hang, all PWMs remain in valid range).
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
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

        uint8_t fuzz_len = (uint8_t)(fuzz_rand() % 65u);
        for (int b = 0; b < fuzz_len; b++) {
            fuzz_data[b] = (uint8_t)(fuzz_rand() & 0xFFu);
        }

        mock_can_inject_node_rx(target_node, fuzz_id, fuzz_data, fuzz_len);

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
    printf("[PASS] test_can_fuzz_robustness (1000 fuzz iterations)\n");
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

    mock_can_set_bus_off(true);
    for (int cycle = 0; cycle < 50; cycle++) {
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
    mock_can_set_bus_off(false);
    for (int cycle = 0; cycle < 10; cycle++) {
        mock_bsp_advance_time_ms(10);
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();
    }
    for (int ch = 0; ch < ROV_NUM_THRUSTERS; ch++) {
        uint16_t pwm = mock_bsp_get_pwm_us((uint8_t)ch);
        assert(pwm >= ROV_PWM_MIN_US && pwm <= ROV_PWM_MAX_US);
    }
    printf("[PASS] test_bus_off_fault_injection\n");
}

int main(void) {
    printf("Running SIL CAN FD Fuzz and Fault Injection Tests...\n");
    test_can_fuzz_robustness();
    test_bus_off_fault_injection();
    printf("All SIL Fuzz Tests Passed Successfully!\n");
    return 0;
}