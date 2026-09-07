/**
 * @file test_multi_node_bus.c
 * @brief Multi-Node System SIL Simulation on In-Memory CAN FD Bus.
 * @organization Purdue ROV
 */

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Forward declarations from each subsystem node application */
extern void node1_app_init(void);
extern void node1_app_step(void);

extern void node2_app_init(void);
extern void node2_app_step(void);

extern void node3_app_init(void);
extern void node3_app_step(void);

void test_full_system_simulation(void) {
    mock_bsp_reset();
    mock_bsp_set_auto_advance_delay(false);
    mock_can_reset();
    mock_sensors_reset();

    /* 1. Initialize Subsea Vehicle Nodes */
    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_init();

    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_init();

    mock_can_set_current_node(ROV_NODE_POWER_SLAB);
    node3_app_init();

    /* 2. Pilot / Pi Core sends Thruster Command (0x100) commanding 1700 us */
    rov_thruster_cmd_t cmd;
    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        cmd.pwm_us[i] = 1700;
    }
    uint8_t tx_buf[64];
    size_t tx_len = 0;
    assert(rov_can_pack_thruster_cmd(&cmd, tx_buf, sizeof(tx_buf), &tx_len) == ROV_OK);

    /* 3. Execute 5 mission simulation cycles (10 ms each = 50 ms total) */
    for (int cycle = 0; cycle < 5; cycle++) {
        mock_bsp_advance_time_ms(10);

        /* Pilot transmits 0x100 on CAN bus */
        mock_can_set_current_node(ROV_NODE_PI_CORE);
        can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len);

        /* Step Node 2 (Control Board): receives 0x100, ramps PWMs, streams 100 Hz Nav (0x200) */
        mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
        node2_app_step();

        /* Step Node 1 (Pi Shield): evaluates leak state */
        mock_can_set_current_node(ROV_NODE_PI_SHIELD);
        node1_app_step();

        /* Step Node 3 (Power Slab): evaluates power */
        mock_can_set_current_node(ROV_NODE_POWER_SLAB);
        node3_app_step();
    }

    /* Verify Node 2 ramped: 1500 + 50 * 2 = 1600 us */
    assert(mock_bsp_get_pwm_us(0) == 1600);
    assert(mock_bsp_get_pwm_us(7) == 1600);

    /* Verify Pi Core received 0x200 Navigation Telemetry from Node 2 */
    mock_can_set_current_node(ROV_NODE_PI_CORE);
    uint32_t rx_id = 0;
    uint8_t rx_data[64];
    uint8_t rx_len = 0;
    bool found_nav = false;
    while (can_receive(&rx_id, rx_data, &rx_len)) {
        if (rx_id == ROV_CAN_ID_NAV_TELEMETRY) {
            found_nav = true;
            rov_nav_telemetry_t nav;
            assert(rov_can_unpack_nav_telemetry(rx_data, rx_len, &nav) == ROV_OK);
            assert(nav.imu_status == 3);
        }
    }
    assert(found_nav);

    /* 4. EMERGENCY LEAK INGRESS EVENT: Ingress detected on Node 1 */
    mock_bsp_set_leak_probe(0, true);
    mock_bsp_advance_time_ms(100);

    /* Step Node 1: detects leak, fires Priority 0 Emergency Break (0x001) */
    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
    node1_app_step();

    /* Step Node 2: receives 0x001 from CAN bus, cuts all thrusters to neutral 1500 us */
    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    node2_app_step();

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }
    assert(mock_bsp_is_emergency_brake_tripped());

    /* 5. Pilot attempts further thruster commands -> must be blocked */
    mock_can_set_current_node(ROV_NODE_PI_CORE);
    can_send(ROV_CAN_ID_THRUSTER_CMD, tx_buf, (uint8_t)tx_len);

    mock_can_set_current_node(ROV_NODE_CONTROL_BOARD);
    mock_bsp_advance_time_ms(20);
    node2_app_step();

    for (int i = 0; i < ROV_NUM_THRUSTERS; i++) {
        assert(mock_bsp_get_pwm_us((uint8_t)i) == ROV_PWM_STOP_US);
    }

    printf("[PASS] test_full_system_simulation\n");
}

int main(void) {
    printf("Running Multi-Node Full Vehicle SIL System Tests...\n");
    test_full_system_simulation();
    printf("All Multi-Node System Tests Passed Successfully!\n");
    return 0;
}
