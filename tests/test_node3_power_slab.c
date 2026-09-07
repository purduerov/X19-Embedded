/**
 * @file test_node3_power_slab.c
 * @brief Host-Native SIL Test for Node 3 (Power Slab) Application Logic.
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

void test_node3_nominal_telemetry(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_POWER_SLAB);

    /* Configure 5 PMBus bricks: 5.2V @ 2A, and 4x 12V @ 5A */
    mock_sensors_set_tps25990(0, 48.0f, 5.2f, 2.0f, 32.0f, 0);
    for (int i = 1; i < 5; i++) {
        mock_sensors_set_tps25990((uint8_t)i, 48.0f, 12.0f, 5.0f, 38.0f, 0);
    }

    node3_app_init();

    /* Step before 50 ms (20 Hz): no packet sent */
    node3_app_step();
    assert(mock_can_get_tx_count() == 0);

    /* Advance 50 ms to trigger 20 Hz power telemetry */
    mock_bsp_advance_time_ms(50);
    node3_app_step();

    assert(mock_can_get_tx_count() == 1);
    uint8_t tx_data[64];
    uint8_t tx_len = 0;
    assert(mock_can_find_latest_tx(ROV_CAN_ID_POWER_TELEMETRY, tx_data, &tx_len));

    rov_power_telemetry_t pwr;
    assert(rov_can_unpack_power_telemetry(tx_data, tx_len, &pwr) == ROV_OK);
    assert(pwr.tether_voltage_mv == 48000);
    assert(pwr.v5_voltage_mv == 5200);
    assert(pwr.v5_current_ma == 2000);
    assert(pwr.v12_current_ma[0] == 5000);
    assert(pwr.pcb_temp_c == 380); /* 38.0 C */
    assert((pwr.status_flags & 0x0001) == 0);

    printf("[PASS] test_node3_nominal_telemetry\n");
}

void test_node3_overcurrent_fault_alert(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_POWER_SLAB);

    /* Set brick 2 to 28A (> 25A maximum rating) */
    mock_sensors_set_tps25990(2, 48.0f, 12.0f, 28.0f, 45.0f, 0);

    node3_app_init();
    mock_bsp_advance_time_ms(50);
    node3_app_step();

    /* Must broadcast Priority 0 eFuse Fault Alert (0x005) */
    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EFUSE_FAULT_ALERT) >= 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;
    assert(mock_can_find_latest_tx(ROV_CAN_ID_EFUSE_FAULT_ALERT, alert_data, &alert_len));
    assert(alert_data[0] == 0xEF);

    printf("[PASS] test_node3_overcurrent_fault_alert\n");
}

void test_node3_overtemperature_fault_alert(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(ROV_NODE_POWER_SLAB);

    /* Set brick 1 to 90 C (> 85 C safe threshold) */
    mock_sensors_set_tps25990(1, 48.0f, 12.0f, 10.0f, 90.0f, 0);

    node3_app_init();
    mock_bsp_advance_time_ms(50);
    node3_app_step();

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EFUSE_FAULT_ALERT) >= 1);

    printf("[PASS] test_node3_overtemperature_fault_alert\n");
}

int main(void) {
    printf("Running Node 3 (Power Slab) SIL Unit Tests...\n");
    test_node3_nominal_telemetry();
    test_node3_overcurrent_fault_alert();
    test_node3_overtemperature_fault_alert();
    printf("All Node 3 SIL Tests Passed Successfully!\n");
    return 0;
}
