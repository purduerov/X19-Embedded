/**
 * @file test_node1_pi_shield.c
 * @brief Host-Native SIL Test for Node 1 (Pi Shield) Application Logic.
 * @organization Purdue ROV
 */

#include "app.h"
#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"
#include "x19_can_protocol.h"
#include "x19_parameters.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void test_node1_nominal_telemetry(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(X19_NODE_PI_SHIELD);

    mock_sensors_set_bme280(1013.25f, 42.0f, 26.5f);
    mock_sensors_set_ina226(5.21f, 1.35f);

    node1_app_init();

    /* Step before 100 ms interval: no telemetry sent yet */
    node1_app_step();
    assert(mock_can_get_tx_count() == 0);

    /* Advance 100 ms to trigger 10 Hz telemetry */
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_can_get_tx_count() == 1);
    uint32_t tx_id = 0;
    uint8_t data[64];
    uint8_t len = 0;
    assert(mock_can_get_last_tx(&tx_id, data, &len));
    assert(tx_id == X19_CAN_ID_ENV_TELEMETRY);

    x19_env_telemetry_t env;
    assert(x19_can_unpack_env_telemetry(data, len, &env) == X19_OK);
    assert(env.pressure_hpa > 1013.0f && env.pressure_hpa < 1014.0f);
    assert(env.humidity_pct > 41.0f && env.humidity_pct < 43.0f);
    assert(env.leak_flags == 0);
    assert(mock_bsp_get_led_toggle_count() >= 1);
    assert(!mock_bsp_is_emergency_brake_tripped());

    printf("[PASS] test_node1_nominal_telemetry\n");
}

void test_node1_vacuum_loss_leak_trigger(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(X19_NODE_PI_SHIELD);

    /* Start with a sealed enclosure pulled to 750 hPa vacuum */
    mock_sensors_set_bme280(750.0f, 30.0f, 22.0f);
    node1_app_init();

    /* Establish baseline */
    mock_bsp_advance_time_ms(100);
    node1_app_step();
    assert(!mock_bsp_is_emergency_brake_tripped());

    /* Simulate vacuum loss: pressure rises by 25 hPa to 775 hPa (> 15 hPa threshold) */
    mock_sensors_set_bme280(775.0f, 30.0f, 22.0f);
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    /* Emergency break should have been fired and hardware brake tripped */
    assert(mock_bsp_is_emergency_brake_tripped());
    assert(mock_can_count_tx_by_id(X19_CAN_ID_EMERGENCY_BREAK) >= 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;
    assert(mock_can_find_latest_tx(X19_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));
    assert(alert_len == 8);
    assert(alert_data[0] == 0xAA && alert_data[1] == 0x55);

    printf("[PASS] test_node1_vacuum_loss_leak_trigger\n");
}

void test_node1_humidity_spike_leak_trigger(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(X19_NODE_PI_SHIELD);

    mock_sensors_set_bme280(1013.25f, 35.0f, 24.0f);
    node1_app_init();
    mock_bsp_advance_time_ms(100);
    node1_app_step();
    assert(!mock_bsp_is_emergency_brake_tripped());

    /* Spike humidity to 85% (threshold is 80%) */
    mock_sensors_set_bme280(1013.25f, 85.0f, 24.0f);
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_bsp_is_emergency_brake_tripped());
    assert(mock_can_count_tx_by_id(X19_CAN_ID_EMERGENCY_BREAK) >= 1);

    printf("[PASS] test_node1_humidity_spike_leak_trigger\n");
}

void test_node1_floor_probe_leak_trigger(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();
    mock_can_set_current_node(X19_NODE_PI_SHIELD);

    mock_sensors_set_bme280(1013.25f, 35.0f, 24.0f);
    node1_app_init();
    mock_bsp_advance_time_ms(100);
    node1_app_step();
    assert(!mock_bsp_is_emergency_brake_tripped());

    /* Ingress detected on floor leak probe 0 */
    mock_bsp_set_leak_probe(0, true);
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_bsp_is_emergency_brake_tripped());
    assert(mock_can_count_tx_by_id(X19_CAN_ID_EMERGENCY_BREAK) >= 1);

    printf("[PASS] test_node1_floor_probe_leak_trigger\n");
}

int main(void) {
    printf("Running Node 1 (Pi Shield) SIL Unit Tests...\n");
    test_node1_nominal_telemetry();
    test_node1_vacuum_loss_leak_trigger();
    test_node1_humidity_spike_leak_trigger();
    test_node1_floor_probe_leak_trigger();
    printf("All Node 1 SIL Tests Passed Successfully!\n");
    return 0;
}
