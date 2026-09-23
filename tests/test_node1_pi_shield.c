/**
 * @file test_node1_pi_shield.c
 * @brief Host-Native SIL Test for Node 1 (Pi Shield) Application Logic.
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

static void setup(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(ROV_NODE_PI_SHIELD);
}

void test_node1_nominal_telemetry(void) {
    setup();

    mock_sensors_set_bme280(1013.25f, 42.0f, 26.5f);

    mock_sensors_set_ina226(5.21f, 1.35f);

    node1_app_init();

    /*
     * Step before 100 ms interval:
     * no telemetry should have been sent yet.
     */
    node1_app_step();

    assert(mock_can_get_tx_count() == 0);

    /*
     * Advance 100 ms to trigger the 10 Hz telemetry path.
     */
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_can_get_tx_count() == 1);

    uint32_t tx_id = 0;
    uint8_t data[64];
    uint8_t len = 0;

    assert(mock_can_get_last_tx(&tx_id, data, &len));

    assert(tx_id == ROV_CAN_ID_ENV_TELEMETRY);

    rov_env_telemetry_t env;

    assert(rov_can_unpack_env_telemetry(data, len, &env) == ROV_OK);

    assert(env.pressure_hpa > 1013.0f && env.pressure_hpa < 1014.0f);

    assert(env.humidity_pct > 41.0f && env.humidity_pct < 43.0f);

    assert(env.leak_flags == 0);

    assert(mock_bsp_get_led_toggle_count() >= 1);

    assert(!mock_bsp_is_emergency_brake_tripped());

    printf("[PASS] test_node1_nominal_telemetry\n");
}

void test_node1_vacuum_loss_leak_trigger(void) {
    setup();

    /*
     * Start with a sealed enclosure pulled to
     * 750 hPa vacuum.
     */
    mock_sensors_set_bme280(750.0f, 30.0f, 22.0f);

    node1_app_init();

    /*
     * Establish baseline.
     */
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(!mock_bsp_is_emergency_brake_tripped());

    /*
     * Simulate vacuum loss:
     * pressure rises by 25 hPa.
     */
    mock_sensors_set_bme280(775.0f, 30.0f, 22.0f);

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    /*
     * Emergency break should have fired and
     * the local hardware brake should be tripped.
     */
    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Environmental leak = bit 0.
     */
    assert(alert_data[2] == 0x01);

    assert(alert_data[3] == 0x00);
    assert(alert_data[4] == 0x00);
    assert(alert_data[5] == 0x00);
    assert(alert_data[6] == 0x00);
    assert(alert_data[7] == 0x00);

    printf("[PASS] test_node1_vacuum_loss_leak_trigger\n");
}

void test_node1_humidity_spike_leak_trigger(void) {
    setup();

    mock_sensors_set_bme280(1013.25f, 35.0f, 24.0f);

    node1_app_init();

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(!mock_bsp_is_emergency_brake_tripped());

    /*
     * Spike humidity to 85%.
     * Threshold is 80%.
     */
    mock_sensors_set_bme280(1013.25f, 85.0f, 24.0f);

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Environmental leak = bit 0.
     */
    assert(alert_data[2] == 0x01);

    printf("[PASS] test_node1_humidity_spike_leak_trigger\n");
}

void test_node1_floor_probe_leak_polling_backup(void) {
    setup();

    mock_sensors_set_bme280(1013.25f, 35.0f, 24.0f);

    node1_app_init();

    /*
     * Establish nominal environmental telemetry.
     */
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(!mock_bsp_is_emergency_brake_tripped());

    /*
     * Simulate water on floor probe 0 without invoking
     * the EXTI handler.
     *
     * The 10 Hz application polling path must still
     * detect the leak as a redundant backup.
     */
    mock_bsp_set_leak_probe(0, true);

    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Probe 0 = bit 1.
     */
    assert(alert_data[2] == 0x02);

    printf("[PASS] test_node1_floor_probe_leak_polling_backup\n");
}

void test_node1_floor_probe0_irq_emergency(void) {
    setup();

    mock_sensors_set_bme280(1013.25f, 35.0f, 24.0f);

    node1_app_init();

    /*
     * Water appears on physical floor probe 0.
     */
    mock_bsp_set_leak_probe(0, true);

    /*
     * Simulate the EXTI callback immediately.
     *
     * Notice that time is NOT advanced by 100 ms.
     * The emergency must occur independently of the
     * periodic telemetry loop.
     */
    node1_leak_irq_handler();

    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Probe 0 = bit 1 = 0x02.
     */
    assert(alert_data[2] == 0x02);

    assert(alert_data[3] == 0x00);
    assert(alert_data[4] == 0x00);
    assert(alert_data[5] == 0x00);
    assert(alert_data[6] == 0x00);
    assert(alert_data[7] == 0x00);

    printf("[PASS] test_node1_floor_probe0_irq_emergency\n");
}

void test_node1_floor_probe1_irq_emergency(void) {
    setup();

    node1_app_init();

    /*
     * Water appears on physical floor probe 1.
     */
    mock_bsp_set_leak_probe(1, true);

    node1_leak_irq_handler();

    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Probe 1 = bit 2 = 0x04.
     */
    assert(alert_data[2] == 0x04);

    printf("[PASS] test_node1_floor_probe1_irq_emergency\n");
}

void test_node1_both_floor_probes_irq_emergency(void) {
    setup();

    node1_app_init();

    /*
     * Water detected by both floor probes.
     */
    mock_bsp_set_leak_probe(0, true);

    mock_bsp_set_leak_probe(1, true);

    node1_leak_irq_handler();

    assert(mock_bsp_is_emergency_brake_tripped());

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    uint8_t alert_data[64];
    uint8_t alert_len = 0;

    assert(mock_can_find_latest_tx(ROV_CAN_ID_EMERGENCY_BREAK, alert_data, &alert_len));

    assert(alert_len == 8);

    assert(alert_data[0] == 0xAA);
    assert(alert_data[1] == 0x55);

    /*
     * Probe 0 = 0x02
     * Probe 1 = 0x04
     *
     * Both = 0x06.
     */
    assert(alert_data[2] == 0x06);

    printf("[PASS] test_node1_both_floor_probes_irq_emergency\n");
}

void test_node1_irq_emergency_is_latched(void) {
    setup();

    node1_app_init();

    mock_bsp_set_leak_probe(0, true);

    /*
     * Simulate multiple interrupt callbacks caused by
     * noise/bouncing around the first wet transition.
     */
    node1_leak_irq_handler();
    node1_leak_irq_handler();
    node1_leak_irq_handler();

    /*
     * Only one emergency frame should be transmitted.
     */
    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    /*
     * Advance into the normal polling interval.
     *
     * The probe is still wet, but the polling backup
     * must not flood additional 0x001 frames.
     */
    mock_bsp_advance_time_ms(100);
    node1_app_step();

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EMERGENCY_BREAK) == 1);

    printf("[PASS] test_node1_irq_emergency_is_latched\n");
}

int main(void) {
    printf("Running Node 1 (Pi Shield) SIL Unit Tests...\n");

    test_node1_nominal_telemetry();

    test_node1_vacuum_loss_leak_trigger();

    test_node1_humidity_spike_leak_trigger();

    test_node1_floor_probe_leak_polling_backup();

    test_node1_floor_probe0_irq_emergency();

    test_node1_floor_probe1_irq_emergency();

    test_node1_both_floor_probes_irq_emergency();

    test_node1_irq_emergency_is_latched();

    printf("All Node 1 SIL Tests Passed Successfully!\n");

    return 0;
}
