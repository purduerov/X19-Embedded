/**
 * @file test_node3_power_slab.c
 * @brief Host-Native SIL Test for Node 3 (Power Slab) Application Logic.
 * @organization Purdue ROV
 */

#include "app.h"
#include "power_sequence.h"

#include "mocks/mock_bsp.h"
#include "mocks/mock_can.h"
#include "mocks/mock_sensors.h"

#include "rov_can_protocol.h"
#include "rov_parameters.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TEST_NUM_POWER_BRICKS 4U

static void setup(void) {
    mock_bsp_reset();
    mock_can_reset();
    mock_sensors_reset();

    mock_can_set_current_node(ROV_NODE_POWER_SLAB);

    /*
     * Default power-sequencing conditions.
     */
    mock_bsp_set_logic_voltage_mv(0U);
    mock_bsp_set_pcb_temperature_c(25.0f);
    mock_bsp_set_lm74700_status_ok(true);
}

/*
 * ==========================================================================
 * Existing Node 3 telemetry tests
 * ==========================================================================
 */

void test_node3_nominal_telemetry(void) {
    setup();

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
    setup();

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
    setup();

    /* Set brick 1 to 90 C (> 85 C safe threshold) */
    mock_sensors_set_tps25990(1, 48.0f, 12.0f, 10.0f, 90.0f, 0);

    node3_app_init();

    mock_bsp_advance_time_ms(50);

    node3_app_step();

    assert(mock_can_count_tx_by_id(ROV_CAN_ID_EFUSE_FAULT_ALERT) >= 1);

    printf("[PASS] test_node3_overtemperature_fault_alert\n");
}

/*
 * ==========================================================================
 * Power Sequencing SIL Tests
 * ==========================================================================
 */

/**
 * Verify cold boot remains in PWR_SEQ_WAIT_LOGIC_STABLE
 * until the logic rail has been above 5.0 V continuously
 * for 500 ms.
 */
void test_node3_power_sequence_logic_stability(void) {
    setup();

    power_sequence_init();

    assert(power_sequence_get_state() == PWR_SEQ_INIT);

    /*
     * First step moves INIT -> WAIT_LOGIC_STABLE.
     */
    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_WAIT_LOGIC_STABLE);

    /*
     * Raise logic rail above 5.0 V.
     * This sample starts the stability timer.
     */
    mock_bsp_set_logic_voltage_mv(5001U);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_WAIT_LOGIC_STABLE);

    /*
     * 499 ms is not enough.
     */
    mock_bsp_advance_time_ms(499U);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_WAIT_LOGIC_STABLE);

    /*
     * Exactly 500 ms allows transition.
     */
    mock_bsp_advance_time_ms(1U);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_DIAGNOSTICS);

    printf("[PASS] test_node3_power_sequence_logic_stability\n");
}

/**
 * Verify a logic-voltage drop resets the 500 ms stability timer.
 */
void test_node3_power_sequence_voltage_drop_resets_timer(void) {
    setup();

    power_sequence_init();
    power_sequence_step();

    mock_bsp_set_logic_voltage_mv(5001U);

    /* Start stability timer. */
    power_sequence_step();

    mock_bsp_advance_time_ms(300U);
    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_WAIT_LOGIC_STABLE);

    /*
     * Logic rail falls back to threshold.
     * Stability timer must reset.
     */
    mock_bsp_set_logic_voltage_mv(5000U);

    power_sequence_step();

    /*
     * Rail becomes good again.
     */
    mock_bsp_set_logic_voltage_mv(5001U);

    power_sequence_step();

    /*
     * 499 ms after the new valid sample is still insufficient.
     */
    mock_bsp_advance_time_ms(499U);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_WAIT_LOGIC_STABLE);

    mock_bsp_advance_time_ms(1U);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_DIAGNOSTICS);

    printf("[PASS] test_node3_power_sequence_voltage_drop_resets_timer\n");
}

/**
 * Helper: advance state machine to PWR_SEQ_STAGGER_ENABLE.
 */
static void advance_to_stagger_enable(void) {
    mock_bsp_set_logic_voltage_mv(5001U);
    mock_bsp_set_pcb_temperature_c(25.0f);
    mock_bsp_set_lm74700_status_ok(true);

    power_sequence_init();

    /* INIT -> WAIT_LOGIC_STABLE */
    power_sequence_step();

    /* Start stability timer */
    power_sequence_step();

    mock_bsp_advance_time_ms(500U);

    /* WAIT_LOGIC_STABLE -> DIAGNOSTICS */
    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_DIAGNOSTICS);

    /* DIAGNOSTICS -> STAGGER_ENABLE */
    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_STAGGER_ENABLE);
}

/**
 * Verify bricks enable sequentially at 50 ms intervals.
 */
void test_node3_power_sequence_brick_stagger(void) {
    setup();

    advance_to_stagger_enable();

    /*
     * No bricks enabled immediately.
     */
    for (uint8_t i = 0U; i < TEST_NUM_POWER_BRICKS; i++) {
        assert(!mock_bsp_is_power_brick_enabled(i));
    }

    /*
     * 49 ms: still none.
     */
    mock_bsp_advance_time_ms(49U);
    power_sequence_step();

    assert(!mock_bsp_is_power_brick_enabled(0U));

    /*
     * 50 ms: brick 0.
     */
    mock_bsp_advance_time_ms(1U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(0U));
    assert(!mock_bsp_is_power_brick_enabled(1U));

    /*
     * +50 ms: brick 1.
     */
    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(0U));
    assert(mock_bsp_is_power_brick_enabled(1U));
    assert(!mock_bsp_is_power_brick_enabled(2U));

    /*
     * +50 ms: brick 2.
     */
    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(2U));
    assert(!mock_bsp_is_power_brick_enabled(3U));

    /*
     * +50 ms: brick 3.
     */
    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(3U));

    assert(power_sequence_get_state() == PWR_SEQ_RUNNING);

    printf("[PASS] test_node3_power_sequence_brick_stagger\n");
}

/**
 * Verify Emergency Break path immediately enters FAULT
 * and disables all rails.
 */
void test_node3_power_sequence_emergency_fault(void) {
    setup();

    advance_to_stagger_enable();

    /*
     * Enable the first two bricks so we can verify that the
     * emergency path actually shuts active rails off.
     */
    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(0U));
    assert(mock_bsp_is_power_brick_enabled(1U));

    /*
     * This is the state-machine entry point used when CAN
     * Emergency Break ID 0x001 is received.
     */
    power_sequence_emergency_stop();

    assert(power_sequence_get_state() == PWR_SEQ_FAULT);

    for (uint8_t i = 0U; i < TEST_NUM_POWER_BRICKS; i++) {
        assert(!mock_bsp_is_power_brick_enabled(i));
    }

    printf("[PASS] test_node3_power_sequence_emergency_fault\n");
}

/**
 * Verify high PCB temperature immediately enters FAULT
 * and disables all rails.
 */
void test_node3_power_sequence_high_temp_fault(void) {
    setup();

    advance_to_stagger_enable();

    /*
     * Turn on two rails first.
     */
    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    mock_bsp_advance_time_ms(50U);
    power_sequence_step();

    assert(mock_bsp_is_power_brick_enabled(0U));
    assert(mock_bsp_is_power_brick_enabled(1U));

    /*
     * Power-sequencer threshold is 50 C.
     */
    mock_bsp_set_pcb_temperature_c(60.0f);

    power_sequence_step();

    assert(power_sequence_get_state() == PWR_SEQ_FAULT);

    for (uint8_t i = 0U; i < TEST_NUM_POWER_BRICKS; i++) {
        assert(!mock_bsp_is_power_brick_enabled(i));
    }

    printf("[PASS] test_node3_power_sequence_high_temp_fault\n");
}

int main(void) {
    printf("Running Node 3 (Power Slab) SIL Unit Tests...\n");

    /*
     * Existing application tests.
     */
    test_node3_nominal_telemetry();
    test_node3_overcurrent_fault_alert();
    test_node3_overtemperature_fault_alert();

    /*
     * Power sequencing review tests.
     */
    test_node3_power_sequence_logic_stability();
    test_node3_power_sequence_voltage_drop_resets_timer();
    test_node3_power_sequence_brick_stagger();
    test_node3_power_sequence_emergency_fault();
    test_node3_power_sequence_high_temp_fault();

    printf("All Node 3 SIL Tests Passed Successfully!\n");

    return 0;
}
