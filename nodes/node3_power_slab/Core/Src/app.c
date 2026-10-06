/**
 * @file app.c
 * @brief Node 3 (Power Slab) Application Layer Logic.
 *
 * Handles PMBus telemetry for 5 converter bricks
 * (4x 12V 300W + 1x 5.2V 50W),
 * PCB copper thermal monitoring, LM74700 diode status,
 * 20 Hz Power Telemetry CAN stream,
 * and 0x005 eFuse Fault Alert broadcast upon overcurrent / overtemp trip.
 *
 * Pure application logic with zero vendor ST HAL calls.
 *
 * @organization Purdue ROV
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include "pmbus_brick.h"
#include "power_sequence.h"
#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Thermal protection configuration                                            */
/* -------------------------------------------------------------------------- */

#define NUM_12V_BRICKS              4U
#define NUM_TOTAL_BRICKS            5U

#define THERMAL_SAMPLE_PERIOD_MS    200U

#define THERMAL_WARNING_TEMP_C      70.0f
#define THERMAL_SHUTDOWN_TEMP_C     85.0f
#define THERMAL_RECOVERY_TEMP_C     65.0f

/*
 * Existing status bit:
 *   bit 0 = fault
 *
 * New bit:
 *   bit 1 = thermal warning
 *
 * If rov_can_protocol.h already defines an official thermal-warning bit,
 * use that definition instead.
 */
#define POWER_STATUS_FAULT           0x0001U
#define POWER_STATUS_THERMAL_WARNING 0x0002U

typedef enum {
    THERMAL_NORMAL = 0,
    THERMAL_WARNING,
    THERMAL_SHUTDOWN
} thermal_state_t;

/* -------------------------------------------------------------------------- */
/* Application state                                                           */
/* -------------------------------------------------------------------------- */

static rov_safety_state_t g_safety_state;
static rov_power_telemetry_t g_power_telemetry;

static pmbus_brick_dev_t g_pmbus_bricks[NUM_TOTAL_BRICKS];

static uint32_t g_last_power_time = 0U;
static uint32_t g_last_pmbus_poll_time = 0U;
static uint32_t g_last_thermal_time = 0U;

static uint8_t g_next_pmbus_brick = 0U;

/* Thermal state for each of the four 12 V bricks. */
static thermal_state_t g_brick_thermal_state[NUM_12V_BRICKS];
static bool g_brick_thermal_latched[NUM_12V_BRICKS];

/* PCB copper temperature state. */
static thermal_state_t g_pcb_thermal_state = THERMAL_NORMAL;
static bool g_pcb_thermal_latched = false;

/*
 * Set when pilot explicitly requests thermal recovery.
 *
 * A latched thermal fault will only clear when:
 *
 *     temperature < 65 C
 *              AND
 *     thermal clear requested
 */
static bool g_thermal_clear_requested = false;

/* -------------------------------------------------------------------------- */
/* Helper functions                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Broadcast a priority 0 eFuse / power fault alert.
 *
 * @param reason Fault reason code.
 * @param source Source brick index, or 0xFF for board-wide fault.
 */
static void send_fault_alert(uint8_t reason, uint8_t source) {
    uint8_t alert[8] = {
        0xEF,
        reason,
        (uint8_t)(g_power_telemetry.status_flags & 0xFFU),
        source,
        0U,
        0U,
        0U,
        0U
    };

    can_send(
        ROV_CAN_ID_EFUSE_FAULT_ALERT,
        alert,
        sizeof(alert)
    );
}

/**
 * @brief Return true if any thermal shutdown remains latched.
 */
static bool any_thermal_fault_latched(void) {
    if (g_pcb_thermal_latched) {
        return true;
    }

    for (uint8_t i = 0U; i < NUM_12V_BRICKS; i++) {
        if (g_brick_thermal_latched[i]) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Run the 5 Hz thermal protection state machine.
 */
static void thermal_protection_step(void) {
    float pcb_temp_c = bsp_get_pcb_temperature_c();
    bool thermal_warning_present = false;

    /* ---------------------------------------------------------------------- */
    /* PCB copper temperature                                                 */
    /* ---------------------------------------------------------------------- */

    if (!g_pcb_thermal_latched) {

        if (pcb_temp_c >= THERMAL_SHUTDOWN_TEMP_C) {

            /*
             * PCB sensor represents the whole power slab, therefore a
             * critical PCB temperature shuts down all four 12 V bricks.
             */
            g_pcb_thermal_state = THERMAL_SHUTDOWN;
            g_pcb_thermal_latched = true;

            bsp_power_brick_disable_all();

            g_power_telemetry.status_flags |= POWER_STATUS_FAULT;
            g_safety_state.overtemperature_tripped = true;

            /*
             * reason = 0x02 -> thermal fault
             * source = 0xFF -> board / PCB-wide fault
             */
            send_fault_alert(0x02U, 0xFFU);

        } else if (pcb_temp_c >= THERMAL_WARNING_TEMP_C) {

            g_pcb_thermal_state = THERMAL_WARNING;
            thermal_warning_present = true;

        } else {

            g_pcb_thermal_state = THERMAL_NORMAL;
        }

    } else {

        /*
         * PCB fault is latched.
         *
         * Cooling below 65 C alone is NOT sufficient.
         * Pilot must also explicitly issue thermal clear.
         */
        if ((pcb_temp_c < THERMAL_RECOVERY_TEMP_C) &&
            g_thermal_clear_requested) {

            g_pcb_thermal_latched = false;
            g_pcb_thermal_state = THERMAL_NORMAL;

            /*
             * Allow the four bricks to operate again.
             *
             * If your team wants recovery to go back through the staggered
             * power sequence instead, we can change this later.
             */
            for (uint8_t i = 0U; i < NUM_12V_BRICKS; i++) {

                /*
                 * Do not re-enable a brick that still has its own
                 * individual thermal fault latched.
                 */
                if (!g_brick_thermal_latched[i]) {
                    bsp_power_brick_enable(i);
                }
            }
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Individual converter temperatures                                      */
    /* ---------------------------------------------------------------------- */

    for (uint8_t i = 0U; i < NUM_12V_BRICKS; i++) {

        float brick_temp_c = g_pmbus_bricks[i].temperature_c;

        if (!g_brick_thermal_latched[i]) {

            if (brick_temp_c >= THERMAL_SHUTDOWN_TEMP_C) {

                g_brick_thermal_state[i] = THERMAL_SHUTDOWN;
                g_brick_thermal_latched[i] = true;

                /*
                 * Requirement:
                 * disable only the affected 12 V converter.
                 */
                bsp_power_brick_disable(i);

                g_power_telemetry.status_flags |= POWER_STATUS_FAULT;
                g_safety_state.overtemperature_tripped = true;

                send_fault_alert(0x02U, i);

            } else if (brick_temp_c >= THERMAL_WARNING_TEMP_C) {

                g_brick_thermal_state[i] = THERMAL_WARNING;
                thermal_warning_present = true;

            } else {

                g_brick_thermal_state[i] = THERMAL_NORMAL;
            }

        } else {

            /*
             * Thermal shutdown stays latched until:
             *
             * brick temperature < 65 C
             * AND
             * pilot explicitly clears the fault.
             */
            if ((brick_temp_c < THERMAL_RECOVERY_TEMP_C) &&
                g_thermal_clear_requested &&
                !g_pcb_thermal_latched) {

                g_brick_thermal_latched[i] = false;
                g_brick_thermal_state[i] = THERMAL_NORMAL;

                bsp_power_brick_enable(i);
            }
        }

        if (g_brick_thermal_state[i] == THERMAL_WARNING) {
            thermal_warning_present = true;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Telemetry flags                                                        */
    /* ---------------------------------------------------------------------- */

    if (thermal_warning_present) {
        g_power_telemetry.status_flags |= POWER_STATUS_THERMAL_WARNING;
    } else {
        g_power_telemetry.status_flags &= ~POWER_STATUS_THERMAL_WARNING;
    }

    /*
     * Clear overtemperature state only after every thermal latch has cleared.
     */
    if (!any_thermal_fault_latched()) {
        g_safety_state.overtemperature_tripped = false;
    }

    /*
     * A clear request is one-shot.
     *
     * If a device is still >= 65 C, the pilot must issue another clear
     * command later after it has cooled.
     */
    g_thermal_clear_requested = false;
}

/* -------------------------------------------------------------------------- */
/* Public Application API                                                      */
/* -------------------------------------------------------------------------- */

void node3_app_init(void) {
    bsp_init();
    power_sequence_init();
    rov_safety_init(&g_safety_state);

    for (uint8_t i = 0U; i < NUM_TOTAL_BRICKS; i++) {
        pmbus_brick_init(
            &g_pmbus_bricks[i],
            (uint8_t)(0x40U + i)
        );
    }

    memset(
        &g_power_telemetry,
        0,
        sizeof(g_power_telemetry)
    );

    g_power_telemetry.tether_voltage_mv = 48000U;
    g_power_telemetry.v5_voltage_mv = 5200U;

    g_last_power_time = 0U;
    g_last_pmbus_poll_time = 0U;
    g_last_thermal_time = 0U;

    g_next_pmbus_brick = 0U;

    g_pcb_thermal_state = THERMAL_NORMAL;
    g_pcb_thermal_latched = false;

    g_thermal_clear_requested = false;

    for (uint8_t i = 0U; i < NUM_12V_BRICKS; i++) {
        g_brick_thermal_state[i] = THERMAL_NORMAL;
        g_brick_thermal_latched[i] = false;
    }
}

/**
 * @brief Request clearing latched thermal shutdowns.
 *
 * A shutdown only clears if the affected temperature has also fallen
 * below 65 C.
 */
void node3_app_request_thermal_clear(void) {
    g_thermal_clear_requested = true;
}

void node3_app_step(void) {
    power_sequence_step();

    uint32_t current_time = time_get_ms();

    /* ---------------------------------------------------------------------- */
    /* PMBus polling - one brick every 10 ms                                  */
    /* ---------------------------------------------------------------------- */

    if ((current_time - g_last_pmbus_poll_time) >= 10U) {

        g_last_pmbus_poll_time = current_time;

        (void)pmbus_brick_read_telemetry(
            &g_pmbus_bricks[g_next_pmbus_brick]
        );

        g_next_pmbus_brick++;

        if (g_next_pmbus_brick >= NUM_TOTAL_BRICKS) {
            g_next_pmbus_brick = 0U;
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Thermal protection - 5 Hz / every 200 ms                               */
    /* ---------------------------------------------------------------------- */

    if ((current_time - g_last_thermal_time) >= THERMAL_SAMPLE_PERIOD_MS) {

        g_last_thermal_time = current_time;

        thermal_protection_step();
    }

    /* ---------------------------------------------------------------------- */
    /* 20 Hz Power Telemetry / overcurrent protection                         */
    /* ---------------------------------------------------------------------- */

    if ((current_time - g_last_power_time) >=
        (1000U / ROV_POWER_TELEMETRY_FREQ_HZ)) {

        g_last_power_time = current_time;

        float total_tether_power_w = 0.0f;
        bool overcurrent_fault_detected = false;

        /*
         * This is the actual TMP1075 PCB copper temperature.
         */
        float pcb_temp_c = bsp_get_pcb_temperature_c();

        for (uint8_t i = 0U; i < NUM_TOTAL_BRICKS; i++) {

            /*
             * Do NOT read PMBus again here.
             *
             * The 10 ms round-robin above owns PMBus polling.
             * We simply use the most recently cached values.
             */

            if (i == 4U) {

                /* Brick 4 = 5.2 V logic converter. */
                g_power_telemetry.v5_voltage_mv =
                    (uint16_t)(
                        g_pmbus_bricks[i].output_voltage_v * 1000.0f
                    );

                g_power_telemetry.v5_current_ma =
                    (uint16_t)(
                        g_pmbus_bricks[i].output_current_a * 1000.0f
                    );

            } else {

                /* Bricks 0..3 = four 12 V converters. */
                g_power_telemetry.v12_current_ma[i] =
                    (uint16_t)(
                        g_pmbus_bricks[i].output_current_a * 1000.0f
                    );

                /*
                 * Overcurrent protection:
                 * 25 A maximum per 12 V converter.
                 */
                if (g_pmbus_bricks[i].output_current_a >
                    ROV_BRICK_MAX_CURRENT_A) {

                    overcurrent_fault_detected = true;
                }
            }

            total_tether_power_w +=
                g_pmbus_bricks[i].output_voltage_v *
                g_pmbus_bricks[i].output_current_a;
        }

        float total_tether_current_a =
            total_tether_power_w *
            (1.0f / ROV_TETHER_NOMINAL_VOLTAGE_V);

        g_power_telemetry.tether_voltage_mv =
            (uint16_t)(
                ROV_TETHER_NOMINAL_VOLTAGE_V * 1000.0f
            );

        g_power_telemetry.tether_current_ma =
            (uint16_t)(
                total_tether_current_a * 1000.0f
            );

        /*
         * Telemetry field now reports the actual PCB sensor temperature,
         * not the hottest converter temperature.
         */
        g_power_telemetry.pcb_temp_c =
            (int16_t)(pcb_temp_c * 10.0f);

        /* ------------------------------------------------------------------ */
        /* Overcurrent handling                                               */
        /* ------------------------------------------------------------------ */

        if (overcurrent_fault_detected) {

            /*
             * Existing behavior remains:
             * overcurrent is treated as a global emergency fault.
             */
            power_sequence_emergency_stop();

            g_power_telemetry.status_flags |= POWER_STATUS_FAULT;

            uint8_t alert[8] = {
                0xEF,
                0x01U,
                (uint8_t)(
                    g_power_telemetry.status_flags & 0xFFU
                ),
                0U,
                0U,
                0U,
                0U,
                0U
            };

            can_send(
                ROV_CAN_ID_EFUSE_FAULT_ALERT,
                alert,
                sizeof(alert)
            );
        }

        /* ------------------------------------------------------------------ */
        /* Stream CAN ID 0x300 Power Telemetry                                */
        /* ------------------------------------------------------------------ */

        uint8_t tx_buf[64];
        size_t packed_len = 0U;

        if (rov_can_pack_power_telemetry(
                &g_power_telemetry,
                tx_buf,
                sizeof(tx_buf),
                &packed_len) == ROV_OK) {

            can_send(
                ROV_CAN_ID_POWER_TELEMETRY,
                tx_buf,
                (uint8_t)packed_len
            );
        }

        led_toggle();
    }
}

#ifndef ROV_UNIT_TEST

void app_main(void) {
    node3_app_init();

    while (1) {
        node3_app_step();
    }
}

#endif
