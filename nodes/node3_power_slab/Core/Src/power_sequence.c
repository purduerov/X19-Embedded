#include "power_sequence.h"

#include "bsp.h"

#define LOGIC_MIN_VOLTAGE_MV  5000U
#define LOGIC_STABLE_TIME_MS  500U
#define BRICK_STAGGER_TIME_MS 50U
#define PCB_MAX_TEMP_C        50.0f
#define NUM_BRICKS            4U

static power_seq_state_t g_state = PWR_SEQ_INIT;
static uint32_t g_state_start_ms = 0U;
static uint8_t g_next_brick = 0U;
static bool g_logic_stable_started = false;

static void enter_fault_state(void) {
    /**
     * This must be the first action.
     * Emergency shutdown requirement is < 1 ms.
     */
    bsp_power_brick_disable_all();

    g_state = PWR_SEQ_FAULT;
}

void power_sequence_init(void) {
    /**
     * Requirement:
     * all 12 V bricks disabled immediately on startup.
     */
    bsp_power_brick_disable_all();

    g_state = PWR_SEQ_INIT;
    g_state_start_ms = time_get_ms();
    g_next_brick = 0U;
    g_logic_stable_started = false;
}

void power_sequence_step(void) {
    uint32_t now = time_get_ms();

    /*
     * Critical thermal fault must shut down immediately,
     * regardless of the current sequencing state.
     */
    if (bsp_get_pcb_temperature_c() >= PCB_MAX_TEMP_C) {
        enter_fault_state();
        return;
    }

    switch (g_state) {
    case PWR_SEQ_INIT: {
        bsp_power_brick_disable_all();

        g_state = PWR_SEQ_WAIT_LOGIC_STABLE;
        g_state_start_ms = now;
        g_logic_stable_started = false;

        break;
    }

    case PWR_SEQ_WAIT_LOGIC_STABLE: {
        uint32_t logic_voltage_mv = bsp_get_logic_voltage_mv();

        if (logic_voltage_mv > LOGIC_MIN_VOLTAGE_MV) {
            /*
             * Start the 500 ms timer only when the logic rail
             * first rises above 5.0 V.
             */
            if (!g_logic_stable_started) {
                g_logic_stable_started = true;
                g_state_start_ms = now;
            }

            /*
             * The voltage must remain continuously above
             * 5.0 V for at least 500 ms.
             */
            if ((now - g_state_start_ms) >= LOGIC_STABLE_TIME_MS) {
                g_state = PWR_SEQ_DIAGNOSTICS;
            }
        } else {
            /*
             * Voltage dropped to or below 5.0 V.
             * The full 500 ms stability period must restart.
             */
            g_logic_stable_started = false;
        }

        break;
    }

    case PWR_SEQ_DIAGNOSTICS: {
        bool diode_ok = bsp_lm74700_status_ok();

        if (!diode_ok) {
            enter_fault_state();
            break;
        }

        g_next_brick = 0U;
        g_state_start_ms = now;
        g_state = PWR_SEQ_STAGGER_ENABLE;

        break;
    }

    case PWR_SEQ_STAGGER_ENABLE: {
        /*
         * Enable one brick every 50 ms.
         */
        if ((now - g_state_start_ms) >= BRICK_STAGGER_TIME_MS) {
            bsp_power_brick_enable(g_next_brick);

            g_next_brick++;
            g_state_start_ms = now;

            if (g_next_brick >= NUM_BRICKS) {
                g_state = PWR_SEQ_RUNNING;
            }
        }

        break;
    }

    case PWR_SEQ_RUNNING: {
        /*
         * Continue monitoring critical safety conditions.
         */
        if (!bsp_lm74700_status_ok()) {
            enter_fault_state();
            break;
        }

        break;
    }

    case PWR_SEQ_FAULT: {
        /*
         * Stay latched off.
         */
        bsp_power_brick_disable_all();

        break;
    }

    default: {
        enter_fault_state();
        break;
    }
    }
}

void power_sequence_emergency_stop(void) {
    /**
     * Called when CAN ID 0x001 Emergency Break is received.
     */
    enter_fault_state();
}

power_seq_state_t power_sequence_get_state(void) {
    return g_state;
}
