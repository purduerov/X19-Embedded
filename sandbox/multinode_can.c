/**
 * @file multinode_can.c
 * @brief Multi-Node CAN FD Ping-Pong & Telemetry Bench Test Firmware.
 *
 * Designed to run seamlessly across heterogeneous hardware (NUCLEO-G474RE,
 * NUCLEO-F411RE, or custom STM32 PCBs) without modifying application code.
 *
 * Roles supported:
 *   - NODE_ROLE 1: Node 1 Pi Shield (Env/Leak Telemetry 0x210 @ 10 Hz)
 *   - NODE_ROLE 2: Node 2 Control Board (Nav Telemetry 0x200 @ 50 Hz, Thruster RX 0x100)
 *   - NODE_ROLE 3: Node 3 Power Slab (Power Telemetry 0x300 @ 20 Hz)
 *
 * Build & Run:
 *   python rov.py sandbox -f sandbox/multinode_can.c -b g474 -s <PROBE_SN>
 *   python rov.py sandbox -f sandbox/multinode_can.c -b f411 -s <PROBE_SN>
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include "rov_can_protocol.h"
#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/* Node Role Configuration                                                   */
/* ========================================================================== */

/* Default node role selection based on target MCU or compile flag */
#ifndef NODE_ROLE
    #if defined(STM32G474xx)
        #define NODE_ROLE 2   /* G474 defaults to Node 2: Control Board */
    #elif defined(STM32F411xE) || defined(STM32F446xx)
        #define NODE_ROLE 1   /* F4 dev boards default to Node 1: Pi Shield */
    #else
        #define NODE_ROLE 3   /* Custom / secondary boards default to Node 3 */
    #endif
#endif

/* ========================================================================== */
/* Role Parameters                                                           */
/* ========================================================================== */

#if (NODE_ROLE == 1)
    #define ROLE_NAME            "Node 1: Pi Shield"
    #define ROLE_TX_CAN_ID       ROV_CAN_ID_ENV_TELEMETRY  /* 0x210 */
    #define ROLE_TX_PERIOD_MS    100                       /* 10 Hz */
#elif (NODE_ROLE == 2)
    #define ROLE_NAME            "Node 2: Control Board"
    #define ROLE_TX_CAN_ID       ROV_CAN_ID_NAV_TELEMETRY  /* 0x200 */
    #define ROLE_TX_PERIOD_MS    20                        /* 50 Hz */
#elif (NODE_ROLE == 3)
    #define ROLE_NAME            "Node 3: Power Slab"
    #define ROLE_TX_CAN_ID       ROV_CAN_ID_POWER_TELEMETRY /* 0x300 */
    #define ROLE_TX_PERIOD_MS    50                        /* 20 Hz */
#else
    #error "Invalid NODE_ROLE defined! Must be 1, 2, or 3."
#endif

/* ========================================================================== */
/* Application Entry Point                                                   */
/* ========================================================================== */

void app_main(void) {
    bsp_init();

    printf("\r\n");
    printf("================================================================\r\n");
    printf("  Purdue ROV X19 Multi-Node CAN FD Test Firmware\r\n");
    printf("  Active Role: %s (Role ID: %d)\r\n", ROLE_NAME, NODE_ROLE);
    printf("  TX CAN ID  : 0x%03X (Period: %d ms)\r\n", ROLE_TX_CAN_ID, ROLE_TX_PERIOD_MS);
    printf("================================================================\r\n\r\n");

    if (!can_init()) {
        printf("[FATAL] FDCAN hardware initialization failed!\r\n");
        while (1) {
            led_toggle();
            delay_ms(100);
        }
    }
    printf("[INIT] FDCAN online (1.0 Mbps nominal / 5.0 Mbps data phase with BRS).\r\n");

    /* Statistics tracking */
    uint32_t last_tx_ms = 0;
    uint32_t last_stats_ms = 0;
    uint32_t tx_count = 0;
    uint32_t rx_count = 0;
    uint32_t emergency_count = 0;
    uint32_t counter = 0;

    while (1) {
        uint32_t now = time_get_ms();

        /* ------------------------------------------------------------------ */
        /* 1. Inbound CAN Frame Reception                                     */
        /* ------------------------------------------------------------------ */
        uint32_t rx_id = 0;
        uint8_t rx_buf[64] = {0};
        uint8_t rx_len = 0;

        while (can_receive(&rx_id, rx_buf, &rx_len)) {
            rx_count++;
            led_toggle();

            /* Handle Emergency Stop (Priority 0) */
            if (rx_id == ROV_CAN_ID_EMERGENCY_BREAK) {
                emergency_count++;
                printf("[ALERT] Emergency Break (0x001) received from bus!\r\n");
            }
            /* Handle Thruster PWM Command (addressed to Node 2) */
            else if (rx_id == ROV_CAN_ID_THRUSTER_CMD) {
                if (rx_len >= 16) {
                    uint16_t pwm0 = (uint16_t)(rx_buf[0] | (rx_buf[1] << 8));
                    uint16_t pwm1 = (uint16_t)(rx_buf[2] | (rx_buf[3] << 8));
                    printf("[RX 0x100] Thruster Command | CH0: %u us, CH1: %u us\r\n", pwm0, pwm1);
                }
            }
            /* Handle Environmental / Leak Telemetry from Node 1 */
            else if (rx_id == ROV_CAN_ID_ENV_TELEMETRY) {
                printf("[RX 0x210] Env Telemetry from Node 1 (%u bytes)\r\n", rx_len);
            }
            /* Handle Navigation Telemetry from Node 2 */
            else if (rx_id == ROV_CAN_ID_NAV_TELEMETRY) {
                printf("[RX 0x200] Nav Telemetry from Node 2 (%u bytes)\r\n", rx_len);
            }
            /* Handle Power Telemetry from Node 3 */
            else if (rx_id == ROV_CAN_ID_POWER_TELEMETRY) {
                printf("[RX 0x300] Power Telemetry from Node 3 (%u bytes)\r\n", rx_len);
            }
            /* Generic frame display */
            else {
                printf("[RX 0x%03lX] DLC: %u bytes\r\n", rx_id, rx_len);
            }
        }

        /* Check for bus-off error recovery */
        if (can_is_bus_off()) {
            printf("[WARN] CAN controller entered Bus-Off state! Attempting recovery...\r\n");
            can_recover();
        }

        /* ------------------------------------------------------------------ */
        /* 2. Periodic Outbound Telemetry Transmission                        */
        /* ------------------------------------------------------------------ */
        if ((now - last_tx_ms) >= (uint32_t)ROLE_TX_PERIOD_MS) {
            last_tx_ms = now;

            uint8_t tx_buf[64] = {0};
            uint8_t tx_len = 0;

#if (NODE_ROLE == 1)
            /* Construct Mock Environmental Telemetry: 17 bytes (packed) */
            /* temperature: 24.5C, humidity: 35.0%, pressure: 1013.25 hPa, leak_mask: 0 */
            float temp_c = 24.5f;
            float humidity_pct = 35.0f;
            float pressure_hpa = 1013.25f;
            uint8_t leak_bitmask = 0x00;
            uint16_t v_rail_mv = 5020;  /* 5.02V rail */
            uint16_t i_rail_ma = 480;   /* 480mA current */

            memcpy(&tx_buf[0], &temp_c, sizeof(float));
            memcpy(&tx_buf[4], &humidity_pct, sizeof(float));
            memcpy(&tx_buf[8], &pressure_hpa, sizeof(float));
            tx_buf[12] = leak_bitmask;
            memcpy(&tx_buf[13], &v_rail_mv, sizeof(uint16_t));
            memcpy(&tx_buf[15], &i_rail_ma, sizeof(uint16_t));
            tx_len = 17;

#elif (NODE_ROLE == 2)
            /* Construct Mock Navigation Telemetry: 33 bytes (packed) */
            /* 8x float (accel x/y/z, gyro x/y/z, depth, heading) + 1 byte status */
            float dummy_floats[8] = {
                0.0f, 0.0f, 9.81f,       /* Accel X, Y, Z */
                0.01f, -0.02f, 0.00f,    /* Gyro X, Y, Z */
                1.25f,                   /* Depth in meters */
                180.5f                   /* Heading in degrees */
            };
            uint8_t status_flags = 0x01; /* Sensor health OK */

            memcpy(&tx_buf[0], dummy_floats, sizeof(dummy_floats));
            tx_buf[32] = status_flags;
            tx_len = 33;

#elif (NODE_ROLE == 3)
            /* Construct Mock Power Telemetry: 18 bytes (packed) */
            /* 48V, 12V, 5V voltages, total current, and eFuse status mask */
            uint16_t v_48v_mv = 48200;
            uint16_t v_12v_mv = 12050;
            uint16_t v_5v_mv  = 5010;
            uint16_t i_tot_ma = 12500;
            uint16_t efuse_status = 0x00FF;

            memcpy(&tx_buf[0], &v_48v_mv, sizeof(uint16_t));
            memcpy(&tx_buf[2], &v_12v_mv, sizeof(uint16_t));
            memcpy(&tx_buf[4], &v_5v_mv, sizeof(uint16_t));
            memcpy(&tx_buf[6], &i_tot_ma, sizeof(uint16_t));
            memcpy(&tx_buf[8], &efuse_status, sizeof(uint16_t));
            tx_len = 10;
#endif

            if (can_send(ROLE_TX_CAN_ID, tx_buf, tx_len)) {
                tx_count++;
            }
            counter++;
        }

        /* ------------------------------------------------------------------ */
        /* 3. 1 Hz Serial Diagnostics Status Printout                         */
        /* ------------------------------------------------------------------ */
        if ((now - last_stats_ms) >= 1000) {
            last_stats_ms = now;
            printf("[%s] Up: %lu s | TX: %lu pkts | RX: %lu pkts | E-Stop: %lu\r\n",
                   ROLE_NAME, (unsigned long)(now / 1000),
                   (unsigned long)tx_count, (unsigned long)rx_count,
                   (unsigned long)emergency_count);
        }
    }
}
