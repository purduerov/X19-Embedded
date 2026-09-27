/**
 * @file app.c
 * @brief Application Layer Logic for Testing & R&D Node.
 * Contains 100% pure application logic with ZERO vendor ST HAL calls.
 */

#include "app.h"
#include "bench_config.h"
#include "bsp.h"
#include "can_interface.h"
#include <stdio.h>

#if defined(BENCH_APP_NODE1_PI_SHIELD)
#include "../../node1_pi_shield/Core/Inc/app.h"
#elif defined(BENCH_APP_NODE2_CONTROL)
#include "../../node2_control_board/Core/Inc/app.h"
#elif defined(BENCH_APP_NODE3_POWER)
#include "../../node3_power_slab/Core/Inc/app.h"
#endif

void app_main(void) {
    /* 1. Initialize Board Support Package (clocks, CAN filters, unbuffered printf) */
    bsp_init();

#if defined(BENCH_APP_NODE1_PI_SHIELD)
    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 1 (Pi Shield) on NUCLEO-F411RE   \r\n");
    printf("======================================================\r\n");
    bsp_i2c_init();
    node1_app_init();
    while (1) {
        node1_app_step();
    }

#elif defined(BENCH_APP_NODE2_CONTROL)
    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 2 (Control Board) on NUCLEO-F411\r\n");
    printf("======================================================\r\n");
    node2_app_init();
    while (1) {
        node2_app_step();
    }

#elif defined(BENCH_APP_NODE3_POWER)
    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 3 (Power Slab) on NUCLEO-F411   \r\n");
    printf("======================================================\r\n");
    node3_app_init();
    while (1) {
        node3_app_step();
    }

#else /* BENCH_APP_RND_SCANNER */
    printf("\r\n======================================================\r\n");
    printf("  Bench Running: R&D Testing / I2C Bus Scanner        \r\n");
    printf("======================================================\r\n");

    /* 2. Initialize I2C on pins PB8 (D15 - SCL) and PB9 (D14 - SDA) */
    bsp_i2c_init();

    /* 3. Run full I2C address scan across 0x08 to 0x77 */
    bsp_i2c_scan();

    uint32_t last_tx_time = 0;
    uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    while (1) {
        /* 2. Transmit CAN heartbeat frame to Raspberry Pi every 500 ms */
        if (time_get_ms() - last_tx_time >= 500) {
            last_tx_time = time_get_ms();
            tx_data[0]++;
            can_send(0x123, tx_data, 8);
            led_toggle(); /* Blink Green LED (1 Hz) */
        }

        /* 3. Poll for incoming CAN messages from Raspberry Pi */
        uint32_t rx_id = 0;
        uint8_t rx_data[8] = {0};
        uint8_t rx_len = 0;

        if (can_receive(&rx_id, rx_data, &rx_len)) {
            printf("\r\n>>> [RX] ID: 0x%03lX | DLC: %u | Data: ", rx_id, rx_len);
            for (uint8_t i = 0; i < rx_len; i++) {
                printf("%02X ", rx_data[i]);
            }
            printf("\r\n");

            /* Immediate Echo back to Pi: CAN ID 0x200 with byte 0 incremented */
            rx_data[0] += 1;
            can_send(0x200, rx_data, rx_len);
        }

        delay_ms(5);
    }
#endif
}
