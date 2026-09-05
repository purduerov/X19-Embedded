/**
 * @file app.c
 * @brief Application Layer Logic for Testing & R&D Node.
 * Contains 100% pure application logic with ZERO vendor ST HAL calls.
 */

#include "app.h"
#include "bsp.h"
#include "can_interface.h"
#include <stdio.h>

void app_main(void) {
    /* 1. Initialize Board Support Package (clocks, CAN filters, unbuffered printf) */
    bsp_init();

    printf("\r\n=== STM32F446RE Application Started (Clean BSP Architecture) ===\r\n");

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
}
