/**
 * @file can_interface.h
 * @brief Unified CAN Transport Abstraction Layer (HAL Wrapper / BSP).
 * @author Purdue ROV Embedded Team
 *
 * Standardized hardware abstraction layer. All node application logic
 * (main.c, control loops, state machines) must invoke these functions
 * instead of direct vendor ST HAL calls (HAL_CAN_... / HAL_FDCAN_...).
 */

#ifndef X19_CAN_INTERFACE_H
#define X19_CAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the CAN hardware peripheral, filters, and bit timings.
 * @return true if initialization and start succeeded, false otherwise.
 */
bool can_init(void);

/**
 * @brief Transmit a CAN frame (non-blocking).
 *
 * Automatically places the message into the highest-priority available hardware
 * transmit mailbox/FIFO. Handles transient mailbox exhaustion without blocking.
 *
 * @param id 11-bit standard CAN identifier (0x000 - 0x7FF) or 29-bit extended ID.
 * @param data Pointer to payload data buffer.
 * @param len Payload length (0 to 8 bytes for classic CAN, up to 64 for CAN FD).
 * @return true if queued into hardware mailbox, false if mailbox full or error.
 */
bool can_send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Immediately submit a safety-critical CAN frame.
 *
 * This transmission path is reserved for time-critical safety messages such
 * as ROV_CAN_ID_EMERGENCY_BREAK (0x001).
 *
 * The hardware-specific implementation must bypass normal software telemetry
 * queues and submit the frame directly to the FDCAN hardware transmit resource.
 * The function must be non-blocking and safe to call from the leak EXTI path.
 *
 * @param id 11-bit standard CAN identifier.
 * @param data Pointer to payload data buffer.
 * @param len Payload length.
 * @return true if submitted to the hardware transmit resource, false otherwise.
 */
bool can_send_emergency(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Retrieve an incoming CAN frame from the RX FIFO (polling).
 *
 * If a message is waiting in the hardware RX FIFO, copies the header and data
 * out, releases the FIFO element, and returns true.
 *
 * @param[out] id Pointer to store the received CAN ID.
 * @param[out] data Buffer to store received payload (must be at least 8 bytes, or 64 for CAN FD).
 * @param[out] len Pointer to store the received payload length (DLC).
 * @return true if a message was successfully retrieved, false if RX FIFO is empty.
 */
bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len);

/**
 * @brief Check if the CAN controller is currently in Bus-Off state.
 * @return true if Bus-Off is active, false if bus is healthy.
 */
bool can_is_bus_off(void);

/**
 * @brief Manually initiate bus recovery if trapped in Bus-Off state.
 */
void can_recover(void);

/**
 * @brief Diagnostic information from hardware CAN controller.
 */
typedef struct {
    uint8_t tx_error_count;       /**< Transmit Error Counter (TEC) */
    uint8_t rx_error_count;       /**< Receive Error Counter (REC) */
    uint8_t last_error_code;      /**< Last Error Code (0=None, 1=Stuff, 2=Form, 3=Ack, 4=Bit1, 5=Bit0, 6=CRC) */
    uint8_t data_last_error_code; /**< Data Phase Last Error Code */
    uint8_t tx_fifo_free_level;   /**< Available TX FIFO slots */
    bool bus_off;                 /**< Controller is in Bus-Off state */
    bool error_passive;           /**< Controller is in Error-Passive state */
    bool error_warning;           /**< Controller is in Error-Warning state */
} can_diagnostics_t;

/**
 * @brief Retrieve hardware CAN diagnostic status.
 * @param[out] diag Pointer to structure receiving diagnostics.

 * * @return true if diagnostics were populated, false if unavailable.
 */
bool can_get_diagnostics(can_diagnostics_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* X19_CAN_INTERFACE_H */
