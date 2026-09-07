/**
 * @file mock_can.h
 * @brief In-memory Multi-Node CAN FD Bus Simulator for Host-Native Testing.
 * @organization Purdue ROV
 */

#ifndef MOCK_CAN_H
#define MOCK_CAN_H

#include "can_interface.h"
#include "rov_types.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOCK_CAN_QUEUE_SIZE     32
#define MOCK_CAN_HISTORY_SIZE   128
#define MOCK_CAN_MAX_FRAME_SIZE 64
#define MOCK_CAN_MAX_NODES      8

typedef struct {
    uint32_t id;
    uint8_t data[MOCK_CAN_MAX_FRAME_SIZE];
    uint8_t len;
    rov_node_id_t sender;
    uint32_t timestamp_ms;
} mock_can_frame_t;

/**
 * @brief Reset the virtual CAN bus state, all node queues, and history.
 */
void mock_can_reset(void);

/**
 * @brief Set which node context is currently executing (Node 1, Node 2, Node 3, or Pi Core).
 */
void mock_can_set_current_node(rov_node_id_t node_id);

/**
 * @brief Get the currently active node context.
 */
rov_node_id_t mock_can_get_current_node(void);

/**
 * @brief Inject a frame into the RX queue of the currently active node.
 */
bool mock_can_inject_rx(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Inject a frame into a specific node's RX queue.
 */
bool mock_can_inject_node_rx(rov_node_id_t target_node, uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Total number of frames transmitted across the virtual bus.
 */
uint32_t mock_can_get_tx_count(void);

/**
 * @brief Get the most recent frame transmitted on the bus.
 */
bool mock_can_get_last_tx(uint32_t *id, uint8_t *data, uint8_t *len);

/**
 * @brief Find the latest frame matching a specific CAN ID from bus history.
 */
bool mock_can_find_latest_tx(uint32_t target_id, uint8_t *data, uint8_t *len);

/**
 * @brief Count how many frames with target_id have been transmitted.
 */
uint32_t mock_can_count_tx_by_id(uint32_t target_id);

/**
 * @brief Enable or disable Bus-Off state for fault injection.
 */
void mock_can_set_bus_off(bool bus_off);

/**
 * @brief Enable or disable TX mailbox exhaustion failure.
 */
void mock_can_set_tx_fail(bool fail);

/**
 * @brief Set the number of consecutive frames to drop on transmission.
 */
void mock_can_set_drop_count(uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_CAN_H */
