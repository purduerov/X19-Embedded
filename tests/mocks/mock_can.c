/**
 * @file mock_can.c
 * @brief In-memory Multi-Node CAN FD Bus Simulator Implementation.
 * @organization Purdue ROV
 */

#include "mock_can.h"
#include "bsp.h"
#include <string.h>

typedef struct {
    mock_can_frame_t queue[MOCK_CAN_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    bool is_bus_off;
    bool tx_fail;
} mock_node_bus_state_t;

static mock_node_bus_state_t g_nodes[MOCK_CAN_MAX_NODES];
static mock_can_frame_t g_bus_history[MOCK_CAN_HISTORY_SIZE];
static uint32_t g_total_tx_count = 0;
static uint32_t g_drop_counter = 0;
static x19_node_id_t g_current_node = X19_NODE_CONTROL_BOARD;

static uint8_t node_to_index(x19_node_id_t node) {
    switch (node) {
    case X19_NODE_PI_CORE:
        return 0;
    case X19_NODE_PI_SHIELD:
        return 1;
    case X19_NODE_CONTROL_BOARD:
        return 2;
    case X19_NODE_POWER_SLAB:
        return 3;
    case X19_NODE_USB_HUB:
        return 4;
    default:
        return 0;
    }
}

void mock_can_reset(void) {
    memset(g_nodes, 0, sizeof(g_nodes));
    memset(g_bus_history, 0, sizeof(g_bus_history));
    g_total_tx_count = 0;
    g_drop_counter = 0;
    g_current_node = X19_NODE_CONTROL_BOARD;
}

void mock_can_set_current_node(x19_node_id_t node_id) {
    g_current_node = node_id;
}

x19_node_id_t mock_can_get_current_node(void) {
    return g_current_node;
}

static bool enqueue_node_frame(uint8_t node_idx, const mock_can_frame_t *frame) {
    mock_node_bus_state_t *st = &g_nodes[node_idx];
    if (st->count >= MOCK_CAN_QUEUE_SIZE) {
        return false; /* FIFO full */
    }
    st->queue[st->tail] = *frame;
    st->tail = (st->tail + 1) % MOCK_CAN_QUEUE_SIZE;
    st->count++;
    return true;
}

bool mock_can_inject_node_rx(x19_node_id_t target_node, uint32_t id, const uint8_t *data, uint8_t len) {
    if (len > MOCK_CAN_MAX_FRAME_SIZE) {
        return false;
    }
    mock_can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.len = len;
    frame.sender = X19_NODE_BROADCAST;
    frame.timestamp_ms = time_get_ms();
    if (data && len > 0) {
        memcpy(frame.data, data, len);
    }
    return enqueue_node_frame(node_to_index(target_node), &frame);
}

bool mock_can_inject_rx(uint32_t id, const uint8_t *data, uint8_t len) {
    return mock_can_inject_node_rx(g_current_node, id, data, len);
}

uint32_t mock_can_get_tx_count(void) {
    return g_total_tx_count;
}

bool mock_can_get_last_tx(uint32_t *id, uint8_t *data, uint8_t *len) {
    if (g_total_tx_count == 0) {
        return false;
    }
    uint32_t idx = (g_total_tx_count - 1) % MOCK_CAN_HISTORY_SIZE;
    if (id)
        *id = g_bus_history[idx].id;
    if (len)
        *len = g_bus_history[idx].len;
    if (data && g_bus_history[idx].len > 0) {
        memcpy(data, g_bus_history[idx].data, g_bus_history[idx].len);
    }
    return true;
}

bool mock_can_find_latest_tx(uint32_t target_id, uint8_t *data, uint8_t *len) {
    if (g_total_tx_count == 0) {
        return false;
    }
    uint32_t search_limit = (g_total_tx_count < MOCK_CAN_HISTORY_SIZE) ? g_total_tx_count : MOCK_CAN_HISTORY_SIZE;
    for (uint32_t i = 0; i < search_limit; i++) {
        uint32_t idx = (g_total_tx_count - 1 - i) % MOCK_CAN_HISTORY_SIZE;
        if (g_bus_history[idx].id == target_id) {
            if (len)
                *len = g_bus_history[idx].len;
            if (data && g_bus_history[idx].len > 0) {
                memcpy(data, g_bus_history[idx].data, g_bus_history[idx].len);
            }
            return true;
        }
    }
    return false;
}

uint32_t mock_can_count_tx_by_id(uint32_t target_id) {
    uint32_t count = 0;
    uint32_t search_limit = (g_total_tx_count < MOCK_CAN_HISTORY_SIZE) ? g_total_tx_count : MOCK_CAN_HISTORY_SIZE;
    for (uint32_t i = 0; i < search_limit; i++) {
        uint32_t idx = (g_total_tx_count - 1 - i) % MOCK_CAN_HISTORY_SIZE;
        if (g_bus_history[idx].id == target_id) {
            count++;
        }
    }
    return count;
}

void mock_can_set_bus_off(bool bus_off) {
    uint8_t cur_idx = node_to_index(g_current_node);
    g_nodes[cur_idx].is_bus_off = bus_off;
}

void mock_can_set_tx_fail(bool fail) {
    uint8_t cur_idx = node_to_index(g_current_node);
    g_nodes[cur_idx].tx_fail = fail;
}

void mock_can_set_drop_count(uint32_t count) {
    g_drop_counter = count;
}

/* ========================================================================== */
/* Hardware Abstraction Layer Implementation (can_interface.h)               */
/* ========================================================================== */

bool can_init(void) {
    return true;
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    uint8_t cur_idx = node_to_index(g_current_node);
    if (g_nodes[cur_idx].is_bus_off) {
        return false;
    }
    if (g_nodes[cur_idx].tx_fail) {
        return false;
    }
    if (len > MOCK_CAN_MAX_FRAME_SIZE) {
        return false;
    }

    /* Simulate dropped frame if drop counter active */
    if (g_drop_counter > 0) {
        g_drop_counter--;
        return true; /* Physical transmission appeared to succeed to the MCU */
    }

    mock_can_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.len = len;
    frame.sender = g_current_node;
    frame.timestamp_ms = time_get_ms();
    if (data && len > 0) {
        memcpy(frame.data, data, len);
    }

    /* Record in bus history */
    uint32_t hist_idx = g_total_tx_count % MOCK_CAN_HISTORY_SIZE;
    g_bus_history[hist_idx] = frame;
    g_total_tx_count++;

    /* Broadcast to all other nodes */
    for (uint8_t i = 0; i < MOCK_CAN_MAX_NODES; i++) {
        if (i != cur_idx) {
            enqueue_node_frame(i, &frame);
        }
    }

    return true;
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    uint8_t cur_idx = node_to_index(g_current_node);
    mock_node_bus_state_t *st = &g_nodes[cur_idx];
    if (st->count == 0) {
        return false;
    }

    mock_can_frame_t *frame = &st->queue[st->head];
    if (id)
        *id = frame->id;
    if (len)
        *len = frame->len;
    if (data && frame->len > 0) {
        memcpy(data, frame->data, frame->len);
    }

    st->head = (st->head + 1) % MOCK_CAN_QUEUE_SIZE;
    st->count--;
    return true;
}

bool can_is_bus_off(void) {
    uint8_t cur_idx = node_to_index(g_current_node);
    return g_nodes[cur_idx].is_bus_off;
}

void can_recover(void) {
    uint8_t cur_idx = node_to_index(g_current_node);
    g_nodes[cur_idx].is_bus_off = false;
}
