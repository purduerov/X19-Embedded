/**
 * @file platform_can.c
 * @brief STM32C5 platform FDCAN transport implementation.
 * @organization Purdue ROV
 */

#include "can_interface.h"
#include "main.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

bool can_init(void) {
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);
    return HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    if (!data || len > 64) {
        return false;
    }

    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    uint32_t dlc;
    if (len <= 8U) {
        dlc = (uint32_t)len << 16U;
    } else if (len <= 12U) {
        dlc = FDCAN_DLC_BYTES_12;
    } else if (len <= 16U) {
        dlc = FDCAN_DLC_BYTES_16;
    } else if (len <= 20U) {
        dlc = FDCAN_DLC_BYTES_20;
    } else if (len <= 24U) {
        dlc = FDCAN_DLC_BYTES_24;
    } else if (len <= 32U) {
        dlc = FDCAN_DLC_BYTES_32;
    } else if (len <= 48U) {
        dlc = FDCAN_DLC_BYTES_48;
    } else {
        dlc = FDCAN_DLC_BYTES_64;
    }
    tx_header.DataLength = dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (const uint8_t *)data) == HAL_OK;
}

bool can_send_emergency(uint32_t id, const uint8_t *data, uint8_t len) {
    return can_send(id, data, len);
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    if (!id || !data || !len) {
        return false;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, &rx_header, data) == HAL_OK) {
        *id = rx_header.Identifier;
        uint32_t dlc_bytes = rx_header.DataLength >> 16U;
        *len = (uint8_t)(dlc_bytes > 64U ? 64U : dlc_bytes);
        return true;
    }

    return false;
}
