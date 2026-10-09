/**
 * @file platform_can.c
 * @brief NUCLEO-G474 platform FDCAN transport implementation.
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
    tx_header.DataLength = ((uint32_t)len <= 8U) ? ((uint32_t)len << 16U) : FDCAN_DLC_BYTES_64;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data) == HAL_OK;
}

bool can_send_emergency(uint32_t id, const uint8_t *data, uint8_t len) {
    return can_send(id, data, len);
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    if (!id || !data || !len) {
        return false;
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) {
        return false;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
        return false;
    }

    *id = rx_header.Identifier;
    *len = (uint8_t)((rx_header.DataLength >> 16U) & 0x0F);
    return true;
}
