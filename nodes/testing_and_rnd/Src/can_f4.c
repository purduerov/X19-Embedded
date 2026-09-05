/**
 * @file can_f4.c
 * @brief bxCAN Transport Abstraction Layer Implementation for STM32F446RE.
 */

#include "can.h"
#include "can_interface.h"
#include <string.h>

bool can_init(void) {
    /* 1. Configure CAN acceptance filter (Accept all frames into FIFO 0) */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        return false;
    }

    /* 2. Start bxCAN hardware peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return false;
    }

    return true;
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
    if (len > 8) {
        len = 8;
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = id;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)data, &tx_mailbox) != HAL_OK) {
        return false; /* Mailboxes full or bus-off */
    }

    return true;
}

bool can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        return false;
    }

    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
        return false;
    }

    if (id != NULL) {
        *id = rx_header.StdId;
    }
    if (len != NULL) {
        *len = (uint8_t)rx_header.DLC;
    }

    return true;
}

bool can_is_bus_off(void) {
    return (hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

void can_recover(void) {
    if (can_is_bus_off()) {
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_Start(&hcan1);
    }
}
