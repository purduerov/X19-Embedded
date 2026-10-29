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
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 26, 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
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
    uint32_t dlc_code = (rx_header.DataLength >> 16U) & 0x0FU;
    static const uint8_t dlc_to_bytes[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    *len = dlc_to_bytes[dlc_code];
    return true;
}

bool can_is_bus_off(void) {
    FDCAN_ProtocolStatusTypeDef ps;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) == HAL_OK) {
        return ps.BusOff != 0;
    }
    return false;
}

void can_recover(void) {
    HAL_FDCAN_Stop(&hfdcan1);
    can_init();
}

bool can_get_diagnostics(can_diagnostics_t *diag) {
    if (!diag) {
        return false;
    }
    memset(diag, 0, sizeof(*diag));

    FDCAN_ProtocolStatusTypeDef ps;
    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) == HAL_OK) {
        diag->last_error_code = (uint8_t)ps.LastErrorCode;
        diag->data_last_error_code = (uint8_t)ps.DataLastErrorCode;
        diag->bus_off = (ps.BusOff != 0);
        diag->error_passive = (ps.ErrorPassive != 0);
        diag->error_warning = (ps.Warning != 0);
    }

    FDCAN_ErrorCountersTypeDef ec;
    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) == HAL_OK) {
        diag->tx_error_count = (uint8_t)ec.TxErrorCnt;
        diag->rx_error_count = (uint8_t)ec.RxErrorCnt;
    }

    diag->tx_fifo_free_level = (uint8_t)(hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFFL);
    return true;
}
