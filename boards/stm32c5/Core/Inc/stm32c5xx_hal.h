/**
 * @file stm32c5xx_hal.h
 * @brief STM32C5xx Hardware Abstraction Layer API.
 * @organization Purdue ROV
 */

#ifndef STM32C5XX_HAL_H
#define STM32C5XX_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32c5xx.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define STM32C5
#define STM32C5xx

typedef enum { HAL_OK = 0x00U, HAL_ERROR = 0x01U, HAL_BUSY = 0x02U, HAL_TIMEOUT = 0x03U } HAL_StatusTypeDef;

typedef enum { HAL_UNLOCKED = 0x00U, HAL_LOCKED = 0x01U } HAL_LockTypeDef;

/* --- GPIO Types and Constants --- */
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET = 1U } GPIO_PinState;

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_All ((uint16_t)0xFFFF)

#define GPIO_MODE_INPUT     0x00000000U
#define GPIO_MODE_OUTPUT_PP 0x00000001U
#define GPIO_MODE_OUTPUT_OD 0x00000011U
#define GPIO_MODE_AF_PP     0x00000002U
#define GPIO_MODE_AF_OD     0x00000012U
#define GPIO_MODE_ANALOG    0x00000003U

#define GPIO_NOPULL   0x00000000U
#define GPIO_PULLUP   0x00000001U
#define GPIO_PULLDOWN 0x00000002U

#define GPIO_SPEED_FREQ_LOW       0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM    0x00000001U
#define GPIO_SPEED_FREQ_HIGH      0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x00000003U

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* --- Core Time and HAL Control --- */
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);

/* --- I2C Types and API --- */
typedef struct {
    uint32_t Timing;
    uint32_t OwnAddress1;
    uint32_t AddressingMode;
    uint32_t DualAddressMode;
    uint32_t OwnAddress2;
    uint32_t GeneralCallMode;
    uint32_t NoStretchMode;
} I2C_InitTypeDef;

typedef struct {
    I2C_TypeDef *Instance;
    I2C_InitTypeDef Init;
    uint8_t *pBuffPtr;
    uint16_t XferSize;
    volatile uint16_t XferCount;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} I2C_HandleTypeDef;

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout);

/* --- FDCAN Types and API --- */
#define FDCAN_STANDARD_ID  0x00000000U
#define FDCAN_EXTENDED_ID  0x40000000U
#define FDCAN_DATA_FRAME   0x00000000U
#define FDCAN_REMOTE_FRAME 0x20000000U

#define FDCAN_DLC_BYTES_0  0x00000000U
#define FDCAN_DLC_BYTES_1  0x00010000U
#define FDCAN_DLC_BYTES_2  0x00020000U
#define FDCAN_DLC_BYTES_3  0x00030000U
#define FDCAN_DLC_BYTES_4  0x00040000U
#define FDCAN_DLC_BYTES_5  0x00050000U
#define FDCAN_DLC_BYTES_6  0x00060000U
#define FDCAN_DLC_BYTES_7  0x00070000U
#define FDCAN_DLC_BYTES_8  0x00080000U
#define FDCAN_DLC_BYTES_12 0x00090000U
#define FDCAN_DLC_BYTES_16 0x000A0000U
#define FDCAN_DLC_BYTES_20 0x000B0000U
#define FDCAN_DLC_BYTES_24 0x000C0000U
#define FDCAN_DLC_BYTES_32 0x000D0000U
#define FDCAN_DLC_BYTES_48 0x000E0000U
#define FDCAN_DLC_BYTES_64 0x000F0000U

#define FDCAN_ESI_ACTIVE   0x00000000U
#define FDCAN_ESI_PASSIVE  0x80000000U
#define FDCAN_BRS_OFF      0x00000000U
#define FDCAN_BRS_ON       0x00100000U
#define FDCAN_CLASSIC_CAN  0x00000000U
#define FDCAN_FD_CAN       0x00200000U
#define FDCAN_NO_TX_EVENTS 0x00000000U

#define FDCAN_FRAME_CLASSIC   0x00000000U
#define FDCAN_FRAME_FD_NO_BRS 0x00010000U
#define FDCAN_FRAME_FD_BRS    0x00020000U

#define FDCAN_MODE_NORMAL            0x00000000U
#define FDCAN_MODE_RESTRICTED        0x00000001U
#define FDCAN_MODE_BUS_MONITORING    0x00000002U
#define FDCAN_MODE_INTERNAL_LOOPBACK 0x00000003U
#define FDCAN_MODE_EXTERNAL_LOOPBACK 0x00000004U

#define FDCAN_ACCEPT_IN_RX_FIFO0 0x00000000U
#define FDCAN_ACCEPT_IN_RX_FIFO1 0x00000001U
#define FDCAN_REJECT             0x00000002U
#define FDCAN_FILTER_REMOTE      0x00000000U
#define FDCAN_REJECT_REMOTE      0x00000001U

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct {
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t RxTimestamp;
    uint32_t FilterIndex;
    uint32_t IsFilterMatchingFrame;
} FDCAN_RxHeaderTypeDef;

typedef struct {
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
    uint32_t RxBufferIndex;
    uint32_t IsExtendedIdMask;
} FDCAN_FilterTypeDef;

typedef struct {
    uint32_t FrameFormat;
    uint32_t Mode;
    uint32_t AutoRetransmission;
    uint32_t TransmitPause;
    uint32_t ProtocolException;
    uint32_t NominalPrescaler;
    uint32_t NominalSyncJumpWidth;
    uint32_t NominalTimeSeg1;
    uint32_t NominalTimeSeg2;
    uint32_t DataPrescaler;
    uint32_t DataSyncJumpWidth;
    uint32_t DataTimeSeg1;
    uint32_t DataTimeSeg2;
} FDCAN_InitTypeDef;

typedef struct {
    FDCAN_TypeDef *Instance;
    FDCAN_InitTypeDef Init;
    HAL_LockTypeDef Lock;
    volatile uint32_t State;
    volatile uint32_t ErrorCode;
} FDCAN_HandleTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;

HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, const FDCAN_FilterTypeDef *sFilterConfig);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, const FDCAN_TxHeaderTypeDef *pTxHeader,
                                                const uint8_t *pTxData);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0,
                                         FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData);

#ifdef __cplusplus
}
#endif

#endif /* STM32C5XX_HAL_H */
