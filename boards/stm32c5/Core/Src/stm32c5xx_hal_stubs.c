/**
 * @file stm32c5xx_hal_stubs.c
 * @brief Hardware Abstraction Layer implementation and peripheral stubs for STM32C542.
 * @organization Purdue ROV
 */

#include "bsp.h"
#include "main.h"
#include "stm32c5xx_hal.h"
#include <string.h>

volatile uint32_t uwTick = 0;
I2C_HandleTypeDef hi2c1;
FDCAN_HandleTypeDef hfdcan1;

static uint16_t s_gpio_port_state[4] = {0};

/* --- Core Time and HAL Control --- */

HAL_StatusTypeDef HAL_Init(void) {
    uwTick = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_DeInit(void) {
    return HAL_OK;
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
    (void)TickPriority;
    return HAL_OK;
}

void HAL_IncTick(void) {
    uwTick++;
}

uint32_t HAL_GetTick(void) {
    return uwTick;
}

void HAL_Delay(uint32_t Delay) {
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;

    if (wait < 0xFFFFFFFFU) {
        wait += 1U;
    }

    while ((HAL_GetTick() - tickstart) < wait) {
        /* On bare metal without hardware timer interrupt running, increment tick counter */
        uwTick++;
    }
}

void HAL_SuspendTick(void) {}

void HAL_ResumeTick(void) {}

/* --- CubeMX Peripheral Initializations --- */

void SystemClock_Config(void) {
    /* 144 MHz System Core Clock for STM32C542 */
    SystemCoreClock = 144000000U;
}

void MX_GPIO_Init(void) {
    /* Clocks and pin initializations for vehicle board */
}

void MX_FDCAN1_Init(void) {
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.State = 1;
}

void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.State = 1;
}

void Error_Handler(void) {
    bsp_emergency_brake_trip();
#if defined(__arm__) || defined(__thumb__)
    __asm volatile("cpsid i" : : : "memory");
#endif
    while (1) {
    }
}

/* --- GPIO Implementation --- */

static uint8_t get_gpio_index(const GPIO_TypeDef *GPIOx) {
    if (GPIOx == GPIOA)
        return 0;
    if (GPIOx == GPIOB)
        return 1;
    if (GPIOx == GPIOC)
        return 2;
    return 3;
}

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_InitTypeDef *GPIO_Init) {
    (void)GPIOx;
    (void)GPIO_Init;
}

void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin) {
    (void)GPIOx;
    (void)GPIO_Pin;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t idx = get_gpio_index(GPIOx);
    return (s_gpio_port_state[idx] & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    uint8_t idx = get_gpio_index(GPIOx);
    if (PinState == GPIO_PIN_SET) {
        s_gpio_port_state[idx] |= GPIO_Pin;
    } else {
        s_gpio_port_state[idx] &= ~GPIO_Pin;
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint8_t idx = get_gpio_index(GPIOx);
    s_gpio_port_state[idx] ^= GPIO_Pin;
}

HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    (void)GPIOx;
    (void)GPIO_Pin;
    return HAL_OK;
}

/* --- I2C Implementation --- */

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c) {
    if (!hi2c)
        return HAL_ERROR;
    hi2c->State = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c) {
    if (!hi2c)
        return HAL_ERROR;
    hi2c->State = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                          uint32_t Timeout) {
    (void)hi2c;
    (void)DevAddress;
    (void)pData;
    (void)Size;
    (void)Timeout;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout) {
    (void)hi2c;
    (void)DevAddress;
    (void)Timeout;
    if (pData && Size > 0) {
        memset(pData, 0, Size);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials,
                                        uint32_t Timeout) {
    (void)hi2c;
    (void)DevAddress;
    (void)Trials;
    (void)Timeout;
    return HAL_OK;
}

/* --- FDCAN Implementation --- */

HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *hfdcan) {
    if (!hfdcan)
        return HAL_ERROR;
    hfdcan->State = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *hfdcan) {
    if (!hfdcan)
        return HAL_ERROR;
    hfdcan->State = 2;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *hfdcan) {
    if (!hfdcan)
        return HAL_ERROR;
    hfdcan->State = 1;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t NonMatchingStd,
                                               uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                                               uint32_t RejectRemoteExt) {
    (void)hfdcan;
    (void)NonMatchingStd;
    (void)NonMatchingExt;
    (void)RejectRemoteStd;
    (void)RejectRemoteExt;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, const FDCAN_FilterTypeDef *sFilterConfig) {
    (void)hfdcan;
    (void)sFilterConfig;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *hfdcan, const FDCAN_TxHeaderTypeDef *pTxHeader,
                                                const uint8_t *pTxData) {
    if (!hfdcan || !pTxHeader || !pTxData)
        return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0,
                                         FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *pRxData) {
    (void)hfdcan;
    (void)RxFifo0;
    if (!pRxHeader || !pRxData)
        return HAL_ERROR;
    return HAL_OK;
}
