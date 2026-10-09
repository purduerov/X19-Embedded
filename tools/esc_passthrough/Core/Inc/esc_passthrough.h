/**
  ******************************************************************************
  * @file    esc_passthrough.h
  * @brief   BLHeli ESC programming passthrough (Betaflight-compatible MSP +
  *          BLHeli 4-way interface) bridging the host UART to the ESC's
  *          one-wire bootloader on PA8.
  ******************************************************************************
  */
#ifndef ESC_PASSTHROUGH_H
#define ESC_PASSTHROUGH_H

#include "main.h"

/**
  * @brief  Run the passthrough programmer on the given host UART. Use with
  *         esc-configurator.com or BLHeliSuite as if this were a flight
  *         controller. Never returns; reset the board to leave.
  */
void ESC_Passthrough_Run(UART_HandleTypeDef *huart);

#endif /* ESC_PASSTHROUGH_H */
