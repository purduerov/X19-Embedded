/**
 * @file bsp_f411.c
 * @brief Board Support Package initialization entry point for NUCLEO-F411RE.
 */

#include "bsp.h"
#include "can_interface.h"
#include <stdio.h>

void bsp_init(void) {
    /* Disable stdout buffering so printf flushes immediately to UART */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Initialize and start CAN hardware and filters */
    can_init();
}
