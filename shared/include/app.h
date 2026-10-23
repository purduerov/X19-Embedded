/**
 * @file app.h
 * @brief Common application entry point and lifecycle interface for Purdue ROV embedded targets.
 * @organization Purdue ROV
 */

#ifndef APP_H
#define APP_H

#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Main application entry point invoked by main.c after low-level hardware boot.
 *
 * Vehicle node firmware and developer scratchpad code implement this entry point
 * with portable logic and zero direct vendor HAL calls.
 */
void app_main(void);

/**
 * @brief Standard application lifecycle initialization prototype.
 */
void app_init(void);

/**
 * @brief Standard single-step cyclic update routine.
 */
void app_step(void);

/* =========================================================================
 * Per-node lifecycle declarations for subsea vehicle nodes & SIL testing
 * ========================================================================= */

/* Node 1: Pi Shield */
void node1_app_init(void);
void node1_app_step(void);
void node1_leak_irq_handler(void);
uint8_t node1_i2c_scan(void);

/* Node 2: Control Board */
void node2_app_init(void);
void node2_app_step(void);

/* Node 3: Power Slab */
void node3_app_init(void);
void node3_app_step(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
