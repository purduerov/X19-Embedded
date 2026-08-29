/**
 * @file main.h
 * @brief Header for Control Board application (Node 2).
 * @organization Purdue ROV
 */

#ifndef NODE2_MAIN_H
#define NODE2_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "x19_parameters.h"
#include "x19_can_protocol.h"
#include "x19_types.h"
#include "x19_safety.h"

/* Function prototypes */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* NODE2_MAIN_H */
