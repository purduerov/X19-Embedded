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

#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"
#include "rov_types.h"

/* Function prototypes */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* NODE2_MAIN_H */
