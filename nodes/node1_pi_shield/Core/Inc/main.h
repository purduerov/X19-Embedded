/**
 * @file main.h
 * @brief Header for Pi Shield application (Node 1).
 * @organization Purdue ROV
 */

#ifndef NODE1_MAIN_H
#define NODE1_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rov_can_protocol.h"
#include "rov_parameters.h"
#include "rov_safety.h"
#include "rov_types.h"

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* NODE1_MAIN_H */
