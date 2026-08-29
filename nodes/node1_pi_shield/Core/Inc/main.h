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

#include "x19_parameters.h"
#include "x19_can_protocol.h"
#include "x19_types.h"
#include "x19_safety.h"

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* NODE1_MAIN_H */
