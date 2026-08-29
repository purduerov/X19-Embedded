/**
 * @file tcan1044.h
 * @brief TI TCAN1044 High-Speed CAN FD Transceiver Driver.
 * @organization Purdue ROV
 */

#ifndef TCAN1044_H
#define TCAN1044_H

#include "x19_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool standby_mode;
} tcan1044_dev_t;

x19_status_t tcan1044_init(tcan1044_dev_t *dev);
x19_status_t tcan1044_set_standby(tcan1044_dev_t *dev, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* TCAN1044_H */
