/**
 * @file tcan1044.c
 * @brief TI TCAN1044 High-Speed CAN FD Transceiver Driver Implementation.
 * @organization Purdue ROV
 */

#include "tcan1044.h"

rov_status_t tcan1044_init(tcan1044_dev_t *dev) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    dev->standby_mode = false;
    return ROV_OK;
}

rov_status_t tcan1044_set_standby(tcan1044_dev_t *dev, bool enable) {
    if (!dev)
        return ROV_ERR_INVALID_ARG;
    dev->standby_mode = enable;
    return ROV_OK;
}
