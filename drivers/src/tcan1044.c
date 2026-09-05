/**
 * @file tcan1044.c
 * @brief TI TCAN1044 High-Speed CAN FD Transceiver Driver Implementation.
 * @organization Purdue ROV
 */

#include "tcan1044.h"

x19_status_t tcan1044_init(tcan1044_dev_t *dev) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    dev->standby_mode = false;
    return X19_OK;
}

x19_status_t tcan1044_set_standby(tcan1044_dev_t *dev, bool enable) {
    if (!dev)
        return X19_ERR_INVALID_ARG;
    dev->standby_mode = enable;
    return X19_OK;
}
