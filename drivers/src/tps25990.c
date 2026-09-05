/**
 * @file tps25990.c
 * @brief TI TPS25990 PMBus Driver Implementation.
 * @organization Purdue ROV
 */

#include "tps25990.h"
#include <string.h>

x19_status_t tps25990_init(tps25990_dev_t *dev, uint8_t pmbus_addr) {
    if (!dev) return X19_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(tps25990_dev_t));
    dev->pmbus_addr = pmbus_addr;
    return X19_OK;
}

x19_status_t tps25990_read_telemetry(tps25990_dev_t *dev) {
    if (!dev) return X19_ERR_INVALID_ARG;
    return X19_OK;
}
