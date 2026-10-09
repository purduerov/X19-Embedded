#ifndef SIL_VEHICLE_H
#define SIL_VEHICLE_H

#include <stdint.h>

typedef struct {
    const char *name;
    uint8_t     can_node_id;
    void      (*init)(void);
    void      (*step)(void);
} sil_node_t;

int sil_vehicle_node_count(void);
const sil_node_t *sil_vehicle_node(int idx);
void sil_vehicle_reset_all(void);

#endif /* SIL_VEHICLE_H */