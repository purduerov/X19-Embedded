#include "sil_vehicle.h"
#include "rov_types.h"

/* Real node lifecycle functions, provided by each node's app.c at link time
 * (sil_bridge_server) or by test-file stubs (test_sil_vehicle). */
extern void node1_app_init(void);
extern void node1_app_step(void);
extern void node2_app_init(void);
extern void node2_app_step(void);
extern void node3_app_init(void);
extern void node3_app_step(void);

static const sil_node_t SIL_NODES[] = {
    { "control_board", ROV_NODE_CONTROL_BOARD, node2_app_init, node2_app_step },
    { "pi_shield",     ROV_NODE_PI_SHIELD,     node1_app_init, node1_app_step },
    { "power_slab",    ROV_NODE_POWER_SLAB,    node3_app_init, node3_app_step },
};
int sil_vehicle_node_count(void) { return (int)(sizeof(SIL_NODES)/sizeof(SIL_NODES[0])); }
const sil_node_t *sil_vehicle_node(int idx) { return &SIL_NODES[idx]; }
void sil_vehicle_reset_all(void) {
    for (int i = 0; i < sil_vehicle_node_count(); i++) {
        const sil_node_t *node = sil_vehicle_node(i);
        if (node->init) {
            node->init();
        }
    }
}