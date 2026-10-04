/* tests/test_sil_vehicle.c */
#include "harness/sil_vehicle.h"
#include "harness/rov_test_main.h"
#include "unity.h"
/* Local stubs so this unit-test binary links the SIL_NODES table without
 * pulling in the full vehicle app sources. They are never invoked. */
void node1_app_init(void) {}
void node1_app_step(void) {}
void node2_app_init(void) {}
void node2_app_step(void) {}
void node3_app_init(void) {}
void node3_app_step(void) {}

void setUp(void) {}
void tearDown(void) {}
static void test_three_nodes_registered(void) {
    TEST_ASSERT_EQUAL_INT(3, sil_vehicle_node_count());
    TEST_ASSERT_EQUAL_STRING("control_board", sil_vehicle_node(0)->name);
}
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_three_nodes_registered); return UNITY_END(); }