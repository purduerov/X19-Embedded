/* tests/test_sil_bridge_logical.c */
#include "unity.h"
#include "harness/rov_test_main.h"
#include "harness/sil_clock.h"
#include "mocks/mock_bsp.h"
void setUp(void) {}
void tearDown(void) {}
static void test_logical_time_matches_cycles(void) {
    sil_clock_reset();
    for (int i = 0; i < 100; i++) { mock_bsp_advance_time_ms(10); }
    TEST_ASSERT_EQUAL_UINT32(1000, (uint32_t)(sil_clock_now_us() / 1000));
}
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_logical_time_matches_cycles); return UNITY_END(); }