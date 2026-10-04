/* tests/test_rov_test_main.c */
#include "harness/rov_test_main.h"
void setUp(void) {}
void tearDown(void) {}
static void test_alpha(void) { TEST_ASSERT_TRUE(1); }
static void test_beta(void)  { TEST_ASSERT_TRUE(1); }
ROV_TEST_MAIN() {
    UNITY_BEGIN();
    ROV_RUN_TEST(test_alpha);
    ROV_RUN_TEST(test_beta);
    return UNITY_END();
}
