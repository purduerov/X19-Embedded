/* tests/test_sil_clock.c */
#include "harness/rov_test_main.h"
#include "harness/sil_clock.h"
#include "unity.h"
void setUp(void) {}
void tearDown(void) {}
static void test_starts_zero(void) { sil_clock_reset(); TEST_ASSERT_EQUAL_UINT64(0, sil_clock_now_us()); }
static void test_advances(void)   { sil_clock_reset(); sil_clock_advance_us(1500); TEST_ASSERT_EQUAL_UINT64(1500, sil_clock_now_us()); }
ROV_TEST_MAIN() { UNITY_BEGIN(); ROV_RUN_TEST(test_starts_zero); ROV_RUN_TEST(test_advances); return UNITY_END(); }