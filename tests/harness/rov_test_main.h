/* tests/harness/rov_test_main.h */
#ifndef ROV_TEST_MAIN_H
#define ROV_TEST_MAIN_H

#include "unity.h"
#include <stdlib.h>
#include <string.h>

/* Run a single Unity test only when it matches the ROV_TEST_FILTER
 * environment variable. An unset or empty filter runs every test.
 * Matching is a substring test against the stringified function name. */
#define ROV_RUN_TEST(fn)                                           \
    do {                                                           \
        const char *filter = getenv("ROV_TEST_FILTER");            \
        if (filter == NULL || filter[0] == '\0' ||                 \
            strstr(#fn, filter) != NULL) {                         \
            RUN_TEST(fn);                                          \
        }                                                          \
    } while (0)

/* Entry point for host SIL test executables built on Unity. */
#define ROV_TEST_MAIN() int main(void)

#endif /* ROV_TEST_MAIN_H */
