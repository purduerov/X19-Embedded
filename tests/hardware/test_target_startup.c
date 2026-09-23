#include "main.h"
#include <setjmp.h>
#include <stdio.h>

int target_firmware_entrypoint(void);

enum { EVENT_HAL_INIT, EVENT_CLOCK, EVENT_GPIO, EVENT_FDCAN, EVENT_I2C, EVENT_TIM1, EVENT_TIM8, EVENT_APP };
static int s_events[16];
static unsigned s_event_count;
static unsigned s_failures;
static jmp_buf s_app_entered;

#define RECORD(event)                                                                                                   \
    do {                                                                                                                \
        if (s_event_count < (sizeof(s_events) / sizeof(s_events[0]))) {                                                 \
            s_events[s_event_count++] = (event);                                                                       \
        }                                                                                                               \
    } while (0)

static unsigned event_position(int event) {
    for (unsigned i = 0; i < s_event_count; i++) {
        if (s_events[i] == event) {
            return i;
        }
    }
    return s_event_count;
}

void HAL_Init(void) { RECORD(EVENT_HAL_INIT); }
void SystemClock_Config(void) { RECORD(EVENT_CLOCK); }
void MX_GPIO_Init(void) { RECORD(EVENT_GPIO); }
void MX_FDCAN1_Init(void) { RECORD(EVENT_FDCAN); }
void MX_I2C1_Init(void) { RECORD(EVENT_I2C); }
void MX_TIM1_Init(void) { RECORD(EVENT_TIM1); }
void MX_TIM8_Init(void) { RECORD(EVENT_TIM8); }

void app_main(void) {
    RECORD(EVENT_APP);
    longjmp(s_app_entered, 1);
}

static void require_before_app(int event, const char *description) {
    unsigned position = event_position(event);
    unsigned app_position = event_position(EVENT_APP);
    if (position >= app_position) {
        printf("[FAIL] %s must run before app_main\n", description);
        s_failures++;
    }
}

int main(void) {
    if (setjmp(s_app_entered) == 0) {
        target_firmware_entrypoint();
        printf("[FAIL] Firmware entry returned without starting app_main\n");
        return 1;
    }

    require_before_app(EVENT_HAL_INIT, "HAL_Init");
    require_before_app(EVENT_CLOCK, "SystemClock_Config");
    require_before_app(EVENT_GPIO, "MX_GPIO_Init");
    require_before_app(EVENT_FDCAN, "MX_FDCAN1_Init");
    require_before_app(EVENT_I2C, "MX_I2C1_Init");
#if TARGET_NODE == 2
    require_before_app(EVENT_TIM1, "MX_TIM1_Init");
    require_before_app(EVENT_TIM8, "MX_TIM8_Init");
#endif

    if (s_failures != 0U) {
        printf("Node %d target startup readiness: %u failure(s)\n", TARGET_NODE, s_failures);
        return 1;
    }
    printf("Node %d target startup readiness passed\n", TARGET_NODE);
    return 0;
}
