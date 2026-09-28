#ifndef POWER_SEQUENCE_H
#define POWER_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PWR_SEQ_INIT = 0,
    PWR_SEQ_WAIT_LOGIC_STABLE,
    PWR_SEQ_DIAGNOSTICS,
    PWR_SEQ_STAGGER_ENABLE,
    PWR_SEQ_RUNNING,
    PWR_SEQ_FAULT
} power_seq_state_t;

void power_sequence_init(void);
void power_sequence_step(void);

void power_sequence_emergency_stop(void);
power_seq_state_t power_sequence_get_state(void);

#endif /* POWER_SEQUENCE_H */
