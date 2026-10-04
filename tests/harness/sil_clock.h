#ifndef SIL_CLOCK_H
#define SIL_CLOCK_H
#include <stdint.h>
void sil_clock_reset(void);
uint64_t sil_clock_now_us(void);
void sil_clock_advance_us(uint64_t dt_us);
#endif /* SIL_CLOCK_H */