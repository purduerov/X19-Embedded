#include "sil_clock.h"
static uint64_t g_now_us;
void sil_clock_reset(void) { g_now_us = 0; }
uint64_t sil_clock_now_us(void) { return g_now_us; }
void sil_clock_advance_us(uint64_t dt_us) { g_now_us += dt_us; }