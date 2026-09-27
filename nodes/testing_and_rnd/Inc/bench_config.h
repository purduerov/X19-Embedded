/**
 * @file bench_config.h
 * @brief Dev Board Bench Application Selector.
 *
 * Uncomment exactly ONE of the lines below to select which vehicle node
 * logic runs on this physical development board (NUCLEO-F411RE / F446RE).
 */

#ifndef BENCH_CONFIG_H
#define BENCH_CONFIG_H

/* =========================================================================
 * Select Active Node on Dev Board Bench (Uncomment ONE):
 * ========================================================================= */
#define BENCH_APP_NODE1_PI_SHIELD /* Node 1: BME280, leak probes, 10 Hz CAN telemetry */
// #define BENCH_APP_NODE2_CONTROL  /* Node 2: 8x ESC PWMs, solenoids, IMU, depth */
// #define BENCH_APP_NODE3_POWER    /* Node 3: PMBus power bricks, eFuse monitor */
// #define BENCH_APP_RND_SCANNER    /* Testing: Raw I2C scanner & CAN ping-pong */

#endif /* BENCH_CONFIG_H */
