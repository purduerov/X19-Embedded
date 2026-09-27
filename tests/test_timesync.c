/**
 * @file test_timesync.c
 * @brief Unit tests for rov_timesync.c (Time Synchronization & Latency Calculation Engine).
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "rov_parameters.h"
#include "rov_timesync.h"
#include <assert.h>
#include <stdio.h>

void test_timesync_initialization(void) {
    rov_timesync_state_t ts;
    rov_timesync_init(&ts);

    assert(ts.clock_offset_us == 0);
    assert(ts.one_way_delay_us == ROV_TIME_SYNC_DEFAULT_DELAY_US);
    assert(ts.synchronized == false);
    assert(ts.initial_sync_done == false);
    assert(rov_timesync_is_synchronized(&ts, 1000) == false);

    printf("[PASS] test_timesync_initialization\n");
}

void test_timesync_master_broadcast_and_slew(void) {
    rov_timesync_state_t ts;
    rov_timesync_init(&ts);

    /* Local hardware time at reception: 10,000 us (10 ms) */
    uint64_t local_rx_us = 10000;
    /* Master time: 1,000,000 us (1.000 s) */
    rov_time_sync_master_t master = {
        .master_time_us = 1000000,
        .sync_seq = 1,
        .flags = 0x01,
        .reserved = {0, 0, 0}
    };

    /* First sync snaps directly */
    assert(rov_timesync_process_master(&ts, &master, local_rx_us) == ROV_OK);
    assert(ts.synchronized == true);
    assert(ts.initial_sync_done == true);
    /* Target offset: (1,000,000 + 45) - 10,000 = 990,045 us */
    assert(ts.clock_offset_us == 990045);

    /* Check synchronized time query at local_rx_us: 10,000 + 990,045 = 1,000,045 us */
    uint64_t synced_time = rov_timesync_get_time_us(&ts, local_rx_us);
    assert(synced_time == 1000045);

    /* Second sync frame 100 ms later (at 10 Hz) with a slight master clock step (e.g. +200 us jump) */
    local_rx_us += 100000; /* local = 110,000 us */
    master.master_time_us += 100200; /* master jumped +200 us extra */
    assert(rov_timesync_process_master(&ts, &master, local_rx_us) == ROV_ERR_INVALID_ARG);
    master.sync_seq++;

    assert(rov_timesync_process_master(&ts, &master, local_rx_us) == ROV_OK);
    /* Slew rate limit is 50 us per step, so offset changes by +50 us, not +200 us */
    assert(ts.clock_offset_us == 990045 + ROV_TIME_SYNC_MAX_SLEW_US_PER_STEP);

    /* Verify time is strictly monotonic */
    uint64_t synced_time_2 = rov_timesync_get_time_us(&ts, local_rx_us);
    assert(synced_time_2 > synced_time);

    printf("[PASS] test_timesync_master_broadcast_and_slew\n");
}

void test_timesync_two_way_latency_calculation(void) {
    /* Simulate IEEE 1588 PTP Delay Request-Response exchange:
     *   Requester sends REQ at t1 = 1,000,000 us
     *   Responder receives REQ at t2 = 2,000,050 us (responder clock is ahead by ~1,000,000 us)
     *   Responder dwells for 20 us, sends RESP at t3 = 2,000,070 us
     *   Requester receives RESP at t4 = 1,000,120 us
     *
     * Total elapsed = t4 - t1 = 120 us
     * Dwell time = t3 - t2 = 20 us
     * Round-Trip Time (RTT) = 120 - 20 = 100 us
     * One-Way Delay = 100 / 2 = 50 us
     * Clock Offset = ((t2 - t1) - (t4 - t3)) / 2
     *   forward_diff = 2,000,050 - 1,000,000 = 1,000,050
     *   reverse_diff = 1,000,120 - 2,000,070 = -999,950
     *   offset = (1,000,050 - (-999,950)) / 2 = 2,000,000 / 2 = 1,000,000 us
     */
    uint64_t t1 = 1000000;
    uint64_t t2 = 2000050;
    uint64_t t3 = 2000070;
    uint64_t t4 = 1000120;

    int64_t offset_us = 0;
    uint32_t rtt_us = 0;
    uint32_t one_way_delay_us = 0;

    assert(rov_timesync_calc_latency(t1, t2, t3, t4, &offset_us, &rtt_us, &one_way_delay_us) == ROV_OK);
    assert(rtt_us == 100);
    assert(one_way_delay_us == 50);
    assert(offset_us == 1000000);

    /* Test invalid / non-causal timestamps */
    assert(rov_timesync_calc_latency(t4, t2, t3, t1, &offset_us, &rtt_us, &one_way_delay_us) == ROV_ERR_INVALID_ARG);
    assert(rov_timesync_calc_latency(t1, t3, t2, t4, &offset_us, &rtt_us, &one_way_delay_us) == ROV_ERR_INVALID_ARG);

    printf("[PASS] test_timesync_two_way_latency_calculation\n");
}

void test_timesync_timeout_and_health(void) {
    rov_timesync_state_t ts;
    rov_timesync_init(&ts);

    uint64_t now_us = 1000000;
    rov_time_sync_master_t master = {
        .master_time_us = 5000000,
        .sync_seq = 10,
        .flags = 0x01,
        .reserved = {0, 0, 0}
    };

    assert(rov_timesync_process_master(&ts, &master, now_us) == ROV_OK);
    assert(rov_timesync_is_synchronized(&ts, now_us) == true);

    /* 500 ms later: still synchronized */
    assert(rov_timesync_is_synchronized(&ts, now_us + 500000) == true);

    /* 1500 ms later (> ROV_TIME_SYNC_TIMEOUT_MS = 1000 ms): dropped synchronization */
    assert(rov_timesync_is_synchronized(&ts, now_us + 1500000) == false);
    assert(rov_timesync_get_time_us(&ts, now_us + 1500000) == now_us + 1500000);

    printf("[PASS] test_timesync_timeout_and_health\n");
}

int main(void) {
    printf("Running Time Synchronization & Latency Unit Tests...\n");
    test_timesync_initialization();
    test_timesync_master_broadcast_and_slew();
    test_timesync_two_way_latency_calculation();
    test_timesync_timeout_and_health();
    printf("All Time Synchronization Tests Passed Successfully!\n");
    return 0;
}
