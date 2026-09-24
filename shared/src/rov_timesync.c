/**
 * @file rov_timesync.c
 * @brief Distributed Time Synchronization & Network Latency Engine Implementation.
 * @organization Purdue ROV
 */

#include "rov_timesync.h"
#include <string.h>

void rov_timesync_init(rov_timesync_state_t *ts) {
    if (!ts)
        return;

    memset(ts, 0, sizeof(rov_timesync_state_t));
    ts->one_way_delay_us = ROV_TIME_SYNC_DEFAULT_DELAY_US;
    ts->synchronized = false;
    ts->initial_sync_done = false;
}

rov_status_t rov_timesync_process_master(rov_timesync_state_t *ts, const rov_time_sync_master_t *sync,
                                         uint64_t local_rx_us) {
    if (!ts || !sync)
        return ROV_ERR_INVALID_ARG;

    if (ts->initial_sync_done && (int32_t)(sync->sync_seq - ts->last_seq) <= 0) {
        return ROV_ERR_INVALID_ARG;
    }

    /* Target synchronized master time accounting for one-way bus propagation delay */
    int64_t target_offset = ((int64_t)sync->master_time_us + (int64_t)ts->one_way_delay_us) - (int64_t)local_rx_us;

    if (!ts->initial_sync_done) {
        /* First synchronization: snap directly to master time */
        ts->clock_offset_us = target_offset;
        ts->initial_sync_done = true;
    } else {
        /* Subsequent sync: apply slew rate limiting to guarantee strict monotonic progression */
        int64_t diff = target_offset - ts->clock_offset_us;
        if (diff > (int64_t)ROV_TIME_SYNC_MAX_SLEW_US_PER_STEP) {
            diff = (int64_t)ROV_TIME_SYNC_MAX_SLEW_US_PER_STEP;
        } else if (diff < -(int64_t)ROV_TIME_SYNC_MAX_SLEW_US_PER_STEP) {
            diff = -(int64_t)ROV_TIME_SYNC_MAX_SLEW_US_PER_STEP;
        }
        ts->clock_offset_us += diff;
    }

    ts->last_sync_rx_us = local_rx_us;
    ts->last_master_time_us = sync->master_time_us;
    ts->last_seq = sync->sync_seq;
    ts->master_status = sync->flags;
    ts->synchronized = true;

    return ROV_OK;
}

rov_status_t rov_timesync_calc_latency(uint64_t t1_us, uint64_t t2_us, uint64_t t3_us, uint64_t t4_us,
                                       int64_t *offset_us, uint32_t *rtt_us, uint32_t *one_way_delay_us) {
    if (!offset_us || !rtt_us || !one_way_delay_us)
        return ROV_ERR_INVALID_ARG;

    /* Check causality */
    if (t4_us < t1_us || t3_us < t2_us)
        return ROV_ERR_INVALID_ARG;

    uint64_t total_elapsed = t4_us - t1_us;
    uint64_t node_dwell = t3_us - t2_us;

    if (total_elapsed < node_dwell)
        return ROV_ERR_INVALID_ARG;

    uint64_t rtt = total_elapsed - node_dwell;
    *rtt_us = (uint32_t)rtt;
    *one_way_delay_us = (uint32_t)(rtt / 2);

    /* Clock offset: ((t2 - t1) - (t4 - t3)) / 2 */
    int64_t forward_diff = (int64_t)t2_us - (int64_t)t1_us;
    int64_t reverse_diff = (int64_t)t4_us - (int64_t)t3_us;
    *offset_us = (forward_diff - reverse_diff) / 2;

    return ROV_OK;
}

rov_status_t rov_timesync_update_latency(rov_timesync_state_t *ts, uint32_t rtt_us, uint32_t one_way_delay_us) {
    if (!ts)
        return ROV_ERR_INVALID_ARG;

    /* Exponential moving average jitter estimation */
    uint32_t delay_diff = (one_way_delay_us > ts->one_way_delay_us) ? (one_way_delay_us - ts->one_way_delay_us)
                                                                    : (ts->one_way_delay_us - one_way_delay_us);
    ts->jitter_us = (ts->jitter_us * 3 + delay_diff) / 4;

    ts->round_trip_us = rtt_us;
    ts->one_way_delay_us = one_way_delay_us;

    return ROV_OK;
}

uint64_t rov_timesync_get_time_us(const rov_timesync_state_t *ts, uint64_t local_now_us) {
    if (!ts || !rov_timesync_is_synchronized(ts, local_now_us))
        return local_now_us;

    int64_t synced = (int64_t)local_now_us + ts->clock_offset_us;
    return (synced > 0) ? (uint64_t)synced : 0ULL;
}

bool rov_timesync_is_synchronized(const rov_timesync_state_t *ts, uint64_t local_now_us) {
    if (!ts || !ts->synchronized || !ts->initial_sync_done)
        return false;

    if (local_now_us < ts->last_sync_rx_us)
        return false;

    uint64_t elapsed_us = local_now_us - ts->last_sync_rx_us;
    if (elapsed_us > (uint64_t)ROV_TIME_SYNC_TIMEOUT_MS * 1000ULL)
        return false;

    return true;
}
