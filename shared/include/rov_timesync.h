/**
 * @file rov_timesync.h
 * @brief High-Resolution Distributed Time Synchronization & Network Latency Engine.
 * @organization Purdue ROV
 *
 * Implements a PTP-Lite (Precision Time Protocol) time synchronization architecture
 * over CAN FD for subsea STM32 microcontroller nodes and the Raspberry Pi companion computer.
 *
 * Features:
 *  - Clock offset tracking between Pi Core (Grandmaster Clock) and local STM32 hardware timers.
 *  - Slew-rate limited clock smoothing to guarantee monotonic time progression (d(T_sync)/dt > 0).
 *  - Two-way round-trip latency (RTT) and one-way bus propagation delay calculation.
 *  - Jitter and sync health monitoring with watchdog timeout fallback.
 *  - Integration with 100 Hz Navigation Telemetry (IMU timestamping).
 */

#ifndef ROV_TIMESYNC_H
#define ROV_TIMESYNC_H

#include "rov_parameters.h"
#include "rov_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Time Synchronization State Tracker.
 */
typedef struct {
    int64_t clock_offset_us;      /**< Clock offset: T_synced = T_local + clock_offset_us */
    uint64_t last_sync_rx_us;     /**< Local hardware timestamp when last master sync was received */
    uint64_t last_master_time_us; /**< Master timestamp from last received sync frame */
    uint32_t last_seq;            /**< Last received sequence number */
    uint32_t round_trip_us;       /**< Last measured round-trip time in microseconds */
    uint32_t one_way_delay_us;    /**< Estimated one-way bus propagation delay in microseconds */
    uint32_t jitter_us;           /**< Estimated delay jitter in microseconds */
    bool synchronized;            /**< true if actively synchronized within timeout window */
    bool initial_sync_done;       /**< true after first valid sync frame */
    uint8_t master_status;        /**< Master status flags (e.g. topside NTP lock) */
} rov_timesync_state_t;

/**
 * @brief Initialize time synchronization state.
 * @param ts Pointer to timesync state tracker.
 */
void rov_timesync_init(rov_timesync_state_t *ts);

/**
 * @brief Process an incoming master clock synchronization frame (0x010).
 *
 * Applies the calibrated one-way delay and updates the local clock offset.
 * Applies slew-rate limiting on subsequent updates to prevent clock discontinuities.
 *
 * @param ts Pointer to timesync state tracker.
 * @param sync Pointer to unpacked master frame.
 * @param local_rx_us Local hardware timer timestamp at moment of frame reception.
 * @return ROV_OK on success, ROV_ERR_INVALID_ARG on null pointers.
 */
rov_status_t rov_timesync_process_master(rov_timesync_state_t *ts, const rov_time_sync_master_t *sync,
                                         uint64_t local_rx_us);

/**
 * @brief Calculate two-way network latency and clock offset from a 4-timestamp exchange.
 *
 * Implements IEEE 1588 PTP delay request-response calculations:
 *   RTT = (t4 - t1) - (t3 - t2)
 *   One-Way Delay = RTT / 2
 *   Clock Offset = ((t2 - t1) - (t4 - t3)) / 2
 *
 * @param t1_us Requester transmit timestamp.
 * @param t2_us Responder receive timestamp.
 * @param t3_us Responder reply transmit timestamp.
 * @param t4_us Requester reply receive timestamp.
 * @param[out] offset_us Calculated clock offset in microseconds.
 * @param[out] rtt_us Round-trip time in microseconds.
 * @param[out] one_way_delay_us One-way delay in microseconds.
 * @return ROV_OK if timestamps are causal and valid, ROV_ERR_INVALID_ARG otherwise.
 */
rov_status_t rov_timesync_calc_latency(uint64_t t1_us, uint64_t t2_us, uint64_t t3_us, uint64_t t4_us,
                                       int64_t *offset_us, uint32_t *rtt_us, uint32_t *one_way_delay_us);

/**
 * @brief Update timesync state with two-way latency measurement results.
 * @param ts Pointer to timesync state tracker.
 * @param rtt_us Measured round-trip time in microseconds.
 * @param one_way_delay_us Measured one-way propagation delay in microseconds.
 * @return ROV_OK on success.
 */
rov_status_t rov_timesync_update_latency(rov_timesync_state_t *ts, uint32_t rtt_us, uint32_t one_way_delay_us);

/**
 * @brief Get current synchronized vehicle time in microseconds.
 *
 * Returns T_local + clock_offset_us. If synchronized is false, returns T_local.
 *
 * @param ts Pointer to timesync state tracker.
 * @param local_now_us Current local hardware timer in microseconds.
 * @return Synchronized vehicle time in microseconds.
 */
uint64_t rov_timesync_get_time_us(const rov_timesync_state_t *ts, uint64_t local_now_us);

/**
 * @brief Check if time synchronization is active and healthy.
 * @param ts Pointer to timesync state tracker.
 * @param local_now_us Current local hardware timer in microseconds.
 * @return true if synchronized within timeout window, false otherwise.
 */
bool rov_timesync_is_synchronized(const rov_timesync_state_t *ts, uint64_t local_now_us);

#ifdef __cplusplus
}
#endif

#endif /* ROV_TIMESYNC_H */
