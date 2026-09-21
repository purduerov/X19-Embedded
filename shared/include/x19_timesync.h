/**
 * @file x19_timesync.h
 * @brief Backward compatibility forwarding header for rov_timesync.h.
 * @organization Purdue ROV
 */

#ifndef X19_TIMESYNC_H
#define X19_TIMESYNC_H

#include "rov_timesync.h"

typedef rov_timesync_state_t x19_timesync_state_t;

#define x19_timesync_init            rov_timesync_init
#define x19_timesync_process_master  rov_timesync_process_master
#define x19_timesync_calc_latency    rov_timesync_calc_latency
#define x19_timesync_update_latency  rov_timesync_update_latency
#define x19_timesync_get_time_us     rov_timesync_get_time_us
#define x19_timesync_is_synchronized rov_timesync_is_synchronized

#endif /* X19_TIMESYNC_H */
