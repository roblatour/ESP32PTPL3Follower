// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include <cstdint>
#include "esp_err.h"
#include "esp_eth.h"

namespace follower
{
    enum class ClockState
    {
        Starting,
        LinkDown,
        Listening,
        Acquiring,
        Locked,
        Holdover,
        Unsynchronized,
        Fault
    };
    struct ClockSnapshot
    {
        ClockState state = ClockState::Starting;
        bool utc_valid = false;
        bool locked = false;
        bool leap_ambiguous = false;
        bool time_traceable = false;
        uint32_t generation = 0;
        uint64_t master_tag = 0;
        int64_t monotonic_us = 0;
        int64_t utc_ns = 0;
        int64_t tai_ns = 0;
        int64_t offset_ns = 0;
        int64_t delay_ns = 0;
        int16_t utc_offset = 0;
        int32_t frequency_ppb = 0;
        uint64_t mutations = 0;
        uint64_t lock_epoch = 0;
        // Why LOCKED was most recently revoked (static string; "none" until the first loss).
        const char *unlock_reason = "none";
        // Most recent precise PHC-derived UTC minus CLOCK_REALTIME comparison.
        int64_t bridge_error_ns = 0;
    };
    // Initialize once before starting protocol tasks. This owner alone writes
    // the TAI PHC and standard UTC CLOCK_REALTIME.
    // Public calls serialize internally and never perform networking or logging.
    esp_err_t clock_owner_init(esp_eth_handle_t ethernet);
    // A read-only value/token; release the owner lock before IO or reporting.
    // utc_valid records UTC establishment, not current synchronization; inspect locked.
    ClockSnapshot clock_snapshot();
    void clock_link(bool up);
    // Call for every verified selected-master Announce, including unchanged metadata,
    // so its monotonic validity deadline is refreshed. Leap ambiguity prevents lock.
    void clock_master(uint64_t tag, bool valid, int16_t utc_offset, bool leap, bool traceable);
    void clock_invalidate();
    // Caller must verify the selected master's coherent Sync/Follow_Up and genuine
    // hardware RX timestamp, in this generation and within configured pair-age limits.
    // Nanoseconds use continuous TAI. This coarse path avoids q17 overflow at boot;
    // it steps CURRENT PHC by master_tai_ns-received_phc_ns, not back to packet time.
    // Startup/recovery step policy applies. True means both PHC/UTC were seeded and
    // generation changed; false means refused or failed (inspect snapshot for FAULT).
    bool clock_seed(int64_t master_tai_ns, int64_t received_phc_ns, uint32_t generation);
    // Offset is follower-minus-master TAI; sample_us is the monotonic sample epoch.
    // False includes acquisition steps, invalid/stale samples, outliers, and faults.
    bool clock_measurement(int64_t offset_ns, int64_t delay_ns, int64_t sample_us, uint32_t generation);
    // Invoke from task context at least every clock_bridge_interval_us, not RX callback.
    void clock_tick();
    const char *clock_state_name(ClockState state);
}
