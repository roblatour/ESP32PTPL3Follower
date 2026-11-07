// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include "follower_core.h"

namespace follower
{
    class SyncRate
    {
    public:
        void reset() { *this = SyncRate{}; }

        void observe(const Pair &pair, int64_t max_gap_ns)
        {
            if (!valid(pair.t1) || !valid(pair.t2) || pair.captured_ns < 0 || max_gap_ns < billion)
            {
                reset();
                return;
            }
            if (!have_anchor_ || pair.generation != anchor_.generation)
            {
                reset();
                anchor_ = pair;
                have_anchor_ = true;
                return;
            }
            int64_t master = 0, local = 0;
            if (pair.captured_ns <= anchor_.captured_ns ||
                pair.captured_ns - anchor_.captured_ns > max_gap_ns ||
                !sequence_after(pair.sequence, anchor_.sequence) || !(pair.security == anchor_.security) ||
                difference_ns(pair.t1, anchor_.t1, master) != Error::ok ||
                difference_ns(pair.t2, anchor_.t2, local) != Error::ok || master <= 0 || local <= 0 ||
                (anchor_.correction > 0 && pair.correction < std::numeric_limits<int64_t>::min() + anchor_.correction) ||
                (anchor_.correction < 0 && pair.correction > std::numeric_limits<int64_t>::max() + anchor_.correction))
            {
                reset();
                return;
            }
            const int64_t elapsed = pair.captured_ns - anchor_.captured_ns;
            const double corrected = static_cast<double>(master) +
                                     static_cast<double>(pair.correction - anchor_.correction) / 65536.0;
            if (corrected <= 0 || std::abs(corrected - static_cast<double>(elapsed)) >
                                      static_cast<double>(elapsed) * 0.01 + 1'000'000.0)
            {
                reset();
                return;
            }
            // Require roughly one second, with margin: elapsed is local capture time, so
            // an exact 1 s threshold randomly needed an extra Sync and slowed E2E cadence.
            if (elapsed < billion - billion / 10)
                return;
            const double measured = (static_cast<double>(local) - corrected) / corrected * 1e9;
            if (!std::isfinite(measured) || std::abs(measured) > 1'000'000.0)
            {
                reset();
                return;
            }
            ppb_ = static_cast<int64_t>(std::llround(measured));
            anchor_ = pair;
            ready_ = true;
        }

        bool rate(int64_t now_ns, int64_t max_age_ns, int64_t &ppb) const
        {
            if (!ready_ || max_age_ns <= 0 || now_ns < anchor_.captured_ns ||
                now_ns - anchor_.captured_ns >= max_age_ns)
                return false;
            ppb = ppb_;
            return true;
        }

    private:
        Pair anchor_{};
        int64_t ppb_ = 0;
        bool have_anchor_ = false, ready_ = false;
    };
}
