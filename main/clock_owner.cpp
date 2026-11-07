// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#include "clock_owner.h"

#include <algorithm>
#include <cmath>
#include <climits>
#include <limits>
#include <sys/time.h>
#include <sys/timex.h>
#include "ESP32PTPL3FollowerSettings.h"
#include "follower_core.h"
#include "esp_eth_clock.h"
#include "esp_eth_mac_esp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"

namespace follower
{
    namespace
    {
        constexpr int64_t Ns = billion;
        constexpr int64_t Q17 = 1LL << 17;
        // Measurements complete every two or more Sync intervals, so this must not be
        // derived from the Sync freshness limit; doing so made LOCKED flap on jitter.
        constexpr int64_t sample_timeout_us = settings::measurement_timeout_us;
        static_assert(sample_timeout_us > 0 && sample_timeout_us <= INT64_MAX / 1000);
        static_assert(settings::holdover_duration_us >= 0 && settings::holdover_duration_us <= INT64_MAX / 1000);
        static_assert(settings::clock_bridge_interval_us > 0 && settings::utc_metadata_validity_us > 0);
        static_assert(settings::max_clock_bridge_error_ns >= 1000);
        static_assert(settings::max_clock_bridge_read_window_ns > 0 &&
                      settings::max_clock_bridge_read_window_ns / 2 < settings::max_clock_bridge_error_ns);
        static_assert(settings::clock_bridge_read_attempts >= 1 && settings::clock_bridge_read_attempts <= 16);
        static_assert(settings::max_clock_bridge_unverified_us >= settings::clock_bridge_interval_us);
        static_assert(settings::max_mean_path_delay_ns >= 0 && settings::negative_delay_tolerance_ns >= 0);

        SemaphoreHandle_t mutex;
        StaticSemaphore_t mutex_storage;
        ClockSnapshot status;
        bool link_up = false, properties_valid = false, have_sample = false, ever_utc = false;
        int64_t last_sample_us = 0, properties_us = 0, last_bridge_us = 0;
        // Monotonic time of the last precise, in-tolerance PHC/CLOCK_REALTIME comparison.
        int64_t last_bridge_verified_us = -1;
        uint32_t base_addend = 0, current_addend = 0;
        double frequency_limit_ppm = 0;
        // Frequency correction learned while LOCKED (RAM only; relearned after reboot).
        // It mostly reflects this board's oscillator error, so it seeds reacquisition.
        double learned_ppm = 0;
        Servo servo;

        struct Guard
        {
            Guard() { xSemaphoreTake(mutex, portMAX_DELAY); }
            ~Guard() { xSemaphoreGive(mutex); }
        };
        bool add(int64_t a, int64_t b, int64_t &out)
        {
            if ((b > 0 && a > INT64_MAX - b) || (b < 0 && a < INT64_MIN - b))
                return false;
            out = a + b;
            return true;
        }
        bool subtract(int64_t a, int64_t b, int64_t &out)
        {
            if ((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b))
                return false;
            out = a - b;
            return true;
        }
        bool within(int64_t value, int64_t bound) { return value >= -bound && value <= bound; }
        bool fresh(int64_t now, int64_t then, int64_t limit)
        {
            return then >= 0 && now >= then && now - then <= limit;
        }
        bool read_clock(clockid_t clock, int64_t &ns)
        {
            timespec t{};
            if (clock_gettime(clock, &t) != 0 || t.tv_sec < 0 || t.tv_sec > INT64_MAX / Ns || t.tv_nsec < 0 || t.tv_nsec >= Ns)
                return false;
            return add(static_cast<int64_t>(t.tv_sec) * Ns, t.tv_nsec, ns);
        }
        bool phc_read(int64_t &ns)
        {
#if SOC_EMAC_IEEE1588V2_SUPPORTED
            return read_clock(CLOCK_PTP_SYSTEM, ns);
#else
            (void)ns;
            return false;
#endif
        }
        bool metadata_ready(int64_t now)
        {
            return link_up && status.master_tag && properties_valid && fresh(now, properties_us, settings::utc_metadata_validity_us);
        }
        bool sample_ready(int64_t now)
        {
            return have_sample && fresh(now, last_sample_us, sample_timeout_us);
        }
        void revoke(ClockState state, const char *reason)
        {
            if (status.locked)
            {
                ++status.lock_epoch;
                status.unlock_reason = reason;
            }
            status.locked = false;
            status.state = state;
        }
        ClockState acquisition_state()
        {
            return link_up ? (status.master_tag ? ClockState::Acquiring : ClockState::Listening) : ClockState::LinkDown;
        }
        void reset_history()
        {
            servo.reset(learned_ppm);
            have_sample = false;
            last_sample_us = 0;
        }
        void fault()
        {
            revoke(ClockState::Fault, "clock owner fault");
            properties_valid = false;
            status.utc_valid = false;
            learned_ppm = 0;
            reset_history();
        }
        bool set_utc(int64_t ns)
        {
            if (ns < 0 || ns / Ns > std::numeric_limits<time_t>::max())
                return false;
            timeval cancel{};
            if (adjtime(&cancel, nullptr) != 0)
                return false;
            timeval t{static_cast<time_t>(ns / Ns), static_cast<suseconds_t>((ns % Ns) / 1000)};
            if (settimeofday(&t, nullptr) != 0)
                return false;
            ++status.mutations;
            return true;
        }
        bool step_phc(int64_t ns)
        {
#if SOC_EMAC_IEEE1588V2_SUPPORTED
            // IDF's EMAC seconds register is uint32_t, even with a 64-bit time_t.
            if (ns < 0 || ns / Ns > UINT32_MAX || ns / Ns > std::numeric_limits<time_t>::max())
                return false;
            timespec t{static_cast<time_t>(ns / Ns), static_cast<long>(ns % Ns)};
            if (clock_settime(CLOCK_PTP_SYSTEM, &t) != 0)
                return false;
            ++status.mutations;
            return true;
#else
            (void)ns;
            return false;
#endif
        }
        bool configure_servo()
        {
#if SOC_EMAC_IEEE1588V2_SUPPORTED
            // Mirror the installed IDF 6.1 fine-update initialization, including its
            // integer subsecond increment. ADJ_FREQUENCY has no driver-side clamp:
            // its uint32_t addend can wrap. The 5120000 ppb ADJ_OFFSET limit is unrelated.
            const eth_mac_ptp_config_t config = ETH_MAC_ESP_PTP_DEFAULT_CONFIG();
            const double increment = config.roll_type == ETH_PTP_DIGITAL_ROLLOVER
                                         ? config.required_accuracy_ns
                                         : config.required_accuracy_ns / 0.465;
            if (!std::isfinite(increment) || increment < 1 || increment > UINT8_MAX)
                return false;
            const double increment_ns = config.roll_type == ETH_PTP_DIGITAL_ROLLOVER
                                            ? static_cast<uint8_t>(increment)
                                            : static_cast<uint8_t>(increment) * 1e9 / (1ULL << 31);
            const double nominal = (1ULL << 32) * config.clk_src_period_ns / increment_ns;
            if (!std::isfinite(nominal) || nominal < 1 || nominal > UINT32_MAX)
                return false;
            base_addend = current_addend = static_cast<uint32_t>(nominal);
            const double hardware_ppm = std::min((static_cast<double>(UINT32_MAX) / base_addend - 1.0) * 1e6, 999999.0);
            frequency_limit_ppm = std::min({settings::max_frequency_ppm, settings::max_slew_ppm,
                                            settings::servo_integral_limit_ppm, hardware_ppm});
            if (!std::isfinite(frequency_limit_ppm) || frequency_limit_ppm <= 0 ||
                !std::isfinite(settings::servo_phase_gain) || settings::servo_phase_gain < 0 ||
                !std::isfinite(settings::servo_integral_gain) || settings::servo_integral_gain < 0)
                return false;
            ServoConfig servo_config;
            servo_config.lock_ns = settings::lock_offset_threshold_ns;
            servo_config.unlock_ns = settings::unlock_offset_threshold_ns;
            servo_config.step_ns = settings::reacquire_step_threshold_ns;
            servo_config.outlier_ns = settings::servo_outlier_limit_ns;
            servo_config.sample_timeout_ns = sample_timeout_us * 1000;
            servo_config.holdover_ns = settings::holdover_duration_us * 1000;
            servo_config.lock_count = settings::lock_sample_count;
            servo_config.unlock_count = settings::unlock_sample_count;
            // The current core exposes one PI saturation limit. Conservatively bound
            // the entire output by the configured integral limit as well.
            servo_config.max_ppm = frequency_limit_ppm;
            servo_config.hardware_max_ppm = hardware_ppm;
            servo_config.proportional = settings::servo_phase_gain;
            servo_config.integral = settings::servo_integral_gain;
            servo_config.allow_startup_step = settings::allow_initial_step;
            servo_config.allow_recovery_step = settings::allow_recovery_step;
            servo = Servo(servo_config);
            return true;
#else
            return false;
#endif
        }
        bool rate(double absolute_ppm)
        {
#if SOC_EMAC_IEEE1588V2_SUPPORTED
            if (!base_addend || !current_addend || !std::isfinite(absolute_ppm) || std::abs(absolute_ppm) > frequency_limit_ppm)
                return false;
            // esp_eth_clock.c -> emac_hal_ptp_adj_freq multiplies CURRENT addend by
            // (1 + relative_ppb/1e9), truncating. Track that truncation rather than
            // accumulating error by treating the last requested rate as physical rate.
            const double target = base_addend * (1.0 + absolute_ppm / 1e6);
            const double relative = (target / current_addend - 1.0) * 1e9;
            if (!std::isfinite(relative) || relative < INT32_MIN || relative > INT32_MAX)
                return false;
            timex tx{};
            tx.modes = ADJ_FREQUENCY;
            tx.freq = static_cast<long>(std::llround(relative));
            const int64_t next = static_cast<int64_t>(current_addend) * (Ns + tx.freq) / Ns;
            if (next <= 0 || next > UINT32_MAX)
                return false;
            if (tx.freq && clock_adjtime(CLOCK_PTP_SYSTEM, &tx) != 0)
                return false;
            current_addend = static_cast<uint32_t>(next);
            status.frequency_ppb = static_cast<int32_t>(std::llround((static_cast<double>(current_addend) / base_addend - 1.0) * 1e9));
            if (tx.freq)
                ++status.mutations;
            return true;
#else
            (void)absolute_ppm;
            return false;
#endif
        }
        // Bracket CLOCK_REALTIME with PHC reads so sequential API latency is not
        // mistaken for UTC bridge error. The midpoint is an estimate, not HW accuracy.
        // If the reading task is preempted inside the bracket (higher-priority tasks or
        // interrupts), the midpoint is off by up to half the preemption, which easily
        // exceeds max_clock_bridge_error_ns. Such a bracket measures scheduling, not the
        // clocks, so it is retried and, if no tight bracket is found, reported as uncertain.
        enum class BridgeRead
        {
            failed,
            uncertain,
            precise
        };
        BridgeRead mapping(int64_t &tai, int64_t &utc, int64_t &error)
        {
            BridgeRead result = BridgeRead::failed;
            int64_t best_width = INT64_MAX;
            for (unsigned attempt = 0; attempt < settings::clock_bridge_read_attempts; ++attempt)
            {
                int64_t first = 0, last = 0, realtime = 0, middle = 0, expected = 0, difference = 0;
                if (!phc_read(first) || !read_clock(CLOCK_REALTIME, realtime) || !phc_read(last) || last < first)
                    return BridgeRead::failed;
                middle = first + (last - first) / 2;
                if (!subtract(middle, static_cast<int64_t>(status.utc_offset) * Ns, expected) || expected < 0 ||
                    !subtract(expected, realtime, difference))
                    return BridgeRead::failed;
                const int64_t width = last - first;
                if (width < best_width)
                {
                    best_width = width;
                    tai = middle;
                    utc = realtime;
                    error = difference;
                    result = BridgeRead::uncertain;
                }
                if (width <= settings::max_clock_bridge_read_window_ns)
                    return BridgeRead::precise;
            }
            return result;
        }
        // Judge one comparison: a precise read decides; an uncertain one defers to the
        // last precise verification, so a single preempted read can neither lock nor unlock.
        bool bridge_judge(BridgeRead read, int64_t error, int64_t now)
        {
            if (read == BridgeRead::precise)
            {
                status.bridge_error_ns = error;
                if (!within(error, settings::max_clock_bridge_error_ns))
                    return false;
                last_bridge_verified_us = now;
                return true;
            }
            return read == BridgeRead::uncertain && fresh(now, last_bridge_verified_us, settings::max_clock_bridge_unverified_us);
        }
        bool bridge_ok()
        {
            int64_t tai = 0, utc = 0, error = 0;
            const BridgeRead read = mapping(tai, utc, error);
            return bridge_judge(read, error, esp_timer_get_time());
        }
        bool bridge(bool step)
        {
            int64_t tai = 0, utc = 0;
            if (step)
            {
                if (!phc_read(tai) || !subtract(tai, static_cast<int64_t>(status.utc_offset) * Ns, utc) || !set_utc(utc))
                    return false;
                last_bridge_verified_us = esp_timer_get_time();
                status.bridge_error_ns = 0;
            }
            else
            {
                int64_t error = 0;
                const BridgeRead read = mapping(tai, utc, error);
                if (read == BridgeRead::failed)
                    return false;
                // Never steer CLOCK_REALTIME from a preempted comparison: adjtime would slew a
                // false error into UTC and the next check would see it as a real bridge fault.
                // Leave last_bridge_us unchanged so the next tick retries promptly.
                if (read == BridgeRead::uncertain)
                    return true;
                // IDF adjtime internally stores microseconds in long (32-bit on P4 and S31).
                // Never silently step while tracking, nor overflow that conversion.
                const int64_t correction_us = error / 1000;
                if (correction_us < LONG_MIN || correction_us > LONG_MAX)
                    return false;
                timeval correction{static_cast<time_t>(correction_us / 1000000), static_cast<suseconds_t>(correction_us % 1000000)};
                if (adjtime(&correction, nullptr) != 0)
                    return false;
                ++status.mutations;
                status.bridge_error_ns = error;
            }
            last_bridge_us = esp_timer_get_time();
            return true;
        }
        bool apply_step(int64_t delta)
        {
            int64_t tai = 0, target = 0, utc = 0;
            if (!phc_read(tai) || !add(tai, delta, target) || target < 0 || target / Ns > UINT32_MAX ||
                !subtract(target, static_cast<int64_t>(status.utc_offset) * Ns, utc) || utc < 0)
                return false;
            revoke(acquisition_state(), "PHC phase step");
            ++status.generation;
            reset_history();
            if (!step_phc(target) || !rate(learned_ppm) || !bridge(true))
            {
                fault();
                return false;
            }
            status.utc_valid = ever_utc = true;
            return true;
        }
        ClockState core_state(ServoState state)
        {
            switch (state)
            {
            case ServoState::locked:
                return ClockState::Locked;
            case ServoState::holdover:
                return ClockState::Holdover;
            case ServoState::unsynchronized:
                return ClockState::Unsynchronized;
            case ServoState::acquiring:
                return acquisition_state();
            }
            return ClockState::Unsynchronized;
        }
        const char *health_reason(int64_t now)
        {
            if (!link_up)
                return "Ethernet link down";
            if (!status.master_tag)
                return "no selected master";
            if (!properties_valid)
                return "selected master's UTC metadata unusable";
            if (!fresh(now, properties_us, settings::utc_metadata_validity_us))
                return "no qualified Announce within utc_metadata_validity_us";
            if (!sample_ready(now))
                return "no accepted E2E measurement within measurement_timeout_us";
            return "servo left LOCKED";
        }
        void check_health(int64_t now)
        {
            if (status.state == ClockState::Fault)
                return;
            const bool ready = metadata_ready(now) && sample_ready(now);
            const auto state = servo.tick(now <= INT64_MAX / 1000 ? now * 1000 : INT64_MAX, ready);
            if (!ready || state != ServoState::locked)
                revoke(core_state(state), health_reason(now));
            if (!link_up)
                revoke(ClockState::LinkDown, "Ethernet link down");
            else if (!status.master_tag)
                revoke(ClockState::Listening, "no selected master");
        }
    }

    esp_err_t clock_owner_init(esp_eth_handle_t ethernet)
    {
        if (mutex)
            return ESP_ERR_INVALID_STATE;
        mutex = xSemaphoreCreateMutexStatic(&mutex_storage);
        if (!mutex)
            return ESP_ERR_NO_MEM;
        Guard guard;
#if SOC_EMAC_IEEE1588V2_SUPPORTED
        if (!configure_servo())
        {
            fault();
            return ESP_ERR_INVALID_ARG;
        }
        const esp_eth_clock_cfg_t config{CLOCK_PTP_SYSTEM};
        const esp_err_t error = esp_eth_clock_init(ethernet, &config);
        if (error != ESP_OK)
        {
            fault();
            return error;
        }
        status.state = ClockState::LinkDown;
        return ESP_OK;
#else
        (void)ethernet;
        fault();
        return ESP_ERR_NOT_SUPPORTED;
#endif
    }
    ClockSnapshot clock_snapshot()
    {
        if (!mutex)
            return {};
        Guard guard;
        ClockSnapshot snapshot = status;
        const int64_t before = esp_timer_get_time();
        int64_t error = 0;
        const BridgeRead read = mapping(snapshot.tai_ns, snapshot.utc_ns, error);
        const bool readable = read != BridgeRead::failed;
        const int64_t after = esp_timer_get_time();
        snapshot.monotonic_us = before + (after - before) / 2;
        if (!readable)
            snapshot.utc_valid = false;
        // Snapshots never discipline a clock or do socket/logging work; they only
        // record bridge verification bookkeeping.
        const bool bridged = readable && bridge_judge(read, error, after);
        snapshot.bridge_error_ns = status.bridge_error_ns;
        if (snapshot.locked && (!readable || !metadata_ready(after) || !sample_ready(after) || !bridged))
        {
            snapshot.locked = false;
            snapshot.state = ClockState::Unsynchronized;
            snapshot.unlock_reason = !readable  ? "PHC/CLOCK_REALTIME read failed"
                                     : !bridged ? "PHC-to-UTC bridge error exceeded max_clock_bridge_error_ns"
                                                : health_reason(after);
        }
        return snapshot;
    }
    void clock_link(bool up)
    {
        if (!mutex)
            return;
        Guard guard;
        if (link_up == up)
            return;
        link_up = up;
        if (status.master_tag)
            ++status.generation;
        status.master_tag = 0;
        status.time_traceable = false;
        properties_valid = false;
        reset_history();
        if (status.state != ClockState::Fault)
            revoke(acquisition_state(), up ? "Ethernet link transition" : "Ethernet link down");
    }
    void clock_master(uint64_t tag, bool valid, int16_t offset, bool leap, bool traceable)
    {
        if (!mutex)
            return;
        Guard guard;
        if (status.state == ClockState::Fault)
            return;
        const bool usable = tag && valid && !leap && offset >= 0 && offset <= 1000;
        const bool changed = tag != status.master_tag || usable != properties_valid || offset != status.utc_offset || leap != status.leap_ambiguous;
        properties_us = esp_timer_get_time();
        status.time_traceable = traceable;
        if (!changed)
            return;
        status.master_tag = tag;
        status.utc_offset = offset;
        status.leap_ambiguous = leap;
        properties_valid = usable;
        ++status.generation;
        reset_history();
        revoke(acquisition_state(), "selected master or its UTC metadata changed");
    }
    void clock_invalidate()
    {
        if (!mutex)
            return;
        Guard guard;
        if (status.state == ClockState::Fault)
            return;
        reset_history();
        revoke(link_up && status.master_tag ? ClockState::Unsynchronized : acquisition_state(),
               "PTP exchange invalidated (see last PTP event)");
    }
    bool clock_seed(int64_t master_tai_ns, int64_t received_phc_ns, uint32_t generation)
    {
        if (!mutex)
            return false;
        Guard guard;
        if (status.state == ClockState::Fault || !metadata_ready(esp_timer_get_time()) || generation != status.generation ||
            master_tai_ns < 0 || received_phc_ns < 0 || (ever_utc ? !settings::allow_recovery_step : !settings::allow_initial_step))
            return false;
        int64_t delta = 0;
        if (!subtract(master_tai_ns, received_phc_ns, delta))
            return false;
        // A small Sync-only phase estimate must not replace a completed E2E sample.
        if (ever_utc && within(delta, settings::reacquire_step_threshold_ns))
            return false;
        return apply_step(delta);
    }
    bool clock_measurement(int64_t offset, int64_t delay, int64_t sample_us, uint32_t generation)
    {
        if (!mutex)
            return false;
        Guard guard;
        const int64_t now = esp_timer_get_time();
        if (status.state == ClockState::Fault || !metadata_ready(now) || generation != status.generation ||
            !fresh(now, sample_us, settings::max_exchange_age_us) || now > INT64_MAX / 1000 ||
            (have_sample && sample_us <= last_sample_us) || delay < -settings::negative_delay_tolerance_ns || delay > settings::max_mean_path_delay_ns ||
            offset < INT64_MIN / Q17 || offset > INT64_MAX / Q17 || delay > INT64_MAX / Q17)
            return false;
        check_health(now);
        Measurement measurement;
        measurement.offset_q17 = offset * Q17;
        measurement.delay_q17 = std::max<int64_t>(0, delay) * Q17;
        const auto action = servo.sample(measurement, sample_us * 1000, ever_utc);
        if (!action.accepted)
            return false;
        status.offset_ns = offset;
        status.delay_ns = delay;
        if (action.step)
        {
            if (ever_utc ? !settings::allow_recovery_step : !settings::allow_initial_step)
                return false;
            apply_step(action.step_delta_ns);
            return false;
        }
        if (!rate(action.frequency_ppm))
        {
            fault();
            return false;
        }
        // If initial stepping is disabled, establish UTC only once the PHC has
        // actually converged; normal acquisition remains governed by core Servo.
        if (!status.utc_valid && within(offset, settings::lock_offset_threshold_ns))
        {
            revoke(acquisition_state(), "UTC established");
            ++status.generation;
            reset_history();
            if (!bridge(true))
            {
                fault();
                return false;
            }
            status.utc_valid = ever_utc = true;
            return false;
        }
        have_sample = true;
        last_sample_us = sample_us;
        if (status.utc_valid && fresh(now, last_bridge_us, settings::clock_bridge_interval_us) == false && !bridge(false))
        {
            fault();
            return false;
        }
        const bool bridged = status.utc_valid && bridge_ok();
        if (action.state == ServoState::locked && bridged && sample_ready(now))
        {
            status.locked = true;
            status.state = ClockState::Locked;
            learned_ppm = servo.integral_ppm();
        }
        else
        {
            revoke(action.state == ServoState::locked ? ClockState::Acquiring : core_state(action.state),
                   action.state != ServoState::locked ? "servo offset exceeded unlock_offset_threshold_ns"
                   : !bridged                         ? "PHC-to-UTC bridge error exceeded max_clock_bridge_error_ns"
                                                      : health_reason(now));
        }
        return true;
    }
    void clock_tick()
    {
        if (!mutex)
            return;
        Guard guard;
        const int64_t now = esp_timer_get_time();
        check_health(now);
        if (status.state == ClockState::Fault)
            return;
        if (status.utc_valid && metadata_ready(now) && !fresh(now, last_bridge_us, settings::clock_bridge_interval_us))
        {
            if (!bridge(false))
            {
                fault();
                return;
            }
        }
        if (status.locked && !bridge_ok())
        {
            revoke(ClockState::Acquiring, "PHC-to-UTC bridge error exceeded max_clock_bridge_error_ns");
            reset_history();
        }
    }
    const char *clock_state_name(ClockState state)
    {
        switch (state)
        {
        case ClockState::Starting:
            return "STARTING";
        case ClockState::LinkDown:
            return "LINK_DOWN";
        case ClockState::Listening:
            return "LISTENING";
        case ClockState::Acquiring:
            return "ACQUIRING";
        case ClockState::Locked:
            return "LOCKED";
        case ClockState::Holdover:
            return "HOLDOVER";
        case ClockState::Unsynchronized:
            return "UNSYNCHRONIZED";
        case ClockState::Fault:
            return "FAULT";
        }
        return "FAULT";
    }
}
