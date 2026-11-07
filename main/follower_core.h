// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace follower
{

    // Core objects have no internal locks. A single protocol/clock-owner task must
    // serialize mutations. Packet memory is borrowed only for parse_ptp's call.
    constexpr int64_t billion = 1000000000;
    enum class Error
    {
        ok,
        invalid,
        unsupported,
        truncated,
        wrong_domain,
        wrong_destination,
        unauthenticated,
        stale,
        duplicate,
        mismatch,
        unavailable,
        ambiguous,
        overflow,
        capacity
    };
    struct Timestamp
    {
        int64_t seconds = 0; // Unix epoch for UTC; same numeric epoch plus offset for TAI.
        uint32_t nanoseconds = 0;
        bool operator==(const Timestamp &other) const;
    };
    bool valid(Timestamp value);
    Error add_ns(Timestamp value, int64_t delta_ns, Timestamp &result);
    Error difference_ns(Timestamp a, Timestamp b, int64_t &result);
    using Identity = std::array<uint8_t, 10>;
    using Mac = std::array<uint8_t, 6>;
    // Exactly six colon-separated hexadecimal octets; nonzero unicast only.
    // On error the output is unchanged.
    Error parse_mac_address(const std::string &text, Mac &out);
    bool sequence_after(uint16_t value, uint16_t previous);

    enum class MessageType : uint8_t
    {
        sync = 0,
        delay_req = 1,
        follow_up = 8,
        delay_resp = 9,
        announce = 11
    };
    struct TimeProperties
    {
        int16_t utc_offset = 0;
        bool utc_valid = false, ptp_timescale = false, leap61 = false, leap59 = false;
        bool time_traceable = false, frequency_traceable = false;
        bool operator==(const TimeProperties &other) const;
        bool usable() const;
    };
    struct Dataset
    {
        uint8_t priority1 = 128, clock_class = 248, accuracy = 254, priority2 = 128;
        uint16_t variance = 65535;
        std::array<uint8_t, 8> grandmaster{};
        bool operator==(const Dataset &other) const;
    };
    struct Packet
    {
        MessageType type = MessageType::sync;
        Identity source{}, requester{};
        Mac source_mac{};
        std::array<uint8_t, 16> source_ip{};
        uint8_t domain = 0, minor_version = 0;
        uint16_t sequence = 0, flags = 0, message_length = 0;
        int8_t log_interval = 0;
        int64_t correction = 0; // Signed 2^-16 nanoseconds, never truncated by parser.
        Timestamp timestamp{};
        Dataset dataset{};
        TimeProperties properties{};
        uint16_t steps_removed = 0;
        bool signed_packet = false;
        uint8_t spp = 0;
        uint32_t key_id = 0;
    };
    struct ParseConfig
    {
        uint8_t domain = 0;
        Mac local_mac{};
        bool ipv6 = false;
    };
    // The follower never sends master message types, so these cannot be its loopback.
    bool master_mac_collision(const Packet &packet, const Mac &local);
    // Parses complete multicast Ethernet/IP/UDP frames, including IP/UDP checksums.
    // This unsigned-only profile rejects authentication and other trailing TLVs.
    Error parse_ptp(const uint8_t *frame, size_t length, const ParseConfig &config, Packet &out);
    struct Security
    {
        bool accepted = false;
        bool authenticated = false;
        uint8_t spp = 0;
        uint32_t key_id = 0;
        bool operator==(const Security &other) const;
    };
    struct SelectionConfig
    {
        size_t capacity = 8;
        unsigned qualification_count = 2, expiry_intervals = 3;
        int min_log_interval = -7, max_log_interval = 4;
        int64_t min_expiry_ns = 10 * billion;
        bool pinned = false, pin_mac = false;
        Identity pinned_source{};
        Mac pinned_mac{};
    };
    struct Candidate
    {
        bool occupied = false;
        Packet announce{};
        Security security{};
        unsigned evidence = 0;
        int64_t last_seen_ns = 0, expires_ns = 0;
    };
    class RestrictedFollowerSelection
    {
    public:
        explicit RestrictedFollowerSelection(SelectionConfig config = {});
        Error announce(const Packet &packet, const Security &security, int64_t monotonic_ns);
        bool expire(int64_t monotonic_ns);
        const Candidate *selected() const;
        uint64_t generation() const { return generation_; }
        void reset();
        static bool better(const Packet &a, const Packet &b);

    private:
        void elect();
        SelectionConfig config_;
        std::array<Candidate, 32> candidates_{};
        int selected_ = -1;
        uint64_t generation_ = 0;
    };
    struct Pair
    {
        Timestamp t1{}, t2{};
        int64_t correction = 0;
        int64_t captured_ns = 0;
        uint64_t generation = 0;
        uint16_t sequence = 0;
        Security security{};
    };
    struct Exchange
    {
        Pair sync{};
        Timestamp t3{}, t4{};
        int64_t delay_correction = 0, completed_ns = 0;
    };
    class ExchangeMatcher
    {
    public:
        explicit ExchangeMatcher(int64_t max_age_ns = 5 * billion,
                                 int64_t response_timeout_ns = 2 * billion, size_t capacity = 8);
        void select(Identity master, Identity local, uint8_t domain, uint64_t generation);
        void reset();
        // Hardware provenance is mandatory for Sync RX and Delay_Req TX. A changed
        // mapping generation must be installed via select(), canceling old work.
        Error sync(const Packet &, const Security &, Timestamp hardware_rx, bool hardware_valid,
                   uint64_t generation, int64_t now_ns);
        Error follow_up(const Packet &, const Security &, uint64_t generation, int64_t now_ns);
        Error take_pair(int64_t now_ns, Pair &out);
        Error begin_delay(uint16_t sequence, Timestamp hardware_tx, bool hardware_valid,
                          const Pair &, uint64_t generation, int64_t now_ns);
        Error delay_response(const Packet &, const Security &, uint64_t generation,
                             int64_t now_ns, Exchange &out);

    private:
        struct Pending
        {
            bool used = false, has_sync = false, has_follow = false;
            uint16_t sequence = 0;
            Timestamp t1{}, t2{};
            int64_t sync_correction = 0, follow_correction = 0, created_ns = 0;
            Security security{};
        };
        Error accept(const Packet &, const Security &, uint64_t generation) const;
        Error ingest(const Packet &, const Security &, Timestamp, bool is_sync, int64_t);
        std::array<Pending, 32> pending_{};
        size_t capacity_;
        Identity master_{}, local_{};
        uint8_t domain_ = 0;
        uint64_t generation_ = 0;
        int64_t age_, timeout_;
        bool selected_ = false, have_consumed_ = false, delay_active_ = false;
        uint16_t consumed_ = 0, delay_sequence_ = 0;
        int64_t delay_started_ = 0;
        Exchange delay_{};
    };
    struct Measurement
    {
        // Results retain correction fractions exactly at 2^-17 ns resolution.
        int64_t offset_q17 = 0, delay_q17 = 0;
        int64_t offset_ns() const;
        int64_t delay_ns() const;
    };
    // Timestamps must share continuous TAI and mapping generation. Rate ppb is the
    // follower-minus-master PHC rate; local T3 is mapped back to T2's master rate.
    Error calculate_e2e(const Exchange &, int64_t now_ns, Measurement &out,
                        int64_t max_age_ns = 5 * billion,
                        int64_t max_delay_ns = billion, int64_t rate_ppb = 0);

    enum class ServoState
    {
        acquiring,
        locked,
        holdover,
        unsynchronized
    };
    struct ServoConfig
    {
        int64_t lock_ns = 100000, unlock_ns = 500000, step_ns = 100000000;
        int64_t outlier_ns = billion, sample_timeout_ns = 5 * billion, holdover_ns = 0;
        unsigned lock_count = 8, unlock_count = 3;
        double max_ppm = 100, hardware_max_ppm = 100;
        double proportional = 0.25, integral = 0.02;
        bool allow_startup_step = true, allow_recovery_step = true;
    };
    struct ServoAction
    {
        ServoState state = ServoState::acquiring;
        bool step = false, accepted = false;
        int64_t step_delta_ns = 0;
        double frequency_ppm = 0;
    };
    class Servo
    {
    public:
        explicit Servo(ServoConfig config = {});
        ServoAction sample(const Measurement &, int64_t now_ns, bool utc_ready);
        ServoState tick(int64_t now_ns, bool inputs_valid);
        void reset();
        // Restart acquisition but begin from a previously learned frequency correction.
        void reset(double frequency_ppm);
        ServoState state() const { return state_; }
        double integral_ppm() const { return integral_; }

    private:
        ServoConfig config_;
        ServoState state_ = ServoState::acquiring;
        unsigned good_ = 0, bad_ = 0;
        bool have_sample_ = false, ever_sample_ = false;
        int64_t last_ns_ = 0;
        double integral_ = 0, frequency_ = 0;
    };
    // UTC is POSIX seconds; leap seconds themselves are deliberately unrepresentable.
    // Each offset covers [utc_begin,utc_end), not arbitrary history/future. Callers
    // exclude ambiguous leap windows. All conversion calls preserve nanoseconds,
    // never set clocks, and never change TZ. Configure TZ once before concurrent use.
    struct OffsetInterval
    {
        int64_t utc_begin = 0, utc_end = 0;
        int32_t tai_minus_utc = 0;
        bool valid = true, leap_ambiguous = false;
    };
    struct OffsetTable
    {
        const OffsetInterval *intervals = nullptr;
        size_t count = 0;
    };
    enum class DstPolicy
    {
        reject,
        earlier,
        later
    };
    struct CivilTime
    {
        int year = 1970, month = 1, day = 1, hour = 0, minute = 0, second = 0;
        uint32_t nanoseconds = 0;
        int utc_offset_seconds = 0;
        bool daylight = false;
        std::string zone; // Exact configured POSIX TZ string, not an ambiguous abbreviation.
    };
    Error configure_timezone(const std::string &posix_zone);
    Error utc_to_tai(Timestamp utc, OffsetTable, Timestamp &tai);
    Error tai_to_utc(Timestamp tai, OffsetTable, Timestamp &utc);
    Error utc_to_local(Timestamp utc, const std::string &zone, CivilTime &local);
    Error local_to_utc(const CivilTime &, DstPolicy, Timestamp &utc);
    Error tai_to_local(Timestamp tai, OffsetTable, const std::string &zone, CivilTime &local);
    Error local_to_tai(const CivilTime &, DstPolicy, OffsetTable, Timestamp &tai);
    // Callable six-direction example returns labeled output without IO/clock writes.
    Error conversion_example(Timestamp utc, OffsetTable, const std::string &zone, std::string &output);
    // Read-only reporter formatting: TAI and UTC use calendar+nanoseconds; local
    // uses the server's unpadded 12-hour style, AM/PM, zone and numeric UTC offset.
    // Configure TZ first. Errors leave output unchanged; no clock writes or IO.
    Error format_time_report(Timestamp tai, Timestamp utc, const std::string &zone, std::string &output);

} // namespace follower
