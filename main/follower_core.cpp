// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#include "follower_core.h"
#include "udp_transport.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tuple>

namespace follower
{
    namespace
    {
        constexpr int64_t maximum = std::numeric_limits<int64_t>::max();
        constexpr int64_t minimum = std::numeric_limits<int64_t>::min();
        constexpr int64_t q16 = 65536, q17 = 131072;
        bool plus(int64_t a, int64_t b, int64_t &out)
        {
            if ((b > 0 && a > maximum - b) || (b < 0 && a < minimum - b))
                return false;
            out = a + b;
            return true;
        }
        bool minus(int64_t a, int64_t b, int64_t &out)
        {
            if ((b > 0 && a < minimum + b) || (b < 0 && a > maximum + b))
                return false;
            out = a - b;
            return true;
        }
        bool times(int64_t a, int64_t b, int64_t &out)
        {
            if (a == 0 || b == 0)
            {
                out = 0;
                return true;
            }
            if (a > 0 ? (b > 0 ? a > maximum / b : b < minimum / a)
                      : (b > 0 ? a < minimum / b : a < maximum / b))
                return false;
            out = a * b;
            return true;
        }
        bool fresh(int64_t now, int64_t then, int64_t age)
        {
            int64_t elapsed = 0;
            return now >= 0 && then >= 0 && age > 0 && minus(now, then, elapsed) &&
                   elapsed >= 0 && elapsed < age;
        }
        template <size_t N>
        bool nonzero(const std::array<uint8_t, N> &bytes)
        {
            return std::any_of(bytes.begin(), bytes.end(), [](uint8_t b)
                               { return b != 0; });
        }
        bool identity_valid(const Identity &id)
        {
            return std::any_of(id.begin(), id.begin() + 8, [](uint8_t b)
                               { return b != 0; }) &&
                   (id[8] != 0 || id[9] != 0) && !(id[8] == 255 && id[9] == 255);
        }
        uint16_t u16(const uint8_t *p) { return uint16_t((uint16_t(p[0]) << 8) | p[1]); }
        uint32_t u32(const uint8_t *p) { return (uint32_t(u16(p)) << 16) | u16(p + 2); }
        int64_t signed64(const uint8_t *p)
        {
            uint64_t v = (uint64_t(u32(p)) << 32) | u32(p + 4);
            return v <= uint64_t(maximum) ? int64_t(v) : -1 - int64_t(~v);
        }
        Timestamp wire_time(const uint8_t *p)
        {
            return {int64_t((uint64_t(u16(p)) << 32) | u32(p + 2)), u32(p + 6)};
        }
        bool trusted(const Packet &p, const Security &s)
        {
            return s.accepted && s.authenticated == p.signed_packet &&
                   (!p.signed_packet || (p.key_id != 0 && s.key_id == p.key_id && s.spp == p.spp));
        }
        // Exact unsigned 64x64 / 64 without compiler-specific 128-bit integers. This
        // also works on the ESP32's 32-bit C++ target. Rounding is nearest, ties away.
        bool multiply_divide(uint64_t a, uint64_t b, uint64_t divisor, int64_t &result)
        {
            if (!divisor)
                return false;
            const uint64_t mask = 0xffffffffULL;
            uint64_t w0 = (a & mask) * (b & mask);
            uint64_t t = (a >> 32) * (b & mask) + (w0 >> 32);
            uint64_t w1 = t & mask, w2 = t >> 32;
            w1 += (a & mask) * (b >> 32);
            uint64_t hi = (a >> 32) * (b >> 32) + w2 + (w1 >> 32);
            uint64_t lo = (w1 << 32) | (w0 & mask);
            if (hi >= divisor)
                return false;
            uint64_t rem = hi, quotient = 0;
            for (int bit = 63; bit >= 0; --bit)
            {
                bool carry = (rem >> 63) != 0;
                rem = (rem << 1) | ((lo >> bit) & 1);
                if (carry || rem >= divisor)
                {
                    rem -= divisor;
                    quotient |= uint64_t(1) << bit;
                }
            }
            if (rem >= divisor - rem)
            {
                if (quotient == std::numeric_limits<uint64_t>::max())
                    return false;
                ++quotient;
            }
            if (quotient > uint64_t(maximum))
                return false;
            result = int64_t(quotient);
            return true;
        }
        int64_t rounded_ns(int64_t value)
        {
            int64_t whole = value / q17, fraction = value % q17;
            if (fraction >= q17 / 2)
                ++whole;
            if (fraction <= -q17 / 2)
                --whole;
            return whole;
        }
    } // namespace

    bool Timestamp::operator==(const Timestamp &o) const { return seconds == o.seconds && nanoseconds == o.nanoseconds; }
    bool valid(Timestamp t) { return t.nanoseconds < uint32_t(billion); }
    Error add_ns(Timestamp t, int64_t delta, Timestamp &out)
    {
        if (!valid(t))
            return Error::invalid;
        int64_t seconds = 0, nanos = int64_t(t.nanoseconds) + delta % billion;
        if (!plus(t.seconds, delta / billion, seconds))
            return Error::overflow;
        if (nanos < 0)
        {
            nanos += billion;
            if (!plus(seconds, -1, seconds))
                return Error::overflow;
        }
        if (nanos >= billion)
        {
            nanos -= billion;
            if (!plus(seconds, 1, seconds))
                return Error::overflow;
        }
        out = {seconds, uint32_t(nanos)};
        return Error::ok;
    }
    Error difference_ns(Timestamp a, Timestamp b, int64_t &out)
    {
        if (!valid(a) || !valid(b))
            return Error::invalid;
        int64_t seconds = 0, nanos = int64_t(a.nanoseconds) - b.nanoseconds;
        if (!minus(a.seconds, b.seconds, seconds))
            return Error::overflow;
        // Normalize toward zero so a representable boundary result never overflows
        // merely because the seconds product precedes the subsecond cancellation.
        if (seconds > 0 && nanos < 0)
        {
            --seconds;
            nanos += billion;
        }
        if (seconds < 0 && nanos > 0)
        {
            ++seconds;
            nanos -= billion;
        }
        int64_t result = 0;
        if (!times(seconds, billion, result) || !plus(result, nanos, result))
            return Error::overflow;
        out = result;
        return Error::ok;
    }
    bool sequence_after(uint16_t value, uint16_t previous)
    {
        uint16_t delta = uint16_t(value - previous);
        return delta != 0 && delta < 0x8000;
    }
    Error parse_mac_address(const std::string &text, Mac &out)
    {
        if (text.size() != 17)
            return Error::invalid;
        const auto hex = [](char c)
        {
            if (c >= '0' && c <= '9')
                return c - '0';
            if (c >= 'a' && c <= 'f')
                return c - 'a' + 10;
            if (c >= 'A' && c <= 'F')
                return c - 'A' + 10;
            return -1;
        };
        Mac result{};
        for (size_t i = 0; i < result.size(); ++i)
        {
            const size_t offset = i * 3;
            const int hi = hex(text[offset]), lo = hex(text[offset + 1]);
            if (hi < 0 || lo < 0 || (i != 5 && text[offset + 2] != ':'))
                return Error::invalid;
            result[i] = uint8_t(hi * 16 + lo);
        }
        if (!nonzero(result) || (result[0] & 1))
            return Error::invalid;
        out = result;
        return Error::ok;
    }
    bool master_mac_collision(const Packet &packet, const Mac &local)
    {
        return packet.source_mac == local &&
               (packet.type == MessageType::announce || packet.type == MessageType::sync ||
                packet.type == MessageType::follow_up || packet.type == MessageType::delay_resp);
    }
    bool TimeProperties::operator==(const TimeProperties &o) const
    {
        return std::tie(utc_offset, utc_valid, ptp_timescale, leap61, leap59, time_traceable, frequency_traceable) ==
               std::tie(o.utc_offset, o.utc_valid, o.ptp_timescale, o.leap61, o.leap59, o.time_traceable, o.frequency_traceable);
    }
    bool TimeProperties::usable() const { return utc_valid && ptp_timescale && !(leap61 && leap59); }
    bool Dataset::operator==(const Dataset &o) const
    {
        return std::tie(priority1, clock_class, accuracy, variance, priority2, grandmaster) ==
               std::tie(o.priority1, o.clock_class, o.accuracy, o.variance, o.priority2, o.grandmaster);
    }
    bool Security::operator==(const Security &o) const
    {
        return std::tie(accepted, authenticated, spp, key_id) == std::tie(o.accepted, o.authenticated, o.spp, o.key_id);
    }

    Error parse_ptp(const uint8_t *frame, size_t length, const ParseConfig &config, Packet &out)
    {
        if (!frame)
            return Error::invalid;
        if (length < 14)
            return Error::truncated;
        udp::Datagram datagram;
        const auto decoded = udp::decode(frame, length, config.ipv6, datagram);
        if (decoded == udp::Result::ignored)
            return Error::wrong_destination;
        if (decoded != udp::Result::valid)
            return Error::invalid;
        Mac source{};
        std::copy_n(frame + 6, 6, source.begin());
        if (!nonzero(source) || (source[0] & 1))
            return Error::invalid;
        const uint8_t *p = datagram.message;
        if ((p[0] & 0xf0) != 0 || (p[1] & 0x0f) != 2 || (p[1] >> 4) > 1)
            return Error::unsupported;
        size_t message_length = u16(p + 2);
        if (message_length < 34)
            return Error::invalid;
        if (message_length > datagram.length)
            return Error::truncated;
        if (p[4] != config.domain)
            return Error::wrong_domain;
        if (p[5] != 0 || u32(p + 16) != 0)
            return Error::unsupported;
        Packet packet;
        packet.type = static_cast<MessageType>(p[0] & 15);
        packet.minor_version = p[1] >> 4;
        packet.message_length = uint16_t(message_length);
        packet.domain = p[4];
        packet.source_mac = source;
        packet.source_ip = datagram.source;
        packet.flags = u16(p + 6);
        packet.correction = signed64(p + 8);
        std::copy_n(p + 20, 10, packet.source.begin());
        if (!identity_valid(packet.source))
            return Error::invalid;
        packet.sequence = u16(p + 30);
        packet.log_interval = p[33] < 128 ? int8_t(p[33]) : int8_t(int(p[33]) - 256);
        size_t base = 0;
        uint8_t control = 0;
        switch (packet.type)
        {
        case MessageType::sync:
            base = 44;
            control = 0;
            break;
        case MessageType::delay_req:
            base = 44;
            control = 1;
            break;
        case MessageType::follow_up:
            base = 44;
            control = 2;
            break;
        case MessageType::delay_resp:
            base = 54;
            control = 3;
            break;
        case MessageType::announce:
            base = 64;
            control = 5;
            break;
        default:
            return Error::unsupported;
        }
        if (message_length < base)
            return Error::truncated;
        if (p[32] != control && !(packet.minor_version == 1 && p[32] == 0))
            return Error::invalid;
        // This deliberately excludes unicast negotiation, alternate masters, Pdelay,
        // profile-specific flags, and security modes outside the immediate subset.
        uint16_t permitted = packet.type == MessageType::announce ? 0x043f : packet.type == MessageType::sync ? 0x0600
                                                                                                              : 0x0400;
        if ((packet.flags & ~permitted) != 0)
            return Error::unsupported;
        if (packet.type == MessageType::sync && !(packet.flags & 0x0200))
            return Error::unsupported;
        packet.timestamp = wire_time(p + 34);
        if (!valid(packet.timestamp))
            return Error::invalid;
        if (packet.type == MessageType::delay_resp)
        {
            std::copy_n(p + 44, 10, packet.requester.begin());
            if (!identity_valid(packet.requester))
                return Error::invalid;
        }
        if (packet.type == MessageType::announce)
        {
            if (p[46] != 0)
                return Error::invalid;
            packet.properties.utc_offset = u16(p + 44) <= 32767 ? int16_t(u16(p + 44)) : int16_t(int(u16(p + 44)) - 65536);
            packet.properties.leap61 = (packet.flags & 1) != 0;
            packet.properties.leap59 = (packet.flags & 2) != 0;
            packet.properties.utc_valid = (packet.flags & 4) != 0;
            packet.properties.ptp_timescale = (packet.flags & 8) != 0;
            packet.properties.time_traceable = (packet.flags & 16) != 0;
            packet.properties.frequency_traceable = (packet.flags & 32) != 0;
            if (packet.properties.leap61 && packet.properties.leap59)
                return Error::invalid;
            packet.dataset.priority1 = p[47];
            packet.dataset.clock_class = p[48];
            packet.dataset.accuracy = p[49];
            packet.dataset.variance = u16(p + 50);
            packet.dataset.priority2 = p[52];
            std::copy_n(p + 53, 8, packet.dataset.grandmaster.begin());
            packet.steps_removed = u16(p + 61);
            if (!nonzero(packet.dataset.grandmaster))
                return Error::invalid;
            if (packet.steps_removed != 0)
                return Error::unsupported;
        }
        if (message_length != base)
            return Error::unsupported;
        out = packet;
        return Error::ok;
    }

    RestrictedFollowerSelection::RestrictedFollowerSelection(SelectionConfig c) : config_(c) {}
    void RestrictedFollowerSelection::reset()
    {
        candidates_ = {};
        selected_ = -1;
        ++generation_;
    }
    const Candidate *RestrictedFollowerSelection::selected() const
    {
        return selected_ < 0 ? nullptr : &candidates_[size_t(selected_)];
    }
    bool RestrictedFollowerSelection::better(const Packet &a, const Packet &b)
    {
        return std::tie(a.dataset.priority1, a.dataset.clock_class, a.dataset.accuracy, a.dataset.variance,
                        a.dataset.priority2, a.dataset.grandmaster, a.source) <
               std::tie(b.dataset.priority1, b.dataset.clock_class, b.dataset.accuracy, b.dataset.variance,
                        b.dataset.priority2, b.dataset.grandmaster, b.source);
    }
    void RestrictedFollowerSelection::elect()
    {
        int next = -1;
        for (size_t i = 0; i < candidates_.size(); ++i)
        {
            const Candidate &c = candidates_[i];
            if (c.occupied && c.evidence >= config_.qualification_count &&
                (next < 0 || better(c.announce, candidates_[size_t(next)].announce)))
                next = int(i);
        }
        if (next != selected_)
        {
            selected_ = next;
            ++generation_;
        }
    }
    bool RestrictedFollowerSelection::expire(int64_t now)
    {
        uint64_t before = generation_;
        for (auto &c : candidates_)
        {
            if (c.occupied && (now < c.last_seen_ns || now >= c.expires_ns))
                c = {};
        }
        elect();
        return generation_ != before;
    }
    Error RestrictedFollowerSelection::announce(const Packet &p, const Security &security, int64_t now)
    {
        if (!trusted(p, security))
            return Error::unauthenticated;
        if (p.type != MessageType::announce || !identity_valid(p.source) || !nonzero(p.dataset.grandmaster) ||
            (p.properties.leap61 && p.properties.leap59) || now < 0)
            return Error::invalid;
        if (p.steps_removed != 0)
            return Error::unsupported;
        if (config_.capacity == 0 || config_.capacity > candidates_.size() || config_.qualification_count == 0 ||
            config_.expiry_intervals == 0 || config_.min_expiry_ns <= 0 || config_.min_log_interval < -30 || config_.max_log_interval > 30 ||
            config_.min_log_interval > config_.max_log_interval)
            return Error::invalid;
        if (p.log_interval < config_.min_log_interval || p.log_interval > config_.max_log_interval)
            return Error::unsupported;
        if (config_.pinned && (p.source != config_.pinned_source ||
                               (config_.pin_mac && p.source_mac != config_.pinned_mac)))
            return Error::mismatch;
        int64_t interval = billion;
        if (p.log_interval >= 0)
        {
            if (!times(interval, int64_t(1) << p.log_interval, interval))
                return Error::overflow;
        }
        else
            interval /= int64_t(1) << -int(p.log_interval);
        int64_t expiry = 0;
        if (!times(interval, config_.expiry_intervals, expiry))
            return Error::overflow;
        expiry = std::max(expiry, config_.min_expiry_ns);
        if (!plus(now, expiry, expiry))
            return Error::overflow;
        // Authentication and all configuration checks precede even expiry mutation.
        expire(now);
        Candidate *slot = nullptr;
        for (size_t i = 0; i < config_.capacity; ++i)
            if (candidates_[i].occupied && candidates_[i].announce.source == p.source)
            {
                slot = &candidates_[i];
                break;
            }
        if (!slot)
        {
            for (size_t i = 0; i < config_.capacity; ++i)
                if (!candidates_[i].occupied)
                {
                    slot = &candidates_[i];
                    break;
                }
            // A newcomer can replace only a worse, unqualified candidate. Qualified
            // candidates (including the active one) are never evicted by one packet.
            if (!slot)
            {
                for (size_t i = 0; i < config_.capacity; ++i)
                {
                    auto &c = candidates_[i];
                    if (c.evidence < config_.qualification_count && better(p, c.announce) &&
                        (!slot || better(slot->announce, c.announce)))
                        slot = &c;
                }
            }
            if (!slot)
                return Error::capacity;
            *slot = {};
        }
        else
        {
            if (p.sequence == slot->announce.sequence)
                return Error::duplicate;
            if (!sequence_after(p.sequence, slot->announce.sequence))
                return Error::stale;
            bool coherent = p.dataset == slot->announce.dataset && p.properties == slot->announce.properties &&
                            security == slot->security && p.source_mac == slot->announce.source_mac &&
                            p.source_ip == slot->announce.source_ip &&
                            p.domain == slot->announce.domain;
            if (!coherent)
            {
                if (selected_ >= 0 && slot == &candidates_[size_t(selected_)])
                {
                    selected_ = -1;
                    ++generation_;
                }
                slot->evidence = 0;
            }
        }
        slot->occupied = true;
        slot->announce = p;
        slot->security = security;
        if (slot->evidence < config_.qualification_count)
            ++slot->evidence;
        slot->last_seen_ns = now;
        slot->expires_ns = expiry;
        elect();
        return Error::ok;
    }

    ExchangeMatcher::ExchangeMatcher(int64_t age, int64_t timeout, size_t capacity)
        : capacity_(capacity), age_(age), timeout_(timeout) {}
    void ExchangeMatcher::reset()
    {
        pending_ = {};
        selected_ = false;
        have_consumed_ = false;
        delay_active_ = false;
        delay_ = {};
    }
    void ExchangeMatcher::select(Identity master, Identity local, uint8_t domain, uint64_t generation)
    {
        reset();
        master_ = master;
        local_ = local;
        domain_ = domain;
        generation_ = generation;
        selected_ = identity_valid(master) && identity_valid(local) && age_ > 0 && timeout_ > 0 &&
                    capacity_ > 0 && capacity_ <= pending_.size();
    }
    Error ExchangeMatcher::accept(const Packet &p, const Security &s, uint64_t generation) const
    {
        if (!trusted(p, s))
            return Error::unauthenticated;
        if (!selected_)
            return Error::unavailable;
        if (generation != generation_ || p.source != master_)
            return Error::mismatch;
        if (p.domain != domain_)
            return Error::wrong_domain;
        return Error::ok;
    }
    Error ExchangeMatcher::ingest(const Packet &p, const Security &s, Timestamp timestamp, bool is_sync, int64_t now)
    {
        if (now < 0 || !valid(timestamp))
            return Error::invalid;
        if (have_consumed_ && !sequence_after(p.sequence, consumed_))
            return p.sequence == consumed_ ? Error::duplicate : Error::stale;
        Pending *slot = nullptr;
        for (auto &entry : pending_)
        {
            if (entry.used && !fresh(now, entry.created_ns, age_))
                entry = {};
            if (entry.used && entry.sequence == p.sequence)
                slot = &entry;
        }
        if (slot)
        {
            if (!(slot->security == s))
            {
                *slot = {};
                return Error::mismatch;
            }
            if (is_sync ? slot->has_sync : slot->has_follow)
            {
                bool same = is_sync ? slot->t2 == timestamp && slot->sync_correction == p.correction : slot->t1 == timestamp && slot->follow_correction == p.correction;
                if (!same)
                    *slot = {};
                return same ? Error::duplicate : Error::mismatch;
            }
        }
        else
        {
            for (size_t i = 0; i < capacity_; ++i)
                if (!pending_[i].used)
                {
                    slot = &pending_[i];
                    break;
                }
            if (!slot)
                return Error::capacity;
            slot->used = true;
            slot->sequence = p.sequence;
            slot->security = s;
            slot->created_ns = now;
        }
        if (is_sync)
        {
            slot->has_sync = true;
            slot->t2 = timestamp;
            slot->sync_correction = p.correction;
        }
        else
        {
            slot->has_follow = true;
            slot->t1 = timestamp;
            slot->follow_correction = p.correction;
        }
        return Error::ok;
    }
    Error ExchangeMatcher::sync(const Packet &p, const Security &s, Timestamp rx, bool hardware, uint64_t generation, int64_t now)
    {
        Error error = accept(p, s, generation);
        if (error != Error::ok)
            return error;
        if (p.type != MessageType::sync)
            return Error::invalid;
        if (!(p.flags & 0x0200))
            return Error::unsupported;
        if (!hardware || !valid(rx))
        {
            for (auto &entry : pending_)
                if (entry.used && entry.sequence == p.sequence)
                    entry = {};
            return Error::unavailable;
        }
        return ingest(p, s, rx, true, now);
    }
    Error ExchangeMatcher::follow_up(const Packet &p, const Security &s, uint64_t generation, int64_t now)
    {
        Error error = accept(p, s, generation);
        if (error != Error::ok)
            return error;
        if (p.type != MessageType::follow_up)
            return Error::invalid;
        return ingest(p, s, p.timestamp, false, now);
    }
    Error ExchangeMatcher::take_pair(int64_t now, Pair &out)
    {
        Pending *best = nullptr;
        for (auto &entry : pending_)
        {
            if (entry.used && (!fresh(now, entry.created_ns, age_) ||
                               (have_consumed_ && !sequence_after(entry.sequence, consumed_))))
                entry = {};
            if (entry.used && entry.has_sync && entry.has_follow &&
                (!best || sequence_after(entry.sequence, best->sequence)))
                best = &entry;
        }
        if (!best)
            return Error::unavailable;
        int64_t correction = 0;
        if (!plus(best->sync_correction, best->follow_correction, correction))
        {
            *best = {};
            return Error::overflow;
        }
        Pair pair{best->t1, best->t2, correction, best->created_ns, generation_, best->sequence, best->security};
        consumed_ = best->sequence;
        have_consumed_ = true;
        for (auto &entry : pending_)
            if (entry.used && !sequence_after(entry.sequence, consumed_))
                entry = {};
        out = pair;
        return Error::ok;
    }
    Error ExchangeMatcher::begin_delay(uint16_t sequence, Timestamp tx, bool hardware, const Pair &pair,
                                       uint64_t generation, int64_t now)
    {
        if (!selected_)
            return Error::unavailable;
        if (generation != generation_ || pair.generation != generation_)
            return Error::mismatch;
        if (!hardware)
        {
            delay_active_ = false;
            return Error::unavailable;
        }
        if (!valid(tx) || !valid(pair.t1) || !valid(pair.t2))
            return Error::invalid;
        if (!fresh(now, pair.captured_ns, age_))
            return Error::stale;
        if (delay_active_ && fresh(now, delay_started_, timeout_))
            return Error::unavailable;
        int64_t elapsed = 0;
        Error error = difference_ns(tx, pair.t2, elapsed);
        if (error != Error::ok)
            return error;
        if (elapsed < 0)
            return Error::invalid;
        delay_ = {};
        delay_.sync = pair;
        delay_.t3 = tx;
        delay_sequence_ = sequence;
        delay_started_ = now;
        delay_active_ = true;
        return Error::ok;
    }
    Error ExchangeMatcher::delay_response(const Packet &p, const Security &s, uint64_t generation, int64_t now, Exchange &out)
    {
        Error error = accept(p, s, generation);
        if (error != Error::ok)
            return error;
        if (p.type != MessageType::delay_resp || !valid(p.timestamp))
            return Error::invalid;
        if (!delay_active_)
            return Error::unavailable;
        if (!fresh(now, delay_started_, timeout_) || !fresh(now, delay_.sync.captured_ns, age_))
        {
            delay_active_ = false;
            return Error::stale;
        }
        if (p.sequence != delay_sequence_ || p.requester != local_)
            return Error::mismatch;
        if (!(s == delay_.sync.security))
            return Error::mismatch;
        delay_.t4 = p.timestamp;
        delay_.delay_correction = p.correction;
        delay_.completed_ns = now;
        delay_active_ = false;
        out = delay_;
        return Error::ok;
    }

    int64_t Measurement::offset_ns() const { return rounded_ns(offset_q17); }
    int64_t Measurement::delay_ns() const { return rounded_ns(delay_q17); }
    Error calculate_e2e(const Exchange &e, int64_t now, Measurement &out, int64_t age, int64_t max_delay, int64_t rate)
    {
        if (age <= 0 || max_delay < 0 || rate <= -billion || rate > billion)
            return Error::invalid;
        if (!fresh(now, e.sync.captured_ns, age) || !fresh(now, e.completed_ns, age) ||
            e.completed_ns < e.sync.captured_ns)
            return Error::stale;
        int64_t a = 0, b = 0, elapsed = 0;
        Error error = difference_ns(e.sync.t2, e.sync.t1, a);
        if (error != Error::ok)
            return error;
        error = difference_ns(e.t4, e.t3, b);
        if (error != Error::ok)
            return error;
        error = difference_ns(e.t3, e.sync.t2, elapsed);
        if (error != Error::ok)
            return error;
        if (elapsed < 0 || elapsed > age)
            return Error::invalid;
        if (!times(a, q16, a) || !times(b, q16, b) ||
            !minus(a, e.sync.correction, a) || !minus(b, e.delay_correction, b))
            return Error::overflow;
        if (rate)
        {
            int64_t elapsed_q16 = 0, adjustment = 0;
            if (!times(elapsed, q16, elapsed_q16) ||
                !multiply_divide(uint64_t(elapsed_q16), uint64_t(rate < 0 ? -rate : rate), uint64_t(billion + rate), adjustment))
                return Error::overflow;
            if (!plus(b, rate < 0 ? -adjustment : adjustment, b))
                return Error::overflow;
        }
        // A and B are q16; their sum/difference are exactly the q17 values of
        // (A+B)/2 and (A-B)/2. No early division discards correction fractions.
        Measurement result;
        if (!plus(a, b, result.delay_q17) || !minus(a, b, result.offset_q17))
            return Error::overflow;
        if (result.delay_q17 < -q17 / 2)
            return Error::invalid;
        if (result.delay_q17 < 0)
            result.delay_q17 = 0; // At most half-ns quantization tolerance.
        if (result.delay_q17 / q17 > max_delay ||
            (result.delay_q17 / q17 == max_delay && result.delay_q17 % q17 != 0))
            return Error::invalid;
        out = result;
        return Error::ok;
    }

    Servo::Servo(ServoConfig config) : config_(config) {}
    void Servo::reset()
    {
        state_ = ServoState::acquiring;
        good_ = bad_ = 0;
        have_sample_ = ever_sample_ = false;
        last_ns_ = 0;
        integral_ = frequency_ = 0;
    }
    void Servo::reset(double frequency_ppm)
    {
        reset();
        const double limit = std::min(config_.max_ppm, config_.hardware_max_ppm);
        if (std::isfinite(frequency_ppm) && std::isfinite(limit) && limit > 0)
            integral_ = frequency_ = std::clamp(frequency_ppm, -limit, limit);
    }
    ServoState Servo::tick(int64_t now, bool inputs_valid)
    {
        if (!inputs_valid || (have_sample_ && !fresh(now, last_ns_, config_.sample_timeout_ns)))
        {
            good_ = bad_ = 0;
            if (state_ == ServoState::locked && config_.holdover_ns > 0 && now >= last_ns_)
                state_ = ServoState::holdover;
            else if (state_ != ServoState::holdover)
                state_ = ServoState::unsynchronized;
            if (state_ == ServoState::holdover)
            {
                int64_t limit = 0;
                if (!plus(config_.sample_timeout_ns, config_.holdover_ns, limit) || !fresh(now, last_ns_, limit))
                    state_ = ServoState::unsynchronized;
            }
        }
        return state_;
    }
    ServoAction Servo::sample(const Measurement &m, int64_t now, bool utc_ready)
    {
        ServoAction action;
        tick(now, utc_ready);
        action.state = state_;
        action.frequency_ppm = frequency_;
        const auto &c = config_;
        if (!utc_ready || now < 0 || c.lock_count == 0 || c.unlock_count == 0 || c.lock_ns < 0 ||
            c.unlock_ns < c.lock_ns || c.step_ns <= c.unlock_ns || c.outlier_ns < c.step_ns ||
            c.sample_timeout_ns <= 0 || c.holdover_ns < 0 || !std::isfinite(c.max_ppm) ||
            !std::isfinite(c.hardware_max_ppm) || !std::isfinite(c.proportional) || !std::isfinite(c.integral) ||
            c.max_ppm <= 0 || c.hardware_max_ppm <= 0 || c.proportional < 0 || c.integral < 0)
            return action;
        if (have_sample_ && now <= last_ns_)
            return action;
        long double offset = static_cast<long double>(m.offset_q17) / q17;
        long double absolute = std::fabs(offset);
        if (m.delay_q17 < 0 || absolute > c.outlier_ns)
        {
            good_ = 0;
            return action;
        }
        bool first = !ever_sample_;
        bool permit_step = first ? c.allow_startup_step : c.allow_recovery_step;
        if (absolute > c.step_ns)
        {
            good_ = bad_ = 0;
            state_ = ServoState::acquiring;
            if (permit_step)
            {
                action.step = true;
                action.step_delta_ns = -m.offset_ns();
                integral_ = frequency_ = 0;
            }
        }
        else
        {
            if (state_ == ServoState::holdover || state_ == ServoState::unsynchronized)
                state_ = ServoState::acquiring;
            double dt = have_sample_ ? double(now - last_ns_) / double(billion) : 1.0;
            // Phase is follower-minus-master: positive error requires a slower PHC.
            // Integrate only when it will not wind farther into actuator saturation.
            double phase = double(offset) / (dt * 1000.0);
            double limit = std::min(c.max_ppm, c.hardware_max_ppm);
            double proposed = std::clamp(integral_ - c.integral * phase, -limit, limit);
            double raw = proposed - c.proportional * phase;
            if ((raw <= limit && raw >= -limit) || (raw > limit && phase > 0) || (raw < -limit && phase < 0))
                integral_ = proposed;
            frequency_ = std::clamp(integral_ - c.proportional * phase, -limit, limit);
            if (absolute <= c.lock_ns)
            {
                if (good_ < c.lock_count)
                    ++good_;
            }
            else
                good_ = 0;
            if (absolute > c.unlock_ns)
            {
                if (bad_ < c.unlock_count)
                    ++bad_;
            }
            else
                bad_ = 0;
            if (state_ == ServoState::locked && bad_ >= c.unlock_count)
            {
                state_ = ServoState::acquiring;
                good_ = 0;
            }
            if (good_ >= c.lock_count)
                state_ = ServoState::locked;
        }
        have_sample_ = ever_sample_ = true;
        last_ns_ = now;
        action.accepted = true;
        action.state = state_;
        action.frequency_ppm = frequency_;
        return action;
    }

    namespace
    {
        std::string configured_zone;
        bool zone_ready = false;
        bool local_calendar(time_t seconds, std::tm &out)
        {
#if defined(_WIN32) && !defined(__MINGW32__)
            return localtime_s(&out, &seconds) == 0;
#else
            return localtime_r(&seconds, &out) != nullptr;
#endif
        }
        bool utc_calendar(time_t seconds, std::tm &out)
        {
#if defined(_WIN32) && !defined(__MINGW32__)
            return gmtime_s(&out, &seconds) == 0;
#else
            return gmtime_r(&seconds, &out) != nullptr;
#endif
        }
        bool to_time_t(int64_t seconds, time_t &out)
        {
            if constexpr (std::numeric_limits<time_t>::is_signed)
            {
                if (seconds < int64_t(std::numeric_limits<time_t>::min()) || seconds > int64_t(std::numeric_limits<time_t>::max()))
                    return false;
            }
            else
            {
                if (seconds < 0 || uint64_t(seconds) > uint64_t(std::numeric_limits<time_t>::max()))
                    return false;
            }
            out = static_cast<time_t>(seconds);
            return true;
        }
        bool same_fields(const std::tm &tm, const CivilTime &c)
        {
            return tm.tm_year == c.year - 1900 && tm.tm_mon == c.month - 1 && tm.tm_mday == c.day &&
                   tm.tm_hour == c.hour && tm.tm_min == c.minute && tm.tm_sec == c.second;
        }
        int64_t calendar_seconds(const std::tm &tm)
        {
            // Gregorian civil days, used only to derive a zone offset from two libc
            // calendars. All timezone rules and DST transitions are still libc-owned.
            int64_t y = int64_t(tm.tm_year) + 1900, m = tm.tm_mon + 1;
            y -= m <= 2;
            int64_t era = (y >= 0 ? y : y - 399) / 400;
            int64_t yoe = y - era * 400;
            int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + tm.tm_mday - 1;
            int64_t days = era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468;
            return days * 86400 + tm.tm_hour * 3600 + tm.tm_min * 60 + tm.tm_sec;
        }
        Error table_valid(OffsetTable table)
        {
            if (!table.intervals || !table.count)
                return Error::unavailable;
            for (size_t i = 0; i < table.count; ++i)
            {
                const auto &v = table.intervals[i];
                if (v.utc_begin >= v.utc_end)
                    return Error::invalid;
                for (size_t j = 0; j < i; ++j)
                    if (v.utc_begin < table.intervals[j].utc_end && table.intervals[j].utc_begin < v.utc_end)
                        return Error::ambiguous;
            }
            return Error::ok;
        }
        Error shift_seconds(Timestamp value, int64_t delta, Timestamp &out)
        {
            int64_t seconds = 0;
            if (!plus(value.seconds, delta, seconds))
                return Error::overflow;
            out = {seconds, value.nanoseconds};
            return Error::ok;
        }
        Error check_zone(const std::string &zone)
        {
            if (!zone_ready)
                return Error::unavailable;
            if (zone != configured_zone)
                return Error::mismatch;
            const char *current = std::getenv("TZ");
            return current && zone == current ? Error::ok : Error::mismatch;
        }
    } // namespace

    Error configure_timezone(const std::string &zone)
    {
        if (zone.empty() || zone.find('\0') != std::string::npos || zone.size() > 255)
            return Error::invalid;
        // Initialization only: once configured, a second zone is rejected rather
        // than racing conversions through a temporary process-global TZ change.
        if (zone_ready)
            return check_zone(zone);
#if defined(_WIN32) && !defined(__MINGW32__)
        // The Microsoft CRT does not implement POSIX rule strings. Never pretend
        // that an ignored rule was honored; native ESP-IDF/newlib uses the path below.
        if (zone.find(',') != std::string::npos || zone.find('<') != std::string::npos)
            return Error::unsupported;
        if (_putenv_s("TZ", zone.c_str()) != 0)
            return Error::unavailable;
        _tzset();
#else
        if (setenv("TZ", zone.c_str(), 1) != 0)
            return Error::unavailable;
        tzset();
#endif
        configured_zone = zone;
        zone_ready = true;
        return Error::ok;
    }
    Error format_time_report(Timestamp tai, Timestamp utc, const std::string &zone, std::string &output)
    {
        if (!valid(tai) || !valid(utc))
            return Error::invalid;
        CivilTime civil;
        const Error error = utc_to_local(utc, zone, civil);
        if (error != Error::ok)
            return error;
        time_t tai_seconds{}, utc_seconds{};
        std::tm tai_tm{}, utc_tm{}, local_tm{};
        if (!to_time_t(tai.seconds, tai_seconds) || !to_time_t(utc.seconds, utc_seconds))
            return Error::overflow;
        if (!utc_calendar(tai_seconds, tai_tm) || !utc_calendar(utc_seconds, utc_tm) ||
            !local_calendar(utc_seconds, local_tm))
            return Error::unavailable;
        char tai_text[32]{}, utc_text[32]{}, local_date[16]{}, abbreviation[64]{};
        if (!std::strftime(tai_text, sizeof(tai_text), "%Y-%m-%dT%H:%M:%S", &tai_tm) ||
            !std::strftime(utc_text, sizeof(utc_text), "%Y-%m-%dT%H:%M:%S", &utc_tm) ||
            !std::strftime(local_date, sizeof(local_date), "%Y-%m-%d", &local_tm) ||
            !std::strftime(abbreviation, sizeof(abbreviation), "%Z", &local_tm))
            return Error::unavailable;
        const int offset = std::abs(civil.utc_offset_seconds);
        std::ostringstream text;
        text << "TAI=" << tai_text << '.' << std::setfill('0') << std::setw(9) << tai.nanoseconds
             << " UTC=" << utc_text << '.' << std::setw(9) << utc.nanoseconds << "Z"
             << " Local=" << local_date << ' ' << (civil.hour % 12 ? civil.hour % 12 : 12)
             << ':' << std::setw(2) << civil.minute << ':' << std::setw(2) << civil.second
             << (civil.hour < 12 ? " AM " : " PM ") << abbreviation
             << " (UTC" << (civil.utc_offset_seconds < 0 ? '-' : '+')
             << std::setw(2) << offset / 3600 << ':' << std::setw(2) << offset % 3600 / 60 << ')';
        output = text.str();
        return Error::ok;
    }
    Error utc_to_tai(Timestamp utc, OffsetTable table, Timestamp &tai)
    {
        if (!valid(utc))
            return Error::invalid;
        Error error = table_valid(table);
        if (error != Error::ok)
            return error;
        for (size_t i = 0; i < table.count; ++i)
        {
            const auto &v = table.intervals[i];
            if (utc.seconds < v.utc_begin || utc.seconds >= v.utc_end)
                continue;
            if (!v.valid)
                return Error::unavailable;
            if (v.leap_ambiguous)
                return Error::ambiguous;
            return shift_seconds(utc, v.tai_minus_utc, tai);
        }
        return Error::unavailable;
    }
    Error tai_to_utc(Timestamp tai, OffsetTable table, Timestamp &utc)
    {
        if (!valid(tai))
            return Error::invalid;
        Error error = table_valid(table);
        if (error != Error::ok)
            return error;
        bool found = false;
        Timestamp result;
        for (size_t i = 0; i < table.count; ++i)
        {
            const auto &v = table.intervals[i];
            Timestamp candidate;
            error = shift_seconds(tai, -int64_t(v.tai_minus_utc), candidate);
            if (error != Error::ok)
                continue;
            if (candidate.seconds < v.utc_begin || candidate.seconds >= v.utc_end)
                continue;
            if (v.leap_ambiguous)
                return Error::ambiguous;
            if (!v.valid)
                return Error::unavailable;
            if (found)
                return Error::ambiguous;
            result = candidate;
            found = true;
        }
        if (!found)
            return Error::unavailable;
        utc = result;
        return Error::ok;
    }
    Error utc_to_local(Timestamp utc, const std::string &zone, CivilTime &local)
    {
        if (!valid(utc))
            return Error::invalid;
        Error error = check_zone(zone);
        if (error != Error::ok)
            return error;
        time_t seconds;
        std::tm l{}, u{};
        if (!to_time_t(utc.seconds, seconds) || !local_calendar(seconds, l) || !utc_calendar(seconds, u))
            return Error::unsupported;
        int64_t offset = calendar_seconds(l) - calendar_seconds(u);
        if (offset < std::numeric_limits<int>::min() || offset > std::numeric_limits<int>::max())
            return Error::overflow;
        CivilTime result;
        result.year = l.tm_year + 1900;
        result.month = l.tm_mon + 1;
        result.day = l.tm_mday;
        result.hour = l.tm_hour;
        result.minute = l.tm_min;
        result.second = l.tm_sec;
        result.nanoseconds = utc.nanoseconds;
        result.utc_offset_seconds = int(offset);
        result.daylight = l.tm_isdst > 0;
        result.zone = zone;
        local = result;
        return Error::ok;
    }
    Error local_to_utc(const CivilTime &c, DstPolicy policy, Timestamp &utc)
    {
        Error error = check_zone(c.zone);
        if (error != Error::ok)
            return error;
        if (c.year < 1 || c.year > 9999 || c.month < 1 || c.month > 12 || c.day < 1 || c.day > 31 ||
            c.hour < 0 || c.hour > 23 || c.minute < 0 || c.minute > 59 || c.second < 0 || c.second > 59 ||
            c.nanoseconds >= uint32_t(billion))
            return Error::invalid;
        if (policy != DstPolicy::reject && policy != DstPolicy::earlier && policy != DstPolicy::later)
            return Error::invalid;
        std::array<int64_t, 3> candidates{};
        size_t count = 0;
        for (int dst : {-1, 0, 1})
        {
            std::tm tm{};
            tm.tm_year = c.year - 1900;
            tm.tm_mon = c.month - 1;
            tm.tm_mday = c.day;
            tm.tm_hour = c.hour;
            tm.tm_min = c.minute;
            tm.tm_sec = c.second;
            tm.tm_isdst = dst;
            time_t result = std::mktime(&tm);
            std::tm check{};
            // time_t(-1) is a legitimate instant on POSIX. A round trip, not the
            // sentinel alone, decides validity and rejects mktime normalization.
            if (!local_calendar(result, check) || !same_fields(check, c))
                continue;
            int64_t seconds = static_cast<int64_t>(result);
            if (std::find(candidates.begin(), candidates.begin() + count, seconds) == candidates.begin() + count)
                candidates[count++] = seconds;
        }
        if (!count)
            return Error::invalid;
        if (count > 1 && policy == DstPolicy::reject)
            return Error::ambiguous;
        auto end = candidates.begin() + count;
        int64_t seconds = policy == DstPolicy::later ? *std::max_element(candidates.begin(), end) : *std::min_element(candidates.begin(), end);
        utc = {seconds, c.nanoseconds};
        return Error::ok;
    }
    Error tai_to_local(Timestamp tai, OffsetTable table, const std::string &zone, CivilTime &local)
    {
        Timestamp utc;
        Error error = tai_to_utc(tai, table, utc);
        return error == Error::ok ? utc_to_local(utc, zone, local) : error;
    }
    Error local_to_tai(const CivilTime &c, DstPolicy policy, OffsetTable table, Timestamp &tai)
    {
        Timestamp utc;
        Error error = local_to_utc(c, policy, utc);
        return error == Error::ok ? utc_to_tai(utc, table, tai) : error;
    }
    Error conversion_example(Timestamp utc, OffsetTable table, const std::string &zone, std::string &output)
    {
        Timestamp tai, back, from_local, from_local_tai;
        CivilTime local, tai_local;
        Error error = utc_to_tai(utc, table, tai);
        if (error != Error::ok)
            return error;
        error = tai_to_utc(tai, table, back);
        if (error != Error::ok)
            return error;
        error = utc_to_local(utc, zone, local);
        if (error != Error::ok)
            return error;
        error = local_to_utc(local, DstPolicy::reject, from_local);
        if (error != Error::ok)
            return error;
        error = tai_to_local(tai, table, zone, tai_local);
        if (error != Error::ok)
            return error;
        error = local_to_tai(local, DstPolicy::reject, table, from_local_tai);
        if (error != Error::ok)
            return error;
        std::ostringstream text;
        auto stamp = [&text](Timestamp t)
        { text << t.seconds << '.' << std::setw(9) << std::setfill('0') << t.nanoseconds; };
        auto civil = [&text](const CivilTime &c)
        {
            text << c.year << '-' << std::setw(2) << std::setfill('0') << c.month << '-' << std::setw(2) << c.day
                 << 'T' << std::setw(2) << c.hour << ':' << std::setw(2) << c.minute << ':' << std::setw(2) << c.second
                 << '.' << std::setw(9) << c.nanoseconds << " zone=" << c.zone << " UTC offset seconds=" << c.utc_offset_seconds;
        };
        text << "UTC to TAI: ";
        stamp(utc);
        text << " UTC -> ";
        stamp(tai);
        text << " TAI\nTAI to UTC: ";
        stamp(back);
        text << " UTC\nUTC to local: ";
        civil(local);
        text << "\nLocal to UTC: ";
        stamp(from_local);
        text << " UTC\nTAI to local: ";
        civil(tai_local);
        text << "\nLocal to TAI: ";
        stamp(from_local_tai);
        text << " TAI\nRound trips preserve instant and nanoseconds: "
             << ((back == utc && from_local == utc && from_local_tai == tai) ? "yes" : "no") << '\n';
        output = text.str();
        return Error::ok;
    }

} // namespace follower
