#include "../main/follower_core.h"

#include "../main/sync_rate.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
using namespace follower;
constexpr int64_t q16 = 65536;
constexpr int64_t q17 = 131072;
constexpr int64_t high = std::numeric_limits<int64_t>::max();
constexpr int64_t low = std::numeric_limits<int64_t>::min();
int checks = 0;
int failures = 0;

void check(bool result, const char* expression, const char* function, int line) {
    ++checks;
    if (!result) {
        ++failures;
        std::cerr << function << ':' << line << ": " << expression << '\n';
    }
}
#define CHECK(...) check((__VA_ARGS__), #__VA_ARGS__, __func__, __LINE__)

Identity identity(uint8_t value) {
    Identity result{};
    result[7] = value;
    result[9] = 1;
    return result;
}
Security unsigned_security() { return {true, false, 0, 0}; }
Security signed_security(uint32_t key = 1) { return {true, true, 7, key}; }
Packet packet(MessageType type, uint16_t sequence = 1, uint8_t source = 1) {
    Packet result;
    result.type = type;
    result.sequence = sequence;
    result.source = identity(source);
    result.requester = identity(9);
    result.source_mac = {{2, 0, 0, 0, 0, source}};
    result.flags = type == MessageType::sync ? 0x0200 : 0;
    result.timestamp = {100, 1000};
    result.dataset.grandmaster[7] = source;
    result.properties = {37, true, true, false, false, true, true};
    return result;
}
void timestamp_tests() {
    Timestamp result{12, 34};
    CHECK(valid({0, 999999999}));
    CHECK(!valid({0, 1000000000}));
    CHECK(add_ns({0, 0}, -1, result) == Error::ok);
    CHECK(result == Timestamp{-1, 999999999});
    CHECK(add_ns(result, 2, result) == Error::ok);
    CHECK(result == Timestamp{0, 1});
    CHECK(add_ns({high, 999999999}, 1, result) == Error::overflow);
    CHECK(result == Timestamp{0, 1});
    CHECK(add_ns({low, 0}, -1, result) == Error::overflow);
    CHECK(add_ns({0, 1000000000}, 1, result) == Error::invalid);
    int64_t difference = 0;
    CHECK(difference_ns({9223372037LL, 0}, {0, 145224193}, difference) == Error::ok);
    CHECK(difference == high);
    CHECK(difference_ns({0, 145224192}, {9223372037LL, 0}, difference) == Error::ok);
    CHECK(difference == low);
    CHECK(difference_ns({9223372037LL, 0}, {0, 145224192}, difference) == Error::overflow);
    CHECK(difference_ns({high, 0}, {low, 0}, difference) == Error::overflow);
    CHECK(sequence_after(0, 65535));
    CHECK(!sequence_after(65535, 0));
    CHECK(!sequence_after(7, 7));
    CHECK(!sequence_after(32768, 0));
    CHECK(sequence_after(32767, 0));
    CHECK(!TimeProperties{}.usable());
    CHECK(packet(MessageType::announce).properties.usable());
}

void selection_tests() {
    const Security security = unsigned_security();
    RestrictedFollowerSelection selection;
    auto a = packet(MessageType::announce, 65535);
    CHECK(selection.selected() == nullptr);
    CHECK(selection.announce(a, {}, 0) == Error::unauthenticated);
    CHECK(selection.generation() == 0 && selection.selected() == nullptr);
    CHECK(selection.announce(a, security, 0) == Error::ok && selection.selected() == nullptr);
    CHECK(selection.announce(a, security, 1) == Error::duplicate);
    a.sequence = 65534;
    CHECK(selection.announce(a, security, 2) == Error::stale);
    a.sequence = 0;
    CHECK(selection.announce(a, security, 3) == Error::ok);
    CHECK(selection.selected() && selection.selected()->evidence == 2);
    const auto generation = selection.generation();
    CHECK(!selection.expire(10 * billion + 2));
    CHECK(selection.announce(a, {}, 20 * billion) == Error::unauthenticated);
    CHECK(selection.generation() == generation && selection.selected());
    CHECK(selection.expire(10 * billion + 3));
    CHECK(selection.selected() == nullptr);
    a.sequence = 1;
    CHECK(selection.announce(a, security, 11 * billion) == Error::ok);
    CHECK(selection.selected() == nullptr);
    ++a.sequence;
    CHECK(selection.announce(a, security, 11 * billion + 1) == Error::ok);
    auto b = packet(MessageType::announce, 1, 2);
    b.dataset.priority1 = 1;
    CHECK(selection.announce(b, security, 11 * billion + 2) == Error::ok);
    CHECK(selection.selected() && selection.selected()->announce.source == a.source);
    ++b.sequence;
    CHECK(selection.announce(b, security, 11 * billion + 3) == Error::ok);
    CHECK(selection.selected() && selection.selected()->announce.source == b.source);
    const auto before_change = selection.generation();
    ++b.sequence; ++b.properties.utc_offset;
    CHECK(selection.announce(b, security, 11 * billion + 4) == Error::ok);
    CHECK(selection.generation() > before_change);
    CHECK(selection.selected() && selection.selected()->announce.source == a.source);
    ++b.sequence;
    CHECK(selection.announce(b, security, 11 * billion + 5) == Error::ok);
    CHECK(selection.selected() && selection.selected()->announce.source == b.source);
    for (int field = 0; field < 7; ++field) {
        auto worse = packet(MessageType::announce);
        worse.dataset.priority1 = 100; worse.dataset.clock_class = 100; worse.dataset.accuracy = 100;
        worse.dataset.variance = 100; worse.dataset.priority2 = 100; worse.dataset.grandmaster[7] = 100;
        worse.source = identity(100);
        auto better = worse;
        switch (field) {
        case 0: --better.dataset.priority1; break;
        case 1: --better.dataset.clock_class; break;
        case 2: --better.dataset.accuracy; break;
        case 3: --better.dataset.variance; break;
        case 4: --better.dataset.priority2; break;
        case 5: --better.dataset.grandmaster[7]; break;
        default: --better.source[7]; break;
        }
        CHECK(RestrictedFollowerSelection::better(better, worse));
        CHECK(!RestrictedFollowerSelection::better(worse, better));
        CHECK(!RestrictedFollowerSelection::better(worse, worse));
    }
    SelectionConfig config;
    config.capacity = 1;
    RestrictedFollowerSelection bounded(config);
    a = packet(MessageType::announce);
    CHECK(bounded.announce(a, security, 0) == Error::ok);
    b = packet(MessageType::announce, 1, 2); b.dataset.priority1 = 1;
    CHECK(bounded.announce(b, security, 1) == Error::ok);
    ++b.sequence;
    CHECK(bounded.announce(b, security, 2) == Error::ok);
    a.dataset.priority1 = 0;
    CHECK(bounded.announce(a, security, 3) == Error::capacity);
    config.pinned = true; config.pin_mac = true; config.pinned_source = a.source; config.pinned_mac = a.source_mac;
    RestrictedFollowerSelection pinned(config);
    CHECK(pinned.announce(b, security, 0) == Error::mismatch);
    CHECK(pinned.announce(a, security, 0) == Error::ok);
    ++a.sequence;
    CHECK(pinned.announce(a, security, 1) == Error::ok);
    CHECK(pinned.expire(10 * billion + 1) && !pinned.selected());
    a.source_mac[5] = 99;
    CHECK(pinned.announce(a, security, 11 * billion) == Error::mismatch);
    a = packet(MessageType::announce); a.log_interval = 5;
    CHECK(selection.announce(a, security, 0) == Error::unsupported);
    a.log_interval = 4;
    RestrictedFollowerSelection slow;
    CHECK(slow.announce(a, security, 0) == Error::ok);
    ++a.sequence;
    CHECK(slow.announce(a, security, 1) == Error::ok);
    CHECK(!slow.expire(48 * billion));
    CHECK(slow.expire(48 * billion + 1));
    config = {}; config.capacity = 33;
    CHECK(RestrictedFollowerSelection(config).announce(a, security, 0) == Error::invalid);
    config = {}; config.capacity = 32;
    RestrictedFollowerSelection expanded(config);
    for (uint8_t source = 1; source <= 32; ++source)
        CHECK(expanded.announce(packet(MessageType::announce, 1, source), security, source) == Error::ok);
    CHECK(expanded.announce(packet(MessageType::announce, 1, 33), security, 33) == Error::capacity);
    config = {}; config.expiry_intervals = 5; config.min_expiry_ns = billion;
    RestrictedFollowerSelection custom_expiry(config);
    a.log_interval = 0;
    CHECK(custom_expiry.announce(a, security, 0) == Error::ok);
    ++a.sequence;
    CHECK(custom_expiry.announce(a, security, 1) == Error::ok);
    CHECK(!custom_expiry.expire(5 * billion));
    CHECK(custom_expiry.expire(5 * billion + 1));
    config.expiry_intervals = 0;
    CHECK(RestrictedFollowerSelection(config).announce(a, security, 0) == Error::invalid);
    config = {}; config.qualification_count = 0;
    CHECK(RestrictedFollowerSelection(config).announce(a, security, 0) == Error::invalid);
    a.log_interval = 0;
    CHECK(selection.announce(a, security, high) == Error::overflow);
    selection.reset();
    CHECK(selection.selected() == nullptr);
}

Pair prepare_pair(ExchangeMatcher& matcher, const Security& security = unsigned_security()) {
    auto sync = packet(MessageType::sync, 65535);
    auto follow = packet(MessageType::follow_up, 65535);
    if (security.authenticated) {
        sync.signed_packet = follow.signed_packet = true;
        sync.key_id = follow.key_id = security.key_id;
        sync.spp = follow.spp = security.spp;
    }
    sync.correction = q16 / 4; follow.correction = -q16 / 2;
    CHECK(matcher.follow_up(follow, security, 7, 100) == Error::ok);
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 101) == Error::ok);
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 102) == Error::duplicate);
    Pair pair;
    CHECK(matcher.take_pair(103, pair) == Error::ok);
    CHECK(pair.correction == -q16 / 4 && pair.sequence == 65535 && pair.generation == 7);
    CHECK(pair.security == security);
    return pair;
}

void rate_tests() {
    const auto pair_at = [](int64_t elapsed, int64_t ppb, uint16_t sequence) {
        Pair pair;
        CHECK(add_ns({100, 0}, elapsed, pair.t1) == Error::ok);
        CHECK(add_ns({100, 1000}, elapsed + elapsed / billion * ppb, pair.t2) == Error::ok);
        pair.captured_ns = elapsed;
        pair.sequence = sequence;
        pair.generation = 7;
        pair.security = unsigned_security();
        return pair;
    };
    for (int64_t ppb : {int64_t(-50000), int64_t(0), int64_t(20000), int64_t(100000)}) {
        SyncRate rate;
        int64_t measured = high;
        CHECK(!rate.rate(0, 5 * billion, measured));
        rate.observe(pair_at(0, ppb, 65535), 5 * billion);
        CHECK(!rate.rate(0, 5 * billion, measured));
        rate.observe(pair_at(billion, ppb, 0), 5 * billion);
        CHECK(rate.rate(billion, 5 * billion, measured));
        CHECK(measured == ppb);
        CHECK(!rate.rate(6 * billion, 5 * billion, measured));
        CHECK(!rate.rate(billion - 1, 5 * billion, measured));
        rate.reset();
        CHECK(!rate.rate(billion, 5 * billion, measured));
    }
    for (int variant = 0; variant < 6; ++variant) {
        SyncRate rate;
        int64_t measured = 0;
        rate.observe(pair_at(0, 20000, 1), 5 * billion);
        rate.observe(pair_at(billion, 20000, 2), 5 * billion);
        CHECK(rate.rate(billion, 5 * billion, measured));
        auto next = pair_at(2 * billion, 20000, 3);
        if (variant == 0) ++next.generation;
        if (variant == 1) next.security = signed_security();
        if (variant == 2) next.sequence = 1;
        if (variant == 3) next.captured_ns = 7 * billion;
        if (variant == 4) next.t1.nanoseconds = 1'000'000'000;
        if (variant == 5) next.t2.seconds += 1;
        rate.observe(next, 5 * billion);
        CHECK(!rate.rate(next.captured_ns, 5 * billion, measured));
    }
    SyncRate corrected;
    auto anchor = pair_at(0, 0, 1);
    corrected.observe(anchor, 5 * billion);
    auto later = pair_at(billion, 0, 2);
    later.correction = 50000 * q16;
    later.t2.nanoseconds += 50000;
    corrected.observe(later, 5 * billion);
    int64_t measured = high;
    CHECK(corrected.rate(billion, 5 * billion, measured) && measured == 0);
}

void matcher_tests() {
    const auto security = unsigned_security();
    ExchangeMatcher matcher;
    matcher.select(identity(1), identity(9), 0, 7);
    Pair pair = prepare_pair(matcher);
    CHECK(matcher.take_pair(104, pair) == Error::unavailable);
    auto sync = packet(MessageType::sync, 65535);
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 104) == Error::duplicate);
    sync.sequence = 65534;
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 104) == Error::stale);
    sync.sequence = 0;
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 104) == Error::ok);
    CHECK(matcher.sync(sync, security, {100, 2001}, true, 7, 104) == Error::mismatch);
    CHECK(matcher.take_pair(105, pair) == Error::unavailable);
    CHECK(matcher.begin_delay(10, {100, 3000}, false, pair, 7, 105) == Error::unavailable);
    CHECK(matcher.begin_delay(10, {100, 3000}, true, pair, 8, 105) == Error::mismatch);
    CHECK(matcher.begin_delay(10, {100, 3000}, true, pair, 7, 105) == Error::ok);
    CHECK(matcher.begin_delay(11, {100, 3000}, true, pair, 7, 106) == Error::unavailable);
    auto response = packet(MessageType::delay_resp, 10);
    Exchange exchange;
    response.requester[8] = 1;
    CHECK(matcher.delay_response(response, security, 7, 106, exchange) == Error::mismatch);
    response.requester = identity(9); response.sequence = 11;
    CHECK(matcher.delay_response(response, security, 7, 106, exchange) == Error::mismatch);
    response.sequence = 10; response.correction = -123;
    CHECK(matcher.delay_response(response, {}, 7, 106, exchange) == Error::unauthenticated);
    CHECK(matcher.delay_response(response, security, 7, 106, exchange) == Error::ok);
    CHECK(exchange.sync.correction == -q16 / 4 && exchange.delay_correction == -123);
    CHECK(matcher.delay_response(response, security, 7, 107, exchange) == Error::unavailable);
    CHECK(matcher.begin_delay(10, {100, 3000}, true, pair, 7, 108) == Error::ok);
    CHECK(matcher.delay_response(response, security, 7, 108 + 2 * billion, exchange) == Error::stale);
    matcher.select(identity(1), identity(9), 0, 8);
    CHECK(matcher.begin_delay(10, {100, 3000}, true, pair, 8, 109) == Error::mismatch);
    CHECK(matcher.follow_up(packet(MessageType::follow_up), security, 7, 109) == Error::mismatch);
    CHECK(matcher.delay_response(response, security, 8, 109, exchange) == Error::unavailable);
    matcher.select(identity(1), identity(9), 0, 7);
    sync = packet(MessageType::sync);
    CHECK(matcher.sync(sync, security, {100, 2000}, false, 7, 0) == Error::unavailable);
    sync.domain = 1;
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 0) == Error::wrong_domain);
    sync.domain = 0; sync.source = identity(2);
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 0) == Error::mismatch);
    sync.source = identity(1);
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 0) == Error::ok);
    CHECK(matcher.follow_up(packet(MessageType::follow_up), security, 7, 1) == Error::ok);
    CHECK(matcher.take_pair(5 * billion, pair) == Error::unavailable);
    matcher.reset();
    CHECK(matcher.follow_up(packet(MessageType::follow_up), security, 7, 1) == Error::unavailable);
    matcher.select(identity(1), identity(9), 0, 7);
    for (uint16_t sequence = 1; sequence <= 8; ++sequence)
        CHECK(matcher.follow_up(packet(MessageType::follow_up, sequence), security, 7, sequence) == Error::ok);
    CHECK(matcher.follow_up(packet(MessageType::follow_up, 9), security, 7, 9) == Error::capacity);
    for (size_t capacity : {size_t(1), size_t(16), size_t(32)}) {
        ExchangeMatcher bounded_pairs(5 * billion, 2 * billion, capacity);
        bounded_pairs.select(identity(1), identity(9), 0, 7);
        for (uint16_t sequence = 1; sequence <= capacity; ++sequence)
            CHECK(bounded_pairs.follow_up(packet(MessageType::follow_up, sequence), security, 7, sequence) == Error::ok);
        CHECK(bounded_pairs.follow_up(packet(MessageType::follow_up, 33), security, 7, 33) == Error::capacity);
    }
    for (size_t capacity : {size_t(0), size_t(33)}) {
        ExchangeMatcher invalid_pairs(5 * billion, 2 * billion, capacity);
        invalid_pairs.select(identity(1), identity(9), 0, 7);
        CHECK(invalid_pairs.follow_up(packet(MessageType::follow_up), security, 7, 0) == Error::unavailable);
    }
    matcher.select(identity(1), identity(9), 0, 7);
    sync = packet(MessageType::sync); sync.correction = high;
    auto follow = packet(MessageType::follow_up); follow.correction = 1;
    CHECK(matcher.sync(sync, security, {100, 2000}, true, 7, 0) == Error::ok);
    CHECK(matcher.follow_up(follow, security, 7, 1) == Error::ok);
    CHECK(matcher.take_pair(2, pair) == Error::overflow);
}

void security_association_regression() {
    ExchangeMatcher matcher;
    matcher.select(identity(1), identity(9), 0, 7);
    const Pair pair = prepare_pair(matcher, signed_security(1));
    CHECK(matcher.begin_delay(10, {100, 3000}, true, pair, 7, 104) == Error::ok);
    auto response = packet(MessageType::delay_resp, 10);
    response.signed_packet = true; response.spp = 7; response.key_id = 2;
    Exchange exchange;
    CHECK(matcher.delay_response(response, signed_security(2), 7, 105, exchange) == Error::mismatch);
    response.key_id = 1;
    CHECK(matcher.delay_response(response, signed_security(1), 7, 106, exchange) == Error::ok);
    CHECK(exchange.sync.security == signed_security(1));

    Security other_spp = signed_security(1);
    ++other_spp.spp;
    const std::array<Security, 4> associations{{unsigned_security(), signed_security(1),
                                                signed_security(2), other_spp}};
    const auto secured_packet = [](MessageType type, uint16_t sequence, const Security& security) {
        auto result = packet(type, sequence);
        result.signed_packet = security.authenticated;
        result.spp = security.spp;
        result.key_id = security.key_id;
        return result;
    };
    for (const auto& first : associations) {
        for (const auto& second : associations) {
            if (first == second) continue;
            for (bool sync_first : {false, true}) {
                matcher.select(identity(1), identity(9), 0, 7);
                auto sync = secured_packet(MessageType::sync, 1, first);
                auto follow = secured_packet(MessageType::follow_up, 1, second);
                if (sync_first) {
                    CHECK(matcher.sync(sync, first, {100, 2000}, true, 7, 100) == Error::ok);
                    CHECK(matcher.follow_up(follow, second, 7, 101) == Error::mismatch);
                } else {
                    CHECK(matcher.follow_up(follow, second, 7, 100) == Error::ok);
                    CHECK(matcher.sync(sync, first, {100, 2000}, true, 7, 101) == Error::mismatch);
                }
                Pair rejected;
                CHECK(matcher.take_pair(102, rejected) == Error::unavailable);
            }

            matcher.select(identity(1), identity(9), 0, 7);
            const Pair original = prepare_pair(matcher, first);
            auto sync = secured_packet(MessageType::sync, 0, second);
            auto follow = secured_packet(MessageType::follow_up, 0, second);
            CHECK(matcher.sync(sync, second, {100, 2000}, true, 7, 104) == Error::ok);
            CHECK(matcher.follow_up(follow, second, 7, 105) == Error::ok);
            Pair rotated;
            CHECK(matcher.take_pair(106, rotated) == Error::ok);
            CHECK(rotated.security == second);
            CHECK(matcher.begin_delay(10, {100, 3000}, true, original, 7, 107) == Error::ok);
            auto wrong = secured_packet(MessageType::delay_resp, 10, second);
            Exchange completed;
            completed.completed_ns = -1;
            CHECK(matcher.delay_response(wrong, second, 7, 108, completed) == Error::mismatch);
            CHECK(completed.completed_ns == -1);
            auto matching = secured_packet(MessageType::delay_resp, 10, first);
            CHECK(matcher.delay_response(matching, first, 7, 109, completed) == Error::ok);
            CHECK(completed.sync.security == first);
            CHECK(matcher.begin_delay(11, {100, 3000}, true, rotated, 7, 110) == Error::ok);
            wrong.sequence = 11;
            CHECK(matcher.delay_response(wrong, second, 7, 111, completed) == Error::ok);
            CHECK(completed.sync.security == second);
        }
    }
}

Exchange synthetic(int64_t offset, int64_t delay) {
    Exchange exchange;
    exchange.sync.t1 = {100, 0};
    CHECK(add_ns(exchange.sync.t1, offset + delay, exchange.sync.t2) == Error::ok);
    exchange.t3 = {101, 0};
    CHECK(add_ns(exchange.t3, delay - offset, exchange.t4) == Error::ok);
    exchange.sync.captured_ns = 100;
    exchange.completed_ns = 200;
    return exchange;
}
void e2e_tests() {
    Measurement result;
    for (int64_t offset : {-10000, 0, 10000}) {
        const auto exchange = synthetic(offset, 20000);
        CHECK(calculate_e2e(exchange, 201, result) == Error::ok);
        CHECK(result.offset_ns() == offset && result.delay_ns() == 20000);
    }
    auto exchange = synthetic(100, 200);
    exchange.sync.correction = q16 / 4;
    exchange.delay_correction = -q16 / 2;
    CHECK(calculate_e2e(exchange, 201, result) == Error::ok);
    CHECK(result.offset_q17 == 100 * q17 - 3 * q16 / 4);
    CHECK(result.delay_q17 == 200 * q17 + q16 / 4);
    CHECK(Measurement{q17 / 2, -q17 / 2}.offset_ns() == 1);
    CHECK(Measurement{-q17 / 2, 0}.offset_ns() == -1);
    exchange = synthetic(0, -1);
    CHECK(calculate_e2e(exchange, 201, result) == Error::invalid);
    exchange = synthetic(0, 0); exchange.sync.correction = q16;
    CHECK(calculate_e2e(exchange, 201, result) == Error::ok && result.delay_q17 == 0);
    ++exchange.sync.correction;
    CHECK(calculate_e2e(exchange, 201, result) == Error::invalid);
    exchange = synthetic(0, 1001);
    CHECK(calculate_e2e(exchange, 201, result, 5 * billion, 1000) == Error::invalid);
    exchange = synthetic(0, 1000); exchange.sync.correction = -1;
    CHECK(calculate_e2e(exchange, 201, result, 5 * billion, 1000) == Error::invalid);
    exchange = synthetic(0, 100);
    CHECK(calculate_e2e(exchange, 100 + 5 * billion, result) == Error::stale);
    CHECK(calculate_e2e(exchange, 199, result) == Error::stale);
    exchange.sync.t2 = {high, 0};
    CHECK(calculate_e2e(exchange, 201, result) == Error::overflow);
    exchange = synthetic(0, 100); exchange.sync.correction = low;
    CHECK(calculate_e2e(exchange, 201, result) == Error::overflow);
    for (int64_t rate : {-100, 100}) {
        exchange = synthetic(100, 100);
        exchange.t3 = {101, uint32_t(200 + rate)};
        exchange.t4 = {101, 200};
        CHECK(calculate_e2e(exchange, 201, result, 5 * billion, billion, rate) == Error::ok);
        CHECK(result.offset_q17 == 100 * q17 && result.delay_q17 == 100 * q17);
    }
    CHECK(calculate_e2e(exchange, 201, result, 5 * billion, billion, -billion) == Error::invalid);
}

Measurement measurement(int64_t offset) { return {offset * q17, 1000 * q17}; }
void servo_tests() {
    Servo servo;
    auto action = servo.sample(measurement(200000000), 0, true);
    CHECK(action.accepted && action.step && action.step_delta_ns == -200000000);
    CHECK(action.state == ServoState::acquiring);
    for (int i = 1; i <= 8; ++i) {
        action = servo.sample(measurement(0), int64_t(i) * billion, true);
        CHECK(action.accepted && !action.step);
        CHECK(action.state == (i == 8 ? ServoState::locked : ServoState::acquiring));
    }
    CHECK(!servo.sample(measurement(0), 8 * billion, true).accepted);
    for (int i = 9; i <= 11; ++i) {
        action = servo.sample(measurement(600000), int64_t(i) * billion, true);
        CHECK(action.state == (i == 11 ? ServoState::acquiring : ServoState::locked));
        CHECK(!action.step && action.frequency_ppm < 0 && std::fabs(action.frequency_ppm) <= 100);
    }
    action = servo.sample(measurement(-200000000), 12 * billion, true);
    CHECK(action.step && action.step_delta_ns == 200000000);
    CHECK(!servo.sample(measurement(1000000001), 13 * billion, true).accepted);
    CHECK(servo.tick(17 * billion, true) == ServoState::unsynchronized);
    servo.reset();
    CHECK(servo.state() == ServoState::acquiring);
    CHECK(!servo.sample(measurement(0), 0, false).accepted);
    {
        Servo seeded;
        seeded.reset(12.5);
        CHECK(seeded.integral_ppm() == 12.5);
        CHECK(seeded.sample(measurement(0), 0, true).frequency_ppm == 12.5);
        seeded.reset(1000);
        CHECK(seeded.integral_ppm() == 100);
        seeded.reset(std::numeric_limits<double>::quiet_NaN());
        CHECK(seeded.integral_ppm() == 0);
    }
    ServoConfig config;
    config.allow_startup_step = false; config.allow_recovery_step = false;
    Servo no_step(config);
    CHECK(!no_step.sample(measurement(200000000), 0, true).step);
    CHECK(!no_step.sample(measurement(-200000000), billion, true).step);
    config = {}; config.hardware_max_ppm = 20;
    Servo bounded(config);
    for (int i = 0; i < 50; ++i) {
        action = bounded.sample(measurement(10000000), int64_t(i) * billion, true);
        CHECK(action.accepted && action.frequency_ppm == -20);
    }
    CHECK(std::fabs(bounded.sample(measurement(0), 50 * billion, true).frequency_ppm) < 0.000001);
    CHECK(bounded.sample(measurement(-10000000), 51 * billion, true).frequency_ppm == 20);
    config = {}; config.lock_count = 1; config.holdover_ns = 2 * billion;
    Servo holdover(config);
    CHECK(holdover.sample(measurement(0), 0, true).state == ServoState::locked);
    CHECK(holdover.tick(5 * billion - 1, true) == ServoState::locked);
    CHECK(holdover.tick(5 * billion, true) == ServoState::holdover);
    CHECK(holdover.tick(7 * billion, true) == ServoState::unsynchronized);
    CHECK(holdover.sample(measurement(0), 8 * billion, true).state == ServoState::locked);
    CHECK(holdover.tick(8 * billion + 1, false) == ServoState::holdover);
    config = {}; config.max_ppm = std::numeric_limits<double>::quiet_NaN();
    CHECK(!Servo(config).sample(measurement(0), 0, true).accepted);
}

void mac_tests() {
    Mac parsed{};
    const Mac expected{{0x82, 0xf1, 0xb2, 0xd2, 0xca, 0x9a}};
    const Mac native{{0x80, 0xf1, 0xb2, 0xd2, 0xca, 0x9a}};
    CHECK(parse_mac_address("82:f1:b2:d2:ca:9a", parsed) == Error::ok && parsed == expected);
    CHECK(parse_mac_address("82:F1:B2:D2:Ca:9A", parsed) == Error::ok && parsed == expected);
    for (const char* text : {"", "82:f1:b2:d2:ca", "82:f1:b2:d2:ca:9a:00",
                             "8:f1:b2:d2:ca:9a", "82-f1-b2-d2-ca-9a",
                             " 82:f1:b2:d2:ca:9a", "82::f1:b2:d2:ca:9a",
                             "82:f1:b2:d2:ca:9", "82:f1:b2:d2:ca:100",
                             "8g:f1:b2:d2:ca:9a",
                             "82:f1:b2:d2:ca:gg", "82:f1:b2:d2:ca:9a ",
                             "00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff",
                             "01:1b:19:00:00:00"}) {
        parsed = native;
        CHECK(parse_mac_address(text, parsed) == Error::invalid);
        CHECK(parsed == native);
    }
    CHECK(parse_mac_address(std::string("82:f1:b2:d2:ca:9a\0", 18), parsed) == Error::invalid);
    CHECK(parsed == native);
    CHECK(parse_mac_address("82:f1:b2:d2:ca:9a", parsed) == Error::ok && parsed == expected);
    for (const auto type : {MessageType::announce, MessageType::sync,
                            MessageType::follow_up, MessageType::delay_resp, MessageType::delay_req}) {
        auto master = packet(type);
        master.source_mac = expected;
        CHECK(master_mac_collision(master, expected) == (type != MessageType::delay_req));
        master.source = identity(9);
        CHECK(master_mac_collision(master, expected) == (type != MessageType::delay_req));
        master.source_mac[0] = 0x80;
        CHECK(!master_mac_collision(master, expected));
    }
}

void offset_table_tests() {
    Timestamp result;
    CHECK(utc_to_tai({0, 0}, {}, result) == Error::unavailable);
    OffsetInterval interval{0, 100, 37, true, false};
    OffsetTable table{&interval, 1};
    CHECK(utc_to_tai({0, 123}, table, result) == Error::ok && result == Timestamp{37, 123});
    CHECK(tai_to_utc(result, table, result) == Error::ok && result == Timestamp{0, 123});
    CHECK(utc_to_tai({100, 0}, table, result) == Error::unavailable);
    CHECK(tai_to_utc({36, 0}, table, result) == Error::unavailable);
    interval.valid = false;
    CHECK(utc_to_tai({10, 0}, table, result) == Error::unavailable);
    interval.valid = true; interval.leap_ambiguous = true;
    CHECK(utc_to_tai({10, 0}, table, result) == Error::ambiguous);
    CHECK(tai_to_utc({47, 0}, table, result) == Error::ambiguous);
    interval = {high - 1, high, 37, true, false};
    CHECK(utc_to_tai({high - 1, 0}, table, result) == Error::overflow);
    interval = {2, 1, 0, true, false};
    CHECK(utc_to_tai({1, 0}, table, result) == Error::invalid);
    OffsetInterval overlap[]{{0, 100, 37, true, false}, {50, 150, 38, true, false}};
    CHECK(utc_to_tai({75, 0}, {overlap, 2}, result) == Error::ambiguous);
    OffsetInterval positive[]{{0, 100, 36, true, false}, {100, 200, 37, true, false}};
    CHECK(tai_to_utc({136, 0}, {positive, 2}, result) == Error::unavailable);
    CHECK(tai_to_utc({137, 11}, {positive, 2}, result) == Error::ok && result == Timestamp{100, 11});
    OffsetInterval negative[]{{0, 100, 37, true, false}, {100, 200, 36, true, false}};
    CHECK(tai_to_utc({136, 0}, {negative, 2}, result) == Error::ambiguous);
}

void conversion_tests(const std::string& zone) {
    CivilTime civil;
    Timestamp back;
    CHECK(utc_to_local({1705320000, 123456789}, zone, civil) == Error::unavailable);
    CHECK(configure_timezone("") == Error::invalid);
    CHECK(configure_timezone(std::string("UTC0\0bad", 8)) == Error::invalid);
#if defined(_WIN32) && !defined(__MINGW32__)
    CHECK(configure_timezone("EST5EDT,M3.2.0/2,M11.1.0/2") == Error::unsupported);
#endif
    CHECK(configure_timezone(zone) == Error::ok);
    CHECK(configure_timezone(zone) == Error::ok);
    CHECK(configure_timezone("OTHER0") == Error::mismatch);
    const std::string saved_zone = std::getenv("TZ");
    OffsetInterval interval{1700000000, 1800000000, 37, true, false};
    const OffsetTable table{&interval, 1};
    for (Timestamp utc : {Timestamp{1705320000, 123456789}, Timestamp{1721044800, 999999999}}) {
        Timestamp tai;
        CHECK(utc_to_tai(utc, table, tai) == Error::ok && tai.seconds == utc.seconds + 37);
        CHECK(tai_to_utc(tai, table, back) == Error::ok && back == utc);
        CHECK(utc_to_local(utc, zone, civil) == Error::ok);
        CHECK(civil.nanoseconds == utc.nanoseconds && civil.zone == zone);
        CHECK(civil.hour == (zone == "UTC0" ? 12 : utc.seconds == 1705320000 ? 7 : 8));
        CHECK(civil.utc_offset_seconds == (zone == "UTC0" ? 0 : utc.seconds == 1705320000 ? -18000 : -14400));
        CHECK(civil.daylight == (zone != "UTC0" && utc.seconds == 1721044800));
        CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::ok && back == utc);
        CivilTime via_tai;
        CHECK(tai_to_local(tai, table, zone, via_tai) == Error::ok);
        CHECK(via_tai.hour == civil.hour && via_tai.nanoseconds == civil.nanoseconds);
        CHECK(local_to_tai(civil, DstPolicy::reject, table, back) == Error::ok && back == tai);
        std::string example;
        CHECK(conversion_example(utc, table, zone, example) == Error::ok);
        for (const char* label : {"UTC to TAI:", "TAI to UTC:", "UTC to local:", "Local to UTC:", "TAI to local:", "Local to TAI:", "Round trips preserve instant and nanoseconds: yes"})
            CHECK(example.find(label) != std::string::npos);
        CHECK(example.find("zone=" + zone) != std::string::npos);
        CHECK(example.find("UTC offset seconds=") != std::string::npos);
        std::string report;
        CHECK(format_time_report(tai, utc, zone, report) == Error::ok);
        CHECK(report.find("TAI=") == 0);
        CHECK(report.find(utc.seconds == 1705320000 ? "TAI=2024-01-15T12:00:37.123456789" :
                         "TAI=2024-07-15T12:00:37.999999999") == 0);
        CHECK(report.find(utc.seconds == 1705320000 ? "UTC=2024-01-15T12:00:00.123456789Z" :
                         "UTC=2024-07-15T12:00:00.999999999Z") != std::string::npos);
        CHECK(report.find(utc.seconds == 1705320000 ? "37.123456789Z" : "37.999999999Z") == std::string::npos);
        CHECK(report.find(zone == "UTC0" ? "12:00:00 PM" :
                         utc.seconds == 1705320000 ? "7:00:00 AM" : "8:00:00 AM") != std::string::npos);
        CHECK(report.find(zone == "UTC0" ? "(UTC+00:00)" :
                         utc.seconds == 1705320000 ? "(UTC-05:00)" : "(UTC-04:00)") != std::string::npos);
        CHECK(report.find(zone == "UTC0" ? "Local=2024-" :
                         utc.seconds == 1705320000 ? "Local=2024-01-15 7" : "Local=2024-07-15 8") != std::string::npos);
        const auto saved_report = report;
        CHECK(format_time_report({0, 1000000000}, utc, zone, report) == Error::invalid);
        CHECK(report == saved_report);
        CHECK(format_time_report(tai, {0, 1000000000}, zone, report) == Error::invalid);
        CHECK(report == saved_report);
        CHECK(format_time_report(tai, utc, "OTHER0", report) == Error::mismatch);
        CHECK(report == saved_report);
        CHECK(saved_zone == std::getenv("TZ"));
    }
    // Reporting derives UTC from TAI using a whole-second master offset, not
    // the microsecond-resolution system clock. Preserve the final 683 ns even
    // when the conversion crosses a minute, day, or year boundary.
    for (Timestamp tai : {Timestamp{1705320000, 807683}, Timestamp{1704067220, 807683}}) {
        const Timestamp utc{tai.seconds - interval.tai_minus_utc, tai.nanoseconds};
        CHECK(tai_to_utc(tai, table, back) == Error::ok && back == utc);
        std::string report;
        CHECK(format_time_report(tai, utc, zone, report) == Error::ok);
        CHECK(report.find(tai.seconds == 1705320000 ? "TAI=2024-01-15T12:00:00.000807683" :
                         "TAI=2024-01-01T00:00:20.000807683") == 0);
        CHECK(report.find(tai.seconds == 1705320000 ? "UTC=2024-01-15T11:59:23.000807683Z" :
                         "UTC=2023-12-31T23:59:43.000807683Z") != std::string::npos);
        CHECK(report.find(zone == "UTC0" ?
                         (tai.seconds == 1705320000 ? "Local=2024-01-15 11:59:23 AM" : "Local=2023-12-31 11:59:43 PM") :
                         (tai.seconds == 1705320000 ? "Local=2024-01-15 6:59:23 AM" : "Local=2023-12-31 6:59:43 PM")) != std::string::npos);
    }
    std::string midnight;
    const Timestamp midnight_utc{zone == "UTC0" ? 1704067200 : 1704085200, 0};
    CHECK(format_time_report({midnight_utc.seconds + 37, 0}, midnight_utc, zone, midnight) == Error::ok);
    CHECK(midnight.find("Local=2024-01-01 12:00:00 AM") != std::string::npos);
    CHECK(midnight.find(zone == "UTC0" ? "TAI=2024-01-01T00:00:37.000000000" :
                        "TAI=2024-01-01T05:00:37.000000000") == 0);
    if (zone != "UTC0") {
        CHECK(format_time_report({1704067237, 0}, {1704067200, 0}, zone, midnight) == Error::ok);
        CHECK(midnight.find("Local=2023-12-31 7:00:00 PM") != std::string::npos);
    }
    civil = {}; civil.zone = zone; civil.year = 2024; civil.month = 2; civil.day = 30;
    CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::invalid);
    civil.day = 1; civil.second = 60;
    CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::invalid);
    civil.second = 0; civil.nanoseconds = 1000000000;
    CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::invalid);
    civil.nanoseconds = 0; civil.zone = "OTHER0";
    CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::mismatch);
    if (zone != "UTC0") {
        civil = {}; civil.zone = zone; civil.year = 2024; civil.month = 11; civil.day = 3;
        civil.hour = 1; civil.minute = 30; civil.nanoseconds = 123;
        CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::ambiguous);
        Timestamp earlier, later;
        CHECK(local_to_utc(civil, DstPolicy::earlier, earlier) == Error::ok);
        CHECK(local_to_utc(civil, DstPolicy::later, later) == Error::ok);
        CHECK(earlier == Timestamp{1730611800, 123});
        CHECK(later == Timestamp{1730615400, 123});
        civil.month = 3; civil.day = 10; civil.hour = 2;
        CHECK(local_to_utc(civil, DstPolicy::reject, back) == Error::invalid);
        CHECK(local_to_utc(civil, DstPolicy::earlier, back) == Error::invalid);
        CHECK(local_to_utc(civil, DstPolicy::later, back) == Error::invalid);
    }
}
}

int main(int argc, char** argv) {
    const std::string zone = argc == 2 ? argv[1] : "UTC0";
    if (zone == "UTC0") {
        timestamp_tests();
            selection_tests();
        matcher_tests();
        rate_tests();
        security_association_regression();
        e2e_tests();
        servo_tests();
        mac_tests();
        offset_table_tests();
    }
    conversion_tests(zone);
    std::cout << "core_tests " << zone << ": " << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
