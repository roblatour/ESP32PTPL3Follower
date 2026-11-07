#include "../main/follower_core.h"
#include "../main/udp_transport.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {
using namespace follower;
int checks = 0, failures = 0;
#define CHECK(...) do { ++checks; if (!(__VA_ARGS__)) { ++failures; std::cerr << __LINE__ << ": " << #__VA_ARGS__ << '\n'; } } while (false)

std::vector<uint8_t> message(MessageType type) {
    std::vector<uint8_t> p(type == MessageType::announce ? 64 : type == MessageType::delay_resp ? 54 : 44);
    p[0] = uint8_t(type); p[1] = 2;
    udp::write16(p.data() + 2, uint16_t(p.size()));
    udp::write16(p.data() + 6, type == MessageType::announce ? 0x3c : type == MessageType::sync ? 0x200 : 0);
    p[27] = 1; p[29] = 1;
    udp::write16(p.data() + 30, 0xabcd);
    p[32] = type == MessageType::announce ? 5 : type == MessageType::sync ? 0 :
            type == MessageType::delay_req ? 1 : type == MessageType::follow_up ? 2 : 3;
    p[33] = 0xff; // log interval -1
    p[39] = 100; // origin seconds
    p[43] = 10;
    if (type == MessageType::announce) {
        udp::write16(p.data() + 44, 37);
        p[47] = 128; p[48] = 6; p[49] = 0x22;
        udp::write16(p.data() + 50, 0x1234);
        p[52] = 128; p[60] = 1;
    }
    if (type == MessageType::delay_resp) { p[51] = 9; p[53] = 1; }
    return p;
}

std::vector<uint8_t> encode(const std::vector<uint8_t>& p, bool ipv6) {
    const uint8_t mac[6] = {2, 3, 4, 5, 6, 7};
    const uint8_t v4[4] = {192, 0, 2, 1};
    const uint8_t v6[16] = {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
    std::vector<uint8_t> bytes(udp::max_frame);
    const size_t size = udp::encapsulate(bytes.data(), bytes.size(), mac, ipv6 ? v6 : v4,
                                         ipv6, p.data(), p.size());
    CHECK(size != 0);
    bytes.resize(size);
    return bytes;
}

void repair(std::vector<uint8_t>& bytes, bool ipv6) {
    uint8_t* ip = bytes.data() + 14;
    const size_t header = ipv6 ? 40 : 20;
    uint8_t* u = ip + header;
    udp::write16(u + 6, 0);
    const auto checksum = uint16_t(~net_packet::fold(net_packet::udp_sum(
        ip + (ipv6 ? 8 : 12), ip + (ipv6 ? 24 : 16), ipv6 ? 16 : 4, u, udp::read16(u + 4))));
    udp::write16(u + 6, checksum ? checksum : 0xffff);
    if (!ipv6) {
        udp::write16(ip + 10, 0);
        udp::write16(ip + 10, net_packet::checksum(ip, header));
    }
}

void exchange_test(bool ipv6) {
    const ParseConfig config{0, {}, ipv6};
    const Security accepted{true, false, 0, 0};
    auto decode = [&](std::vector<uint8_t> p) {
        const auto bytes = encode(p, ipv6);
        Packet parsed;
        CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
        return parsed;
    };
    auto sync_message = message(MessageType::sync);
    auto follow_message = message(MessageType::follow_up);
    follow_message[43] = 0;
    const auto sync = decode(sync_message);
    const auto follow = decode(follow_message);
    Identity local{}; local[7] = 9; local[9] = 1;
    ExchangeMatcher matcher;
    matcher.select(sync.source, local, 0, 1);
    CHECK(matcher.sync(sync, accepted, {100, 250000}, true, 1, 0) == Error::ok);
    CHECK(matcher.follow_up(follow, accepted, 1, 1'000'000) == Error::ok);
    Pair pair;
    CHECK(matcher.take_pair(2'000'000, pair) == Error::ok);
    CHECK(matcher.begin_delay(0xabcd, {101, 50000}, true, pair, 1, billion) == Error::ok);
    auto response_message = message(MessageType::delay_resp);
    response_message[39] = 101;
    response_message[40] = 0; response_message[41] = 3;
    response_message[42] = 0x0d; response_message[43] = 0x40; // 200000 ns
    const auto response = decode(response_message);
    Exchange exchange;
    CHECK(matcher.delay_response(response, accepted, 1, billion + 1'000'000, exchange) == Error::ok);
    Measurement measurement;
    CHECK(calculate_e2e(exchange, billion + 2'000'000, measurement) == Error::ok);
    CHECK(measurement.offset_ns() == 50000 && measurement.delay_ns() == 200000);
}

void run(bool ipv6) {
    const ParseConfig config{0, {}, ipv6};
    const size_t header = ipv6 ? 40 : 20, payload = 14 + header + 8;
    Packet parsed;
    for (MessageType type : {MessageType::announce, MessageType::sync, MessageType::follow_up,
                             MessageType::delay_req, MessageType::delay_resp}) {
        auto bytes = encode(message(type), ipv6);
        CHECK(bytes.size() == payload + message(type).size());
        CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
        CHECK(parsed.type == type && parsed.sequence == 0xabcd && parsed.log_interval == -1);
        CHECK(parsed.timestamp == Timestamp{100, 10});
        CHECK(parsed.source_ip[0] == (ipv6 ? 0xfe : 192));
        if (type == MessageType::announce) {
            CHECK(parsed.properties.usable() && parsed.properties.utc_offset == 37);
            CHECK(parsed.dataset.clock_class == 6 && parsed.dataset.variance == 0x1234);
        }
        if (type == MessageType::delay_resp) CHECK(parsed.requester[7] == 9);
        for (size_t n = 0; n < bytes.size(); ++n)
            CHECK(parse_ptp(bytes.data(), n, config, parsed) != Error::ok);
        bytes.resize(bytes.size() + 16, 0xff);
        CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
    }
    CHECK(parse_ptp(nullptr, 0, config, parsed) == Error::invalid);
    const auto reject = [&](size_t offset, uint8_t value, Error expected) {
        auto bytes = encode(message(MessageType::announce), ipv6);
        bytes[offset] = value;
        repair(bytes, ipv6);
        Packet sentinel; sentinel.sequence = 123;
        CHECK(parse_ptp(bytes.data(), bytes.size(), config, sentinel) == expected);
        CHECK(sentinel.sequence == 123);
    };
    reject(12, 0x81, Error::wrong_destination);
    reject(0, 2, Error::invalid);
    reject(6, 3, Error::invalid);
    reject(payload, 0x1b, Error::unsupported);
    reject(payload + 1, 3, Error::unsupported);
    reject(payload + 1, 0x22, Error::unsupported);
    reject(payload + 4, 1, Error::wrong_domain);
    reject(payload + 5, 1, Error::unsupported);
    reject(payload + 16, 1, Error::unsupported);
    reject(payload + 32, 0, Error::invalid);
    reject(payload + 6, 0x80, Error::unsupported);
    reject(payload + 46, 1, Error::invalid);
    reject(payload + 62, 1, Error::unsupported);
    reject(payload + 40, 0xff, Error::invalid);
    reject(payload + 7, 0x3f, Error::invalid);
    reject(14 + header + 3, 0x3f, Error::invalid);
    reject(14 + 6, ipv6 ? 44 : 0x20, Error::invalid); // extension/fragment
    reject(14 + (ipv6 ? 7 : 8), 0, Error::invalid); // zero hop limit
    auto bytes = encode(message(MessageType::announce), ipv6);
    bytes.back() ^= 1;
    CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::invalid);
    bytes = encode(message(MessageType::announce), ipv6);
    udp::write16(bytes.data() + 14 + header + 6, 0);
    CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == (ipv6 ? Error::invalid : Error::ok));
    // Unknown/authentication TLVs are rejected even with a valid UDP checksum.
    bytes = encode(message(MessageType::sync), ipv6);
    bytes[payload + 1] = 0x12; repair(bytes, ipv6);
    CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
    bytes.resize(bytes.size() + 26, 0);
    udp::write16(bytes.data() + payload + 2, 70);
    udp::write16(bytes.data() + payload + 44, 0x8009);
    udp::write16(bytes.data() + payload + 46, 22);
    udp::write16(bytes.data() + 14 + header + 4, 78);
    udp::write16(bytes.data() + 14 + (ipv6 ? 4 : 2), uint16_t(ipv6 ? 78 : header + 78));
    repair(bytes, ipv6);
    CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::unsupported);
    // Foreign group/family traffic remains lwIP's responsibility.
    udp::Datagram datagram;
    bytes = encode(message(MessageType::announce), ipv6);
    CHECK(udp::decode(bytes.data(), bytes.size(), !ipv6, datagram) == udp::Result::ignored);
    bytes[14 + (ipv6 ? 24 : 16)] ^= 1;
    CHECK(udp::decode(bytes.data(), bytes.size(), ipv6, datagram) == udp::Result::ignored);
    CHECK(udp::encapsulate(nullptr, 0, nullptr, nullptr, ipv6, nullptr, 44) == 0);
    if (!ipv6) {
        bytes = encode(message(MessageType::announce), false);
        bytes.insert(bytes.begin() + 34, 4, 0); // IPv4 options, before UDP
        bytes[14] = 0x46;
        udp::write16(bytes.data() + 16, uint16_t(bytes.size() - 14));
        udp::write16(bytes.data() + 24, 0);
        udp::write16(bytes.data() + 24, net_packet::checksum(bytes.data() + 14, 24));
        CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
    }

    // Changing only the master's IP must restart qualification, not reuse lock.
    RestrictedFollowerSelection selection;
    const Security accepted{true, false, 0, 0};
    bytes = encode(message(MessageType::announce), ipv6);
    CHECK(parse_ptp(bytes.data(), bytes.size(), config, parsed) == Error::ok);
    CHECK(selection.announce(parsed, accepted, 0) == Error::ok);
    ++parsed.sequence;
    CHECK(selection.announce(parsed, accepted, billion) == Error::ok);
    CHECK(selection.selected() != nullptr);
    const auto generation = selection.generation();
    ++parsed.sequence; parsed.source_ip[ipv6 ? 15 : 3] ^= 1;
    CHECK(selection.announce(parsed, accepted, 2 * billion) == Error::ok);
    CHECK(selection.generation() != generation && selection.selected() == nullptr);
}
}

int main() {
    const uint8_t rfc1071[] = {0, 1, 0xf2, 3, 0xf4, 0xf5, 0xf6, 0xf7};
    CHECK(net_packet::checksum(rfc1071, sizeof(rfc1071)) == 0x220d);
    run(false); run(true);
    exchange_test(false); exchange_test(true);
    std::cout << checks << " UDP checks; " << failures << " failures\n";
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
