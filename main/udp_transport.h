// Copyright Rob Latour, 2026. MIT license.
#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include "internet_checksum.h"

// Small, host-testable codec shared by RX validation and hardware-timestamped
// Delay_Req TX. lwIP still owns addressing, DHCP, ARP, ND and multicast membership.
// VLANs, IP fragments and IPv6 extension headers are outside this demo profile.
namespace follower::udp {
inline constexpr uint8_t ipv4_group[4] = {224, 0, 1, 129};
inline constexpr uint8_t ipv6_group[16] = {0xff, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x81};
inline constexpr uint8_t ipv4_mac[6] = {1, 0, 0x5e, 0, 1, 0x81};
inline constexpr uint8_t ipv6_mac[6] = {0x33, 0x33, 0, 0, 1, 0x81};
inline constexpr size_t max_frame = 14 + 40 + 8 + 64;

inline uint16_t read16(const uint8_t* p) { return uint16_t((uint16_t(p[0]) << 8) | p[1]); }
inline void write16(uint8_t* p, uint16_t value) {
    p[0] = uint8_t(value >> 8); p[1] = uint8_t(value);
}
struct Datagram {
    const uint8_t* message = nullptr;
    size_t length = 0;
    std::array<uint8_t, 16> source{};
};
enum class Result { ignored, invalid, valid };

// Only consume this configured multicast group. Unrelated traffic, including
// DHCP/ND, must reach esp_netif unchanged. Ethernet padding is outside IP length.
inline Result decode(const uint8_t* frame, size_t length, bool ipv6, Datagram& out) {
    if (!frame || length < 14 || read16(frame + 12) != (ipv6 ? 0x86dd : 0x0800))
        return Result::ignored;
    const size_t minimum = ipv6 ? 40 : 20;
    if (length < 14 + minimum) return Result::ignored;
    const uint8_t* ip = frame + 14;
    const size_t destination = ipv6 ? 24 : 16;
    if (std::memcmp(ip + destination, ipv6 ? ipv6_group : ipv4_group, ipv6 ? 16 : 4) != 0)
        return Result::ignored;
    const size_t header = ipv6 ? 40 : (ip[0] & 15) * 4U;
    if ((ip[0] >> 4) != (ipv6 ? 6 : 4) || header < minimum ||
        length < 14 + header + 8 || ip[ipv6 ? 6 : 9] != 17)
        return Result::invalid;
    const uint8_t* payload = ip + header;
    const uint16_t port = read16(payload + 2);
    if (port != 319 && port != 320) return Result::ignored;
    const size_t ip_length = ipv6 ? 40 + read16(ip + 4) : read16(ip + 2);
    const size_t udp_length = read16(payload + 4);
    if (udp_length < 8 + 34 || ip_length > length - 14 || ip_length != header + udp_length ||
        read16(payload) != port || ip[ipv6 ? 7 : 8] == 0 ||
        std::memcmp(frame, ipv6 ? ipv6_mac : ipv4_mac, 6) != 0 ||
        (frame[6] & 1) || std::all_of(frame + 6, frame + 12, [](uint8_t b) { return b == 0; }))
        return Result::invalid;
    const uint8_t* source = ip + (ipv6 ? 8 : 12);
    if (ipv6 ? (source[0] == 0xff || std::all_of(source, source + 16, [](uint8_t b) { return b == 0; }))
             : ((read16(ip + 6) & 0xbfff) != 0 || net_packet::checksum(ip, header) != 0 ||
                source[0] == 0 || source[0] >= 224))
        return Result::invalid;
    const bool checksum_present = read16(payload + 6) != 0;
    if ((ipv6 && !checksum_present) ||
        (checksum_present && net_packet::fold(net_packet::udp_sum(source, ip + destination,
            ipv6 ? 16 : 4, payload, udp_length)) != 0xffff))
        return Result::invalid;
    const uint8_t* message = payload + 8;
    const uint8_t type = message[0] & 15;
    if (read16(message + 2) != udp_length - 8 ||
        ((type == 0 || type == 1) ? port != 319 : port != 320))
        return Result::invalid;
    Datagram decoded{message, udp_length - 8, {}};
    std::copy_n(source, ipv6 ? 16 : 4, decoded.source.begin());
    out = decoded;
    return Result::valid;
}

inline size_t encapsulate(uint8_t* frame, size_t capacity, const uint8_t* source_mac,
                          const uint8_t* source_ip, bool ipv6, const uint8_t* message, size_t length) {
    const size_t header = ipv6 ? 40 : 20, udp_length = 8 + length;
    const size_t total = 14 + header + udp_length;
    if (!frame || !source_mac || !source_ip || !message || length < 34 || length > 64 || capacity < total)
        return 0;
    std::memset(frame, 0, total);
    std::memcpy(frame, ipv6 ? ipv6_mac : ipv4_mac, 6);
    std::memcpy(frame + 6, source_mac, 6);
    write16(frame + 12, ipv6 ? 0x86dd : 0x0800);
    uint8_t* ip = frame + 14;
    ip[0] = ipv6 ? 0x60 : 0x45;
    write16(ip + (ipv6 ? 4 : 2), uint16_t(ipv6 ? udp_length : header + udp_length));
    ip[ipv6 ? 6 : 9] = 17;
    ip[ipv6 ? 7 : 8] = 1;
    std::memcpy(ip + (ipv6 ? 8 : 12), source_ip, ipv6 ? 16 : 4);
    std::memcpy(ip + (ipv6 ? 24 : 16), ipv6 ? ipv6_group : ipv4_group, ipv6 ? 16 : 4);
    if (!ipv6) write16(ip + 10, net_packet::checksum(ip, header));
    uint8_t* payload = ip + header;
    const uint16_t port = (message[0] & 15) <= 1 ? 319 : 320;
    write16(payload, port); write16(payload + 2, port); write16(payload + 4, uint16_t(udp_length));
    std::memcpy(payload + 8, message, length);
    const uint16_t checksum = uint16_t(~net_packet::fold(net_packet::udp_sum(
        ip + (ipv6 ? 8 : 12), ip + (ipv6 ? 24 : 16), ipv6 ? 16 : 4, payload, udp_length)));
    write16(payload + 6, checksum ? checksum : 0xffff);
    return total;
}
}
