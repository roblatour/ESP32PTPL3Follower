// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace follower
{
    enum class MasterSelection : uint8_t
    {
        ranked,
        pinned
    };
    enum class EthernetPhy : uint8_t
    {
        ip101,
        yt8531
    };
    enum class RmiiClock : uint8_t
    {
        external_input,
        internal_output
    };
    enum class SiliconProfile : uint8_t
    {
        pre_v3,
        v3_0,
        v3_1_or_later
    };
    enum class DstResolution : uint8_t
    {
        reject,
        earlier,
        later
    };

    using PortIdentity = std::array<uint8_t, 10>;
    using MacAddress = std::array<uint8_t, 6>;
}
