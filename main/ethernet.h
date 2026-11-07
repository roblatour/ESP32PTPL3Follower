// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include "esp_err.h"
#include "follower_core.h"

namespace follower
{
    // Call once from application initialization after public settings are fixed.
    // Installs native Ethernet (ESP32-P4: RMII/IP101; ESP32-S31: RGMII/YT8531) and one
    // bounded UDP PTP worker, plus DHCP/IPv6 addressing and IGMP/MLD membership. The
    // worker borrows the clock-owner API; it never sets UTC/TAI clocks directly.
    // Returns an IDF error on configuration, timestamp qualification, or setup failure.
    // Not safe for concurrent calls; a successful installation lives until reboot.
    esp_err_t ethernet_start();
    // Call only after ethernet_start() succeeds; returns the driver's effective MAC.
    Mac ethernet_mac();
    // Returns the selected PTP master's Ethernet MAC, if a master is currently selected.
    bool ethernet_master_mac(Mac &master_mac);
    // Call after the Ethernet link is up; returns the negotiated speed in Mbps.
    esp_err_t ethernet_link_speed(uint32_t *speed_mbps);
}
