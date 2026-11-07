// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#pragma once

#include "sdkconfig.h"
#include "setting_types.h"

// The chip is selected by the ESP-IDF build target (idf.py set-target esp32p4 /
// idf.py --preview set-target esp32s31), never by an editable setting here.
// ESP32_CHIP_TYPE is a logging label only; branch on CONFIG_IDF_TARGET_* instead.
#if CONFIG_IDF_TARGET_ESP32P4
#define ESP32_CHIP_TYPE "P4"
#elif CONFIG_IDF_TARGET_ESP32S31
#define ESP32_CHIP_TYPE "S31"
#else
#error "ESP32PTPL3Follower supports only ESP32-P4 and ESP32-S31"
#endif

namespace follower::settings
{

    inline constexpr char mac_override[] = ""; // leave blank ( "" ) to use the native mac of this device, or override it by specifying a mac to be used, such as: "1e:29:04:d0:2e:3b"

    // Set on esp_netif before Ethernet starts; DHCP advertises this hostname.
    // This does not start an mDNS responder or guarantee a DNS record.
    inline constexpr char hostname[] = "ESP32PTPL3Follower";

    // One timing transport at a time avoids mixing IPv4 and IPv6 exchanges from
    // the same master's port identity. Both IP stacks remain available to lwIP.
    // 0: 224.0.1.129 (IPv4, DHCP); 1: ff02::181 (IPv6, link-local).
#ifndef FOLLOWER_PTP_IPV6
#define FOLLOWER_PTP_IPV6 0
#endif
    inline constexpr bool ptp_ipv6 = FOLLOWER_PTP_IPV6;

    // used for console log display purposes
    inline constexpr char timeZoneSpec[] = "EST5EDT,M3.2.0/2,M11.1.0/2";

    // UDP PTP is unsigned only: no key ring, secrets header or authentication.
    // unless you know what your are doing, do not change below this line

    // Hardware timestamps are mandatory; there is no software-timestamp fallback.
    // All-multicast reception also permits IGMP/MLD and IPv6 neighbor discovery.
    inline constexpr std::size_t ethernet_rx_queue_capacity = 16;
    inline constexpr std::size_t ethernet_max_ptp_frame_bytes = 192;

    inline constexpr DstResolution local_dst_resolution = DstResolution::reject;

    // Restricted, single-segment BMCA-style follower selection, not full BMCA.
    inline constexpr uint8_t ptp_domain = 0;
    inline constexpr MasterSelection master_selection = MasterSelection::ranked;
    inline constexpr PortIdentity pinned_source_identity{};
    inline constexpr bool pinned_mac_enabled = false;
    inline constexpr MacAddress pinned_source_mac{};
    inline constexpr std::size_t candidate_capacity = 8;
    inline constexpr unsigned announce_qualification_count = 2;
    inline constexpr int announce_log_interval_min = -7;
    inline constexpr int announce_log_interval_max = 4;
    inline constexpr int sync_log_interval_min = -7;
    inline constexpr int sync_log_interval_max = 4;

    // All *_us scheduling limits use monotonic microseconds; *_ns are durations or
    // phase errors in nanoseconds. Intervals are bounded before exponentiation.
    inline constexpr int64_t announce_expiry_min_us = 10'000'000;
    inline constexpr unsigned announce_expiry_intervals = 3;
    inline constexpr int64_t delay_request_interval_us = 1'000'000;
    inline constexpr int64_t delay_response_timeout_us = 2'000'000;
    inline constexpr int64_t max_exchange_age_us = 5'000'000;
    inline constexpr int64_t max_pair_age_us = 5'000'000;
    inline constexpr int64_t sync_freshness_min_us = 3'000'000;
    inline constexpr unsigned sync_freshness_intervals = 3;
    // Longest gap between accepted E2E (Sync + Delay_Req/Delay_Resp) measurements before
    // LOCKED is revoked. At a 1 Hz Sync rate a measurement completes about every 2 s, so
    // this tolerates a few lost exchanges; loss of Sync/Announce is detected separately.
    inline constexpr int64_t measurement_timeout_us = 10'000'000;
    inline constexpr std::size_t pending_pair_capacity = 8;
    inline constexpr int64_t max_mean_path_delay_ns = 1'000'000'000;
    inline constexpr int64_t negative_delay_tolerance_ns = 1;

    // Bounded phase/frequency controller. Clamp again to actual EMAC limits. Startup
    // steps are explicit; while locked, large errors declare reacquisition first.
    inline constexpr bool allow_initial_step = true;
    inline constexpr bool allow_recovery_step = true;
    inline constexpr int64_t reacquire_step_threshold_ns = 100'000'000;
    inline constexpr double max_frequency_ppm = 100.0;
    inline constexpr double max_slew_ppm = 100.0;
    inline constexpr double servo_phase_gain = 0.5;
    inline constexpr double servo_integral_gain = 0.02;
    inline constexpr double servo_integral_limit_ppm = 100.0;
    inline constexpr int64_t servo_outlier_limit_ns = 1'000'000'000;
    inline constexpr int64_t lock_offset_threshold_ns = 100'000;
    inline constexpr int64_t unlock_offset_threshold_ns = 500'000;
    inline constexpr unsigned lock_sample_count = 8;
    inline constexpr unsigned unlock_sample_count = 3;
    // The last UTC clock keeps advancing, but zero qualified holdover means it must
    // immediately be labeled UNSYNCHRONIZED after input failure, not LOCKED.
    inline constexpr int64_t holdover_duration_us = 0;
    inline constexpr double holdover_uncertainty_ppm = 200.0;
    inline constexpr int64_t clock_bridge_interval_us = 100'000;
    inline constexpr int64_t max_clock_bridge_error_ns = 100'000;
    // A PHC-vs-CLOCK_REALTIME comparison brackets the CLOCK_REALTIME read between two PHC
    // reads. A bracket wider than this means the reading task was preempted, so the
    // comparison is retried (up to the attempt count) and is never used to steer UTC or
    // to revoke LOCKED. Half the window bounds the uncertainty of an accepted comparison.
    inline constexpr int64_t max_clock_bridge_read_window_ns = 40'000;
    inline constexpr unsigned clock_bridge_read_attempts = 4;
    // LOCKED still requires a precise, in-tolerance comparison at least this recently.
    inline constexpr int64_t max_clock_bridge_unverified_us = 1'000'000;
    inline constexpr int64_t utc_metadata_validity_us = 5'000'000;
    // PTP wire timestamps are TAI; application CLOCK_REALTIME is UTC. Conversion requires
    // selected-master UTC-offset metadata, never a hardcoded TAI-minus-UTC value.
    inline constexpr int log_verbosity = 3; // ESP_LOG_INFO; diagnostics are rate limited.
    inline constexpr bool second_reporting_enabled = true;
    inline constexpr int console_baud_rate = 115200;

    static_assert(candidate_capacity >= 1 && candidate_capacity <= 32);
    static_assert(pending_pair_capacity >= 1 && pending_pair_capacity <= 32);
    static_assert(announce_expiry_intervals > 0);
    static_assert(measurement_timeout_us >= sync_freshness_min_us);
    static_assert(announce_qualification_count >= 2);
    static_assert(announce_log_interval_min >= -7 && announce_log_interval_max <= 4 &&
                  announce_log_interval_min <= announce_log_interval_max);
    static_assert(lock_sample_count > 0 && unlock_sample_count > 0);
    static_assert(lock_offset_threshold_ns < unlock_offset_threshold_ns);
    static_assert(max_frequency_ppm > 0 && max_slew_ppm > 0);
    static_assert(FOLLOWER_PTP_IPV6 == 0 || FOLLOWER_PTP_IPV6 == 1);
    static_assert(sizeof(hostname) > 1 && sizeof(hostname) <= 64);

    // Board Ethernet wiring is selected by the build target. Network identities and
    // live deployment values are deliberately absent.
#if CONFIG_IDF_TARGET_ESP32P4
    // Waveshare ESP32-P4-ETH: IP101 PHY over RMII with an external 50 MHz reference
    // clock, verified against the public reference settings.
    // https://docs.waveshare.com/ESP32-P4-ETH
    inline constexpr EthernetPhy ethernet_phy = EthernetPhy::ip101;
    inline constexpr RmiiClock rmii_clock = RmiiClock::external_input;
    inline constexpr int ethernet_mdc_gpio = 31;
    inline constexpr int ethernet_mdio_gpio = 52;
    inline constexpr int ethernet_phy_reset_gpio = 51;
    inline constexpr int ethernet_phy_address = 1;
    inline constexpr int ethernet_rmii_clock_gpio = 50;
    inline constexpr int ethernet_rmii_tx_enable_gpio = 49;
    inline constexpr int ethernet_rmii_tx0_gpio = 34;
    inline constexpr int ethernet_rmii_tx1_gpio = 35;
    inline constexpr int ethernet_rmii_crs_dv_gpio = 28;
    inline constexpr int ethernet_rmii_rx0_gpio = 29;
    inline constexpr int ethernet_rmii_rx1_gpio = 30;
#elif CONFIG_IDF_TARGET_ESP32S31
    // Espressif ESP32-S31-Function-CoreBoard-1: onboard YT8531 PHY over RGMII
    // (10/100/1000 Mbps). Values match the board schematic and the ESP-IDF v6.1
    // EMAC defaults / Ethernet basic example. These pins are fixed board wiring.
    // https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-function-coreboard-1/user_guide.html
    inline constexpr EthernetPhy ethernet_phy = EthernetPhy::yt8531;
    inline constexpr int ethernet_mdc_gpio = 5;
    inline constexpr int ethernet_mdio_gpio = 6;
    inline constexpr int ethernet_phy_reset_gpio = 7;
    // -1 lets the driver discover the PHY address (board straps default it low).
    inline constexpr int ethernet_phy_address = -1;
    inline constexpr int ethernet_rgmii_rx_clock_gpio = 14;
    inline constexpr int ethernet_rgmii_tx_clock_gpio = 13;
    inline constexpr int ethernet_rgmii_phy_ref_clock_gpio = -1; // not used on this board
    inline constexpr int ethernet_rgmii_tx_control_gpio = 12;
    inline constexpr int ethernet_rgmii_tx0_gpio = 8;
    inline constexpr int ethernet_rgmii_tx1_gpio = 9;
    inline constexpr int ethernet_rgmii_tx2_gpio = 10;
    inline constexpr int ethernet_rgmii_tx3_gpio = 11;
    inline constexpr int ethernet_rgmii_rx_control_gpio = 15;
    inline constexpr int ethernet_rgmii_rx0_gpio = 19;
    inline constexpr int ethernet_rgmii_rx1_gpio = 18;
    inline constexpr int ethernet_rgmii_rx2_gpio = 17;
    inline constexpr int ethernet_rgmii_rx3_gpio = 16;
#endif
}
