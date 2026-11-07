// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#include "ethernet.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cinttypes>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include "ESP32PTPL3FollowerSettings.h"
#include "udp_transport.h"
#include "clock_owner.h"
#include "follower_core.h"
#include "sync_rate.h"
#include "esp_chip_info.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_eth_netif_glue.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <cerrno>
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

namespace follower
{
    namespace
    {
        constexpr const char *Tag = "ethernet";
        static_assert(settings::ethernet_rx_queue_capacity > 0 && settings::ethernet_rx_queue_capacity <= 128);
        static_assert(settings::ethernet_max_ptp_frame_bytes >= 104 && settings::ethernet_max_ptp_frame_bytes <= 1536);
        static_assert(std::atomic<uint32_t>::is_always_lock_free);

        struct Reception
        {
            std::array<uint8_t, settings::ethernet_max_ptp_frame_bytes> frame{};
            Timestamp timestamp{};
            int64_t received_us = 0;
            uint32_t generation = 0, link_epoch = 0;
            uint16_t length = 0;
            bool hardware = false;
        };
        esp_eth_handle_t ethernet;
        esp_eth_mac_t *mac;
        esp_eth_phy_t *phy;
        esp_event_handler_instance_t event_handler;
        QueueHandle_t queue;
        StaticQueue_t queue_control;
        alignas(4) uint8_t queue_storage[settings::ethernet_rx_queue_capacity * sizeof(Reception)];
        std::atomic<uint32_t> receive_generation{0}, link_epoch{0}, link_state{0}, overflow{0};
        std::atomic<uint32_t> ptp_received{0}, ptp_queued{0}, ptp_dropped{0};
        bool attempted = false;
        esp_netif_t *netif = nullptr;
        // Only the protocol worker mutates socket membership and source addressing.
        // Socket membership generates IGMP/MLD reports for snooping switches;
        // event packets themselves bypass sockets to retain EMAC RX timestamps.
        int membership_sockets[2] = {-1, -1};
        std::array<uint8_t, 16> source_ip{};
        bool network_ready = false;
        uint32_t network_epoch = 0;
        int64_t network_retry_us = 0;
        Mac local_mac{};
        Identity local_identity{};
        portMUX_TYPE selected_master_mac_mux = portMUX_INITIALIZER_UNLOCKED;
        Mac selected_master_mac{};
        bool selected_master_mac_valid = false;

        void publish_master_mac(const Candidate *candidate)
        {
            portENTER_CRITICAL(&selected_master_mac_mux);
            if (candidate)
            {
                selected_master_mac = candidate->announce.source_mac;
                selected_master_mac_valid = true;
            }
            else
            {
                selected_master_mac.fill(0);
                selected_master_mac_valid = false;
            }
            portEXIT_CRITICAL(&selected_master_mac_mux);
        }

        void write16(uint8_t *p, uint16_t value)
        {
            p[0] = uint8_t(value >> 8);
            p[1] = uint8_t(value);
        }
        bool fresh(int64_t now, int64_t then, int64_t limit)
        {
            return then >= 0 && now >= then && now - then < limit;
        }
        bool timestamp_ns(Timestamp value, int64_t &result)
        {
            if (!valid(value) || value.seconds < 0 || value.seconds > (INT64_MAX - value.nanoseconds) / billion)
                return false;
            result = value.seconds * billion + value.nanoseconds;
            return true;
        }
        bool hardware_time(const eth_mac_time_t &value)
        {
            return (value.seconds || value.nanoseconds) && value.nanoseconds < billion;
        }

        esp_err_t ethernet_input(esp_eth_handle_t, uint8_t *buffer, uint32_t length, void *context, void *info)
        {
            if (!buffer)
                return ESP_ERR_INVALID_ARG;
            udp::Datagram datagram;
            const auto decoded = udp::decode(buffer, length, settings::ptp_ipv6, datagram);
            if (decoded != udp::Result::ignored)
            {
                ptp_received.fetch_add(1, std::memory_order_relaxed);
                if (decoded == udp::Result::valid && length <= settings::ethernet_max_ptp_frame_bytes)
                {
                    Reception reception;
                    reception.received_us = esp_timer_get_time();
                    reception.generation = receive_generation.load(std::memory_order_acquire);
                    reception.link_epoch = link_epoch.load(std::memory_order_acquire);
                    reception.length = uint16_t(length);
                    std::memcpy(reception.frame.data(), buffer, length);
#if SOC_EMAC_IEEE1588V2_SUPPORTED
                    if (info)
                    {
                        const auto stamp = *static_cast<const eth_mac_time_t *>(info);
                        reception.hardware = hardware_time(stamp);
                        if (reception.hardware)
                            reception.timestamp = {stamp.seconds, stamp.nanoseconds};
                    }
#else
                    (void)info;
#endif
                    if (xQueueSend(queue, &reception, 0) == pdTRUE)
                    {
                        ptp_queued.fetch_add(1, std::memory_order_relaxed);
                    }
                    else
                    {
                        ptp_dropped.fetch_add(1, std::memory_order_relaxed);
                        overflow.store(1, std::memory_order_release);
                    }
                }
                else
                    ptp_dropped.fetch_add(1, std::memory_order_relaxed);
                std::free(buffer);
                return ESP_OK;
            }
            // esp_netif takes ownership, including freeing the RX buffer. Never
            // swallow DHCP, ARP or IPv6 ND while intercepting timestamped PTP.
            return esp_netif_receive(static_cast<esp_netif_t *>(context), buffer, length, nullptr);
        }

        void ethernet_event(void *, esp_event_base_t, int32_t id, void *data)
        {
            if (!data || *static_cast<esp_eth_handle_t *>(data) != ethernet)
                return;
            if (id == ETHERNET_EVENT_CONNECTED || id == ETHERNET_EVENT_DISCONNECTED || id == ETHERNET_EVENT_STOP)
            {
                link_state.store(id == ETHERNET_EVENT_CONNECTED ? 1 : 0, std::memory_order_release);
                link_epoch.fetch_add(1, std::memory_order_acq_rel);
                if (id == ETHERNET_EVENT_CONNECTED)
                {
                    const esp_err_t error = esp_netif_create_ip6_linklocal(netif);
                    if (error != ESP_OK)
                        ESP_LOGE(Tag, "IPv6 link-local creation failed: %s", esp_err_to_name(error));
                }
            }
        }

        esp_err_t restore_link_configuration()
        {
#if SOC_EMAC_IEEE1588V2_SUPPORTED
            // Ordinary link changes do not reset the IDF PHC. Re-enabling PTP itself
            // would reset the addend behind the clock owner's back, so never do that.
            esp_err_t error = esp_eth_mac_enable_ts4all(mac, true);
            if (error != ESP_OK)
                return error;
            {
                bool all = true;
                error = esp_eth_ioctl(ethernet, ETH_CMD_S_ALL_MULTICAST, &all);
                if (error != ESP_OK)
                    return error;
            }
            return ESP_OK;
#else
            return ESP_ERR_NOT_SUPPORTED;
#endif
        }

        void close_memberships()
        {
            network_ready = false;
            for (int &fd : membership_sockets)
            {
                if (fd >= 0 && close(fd) != 0)
                    ESP_LOGE(Tag, "multicast socket close failed: errno=%d", errno);
                fd = -1;
            }
        }

        bool join_memberships()
        {
            const int index = esp_netif_get_netif_impl_index(netif);
            if (index <= 0)
            {
                ESP_LOGE(Tag, "Ethernet interface index unavailable");
                return false;
            }
            for (unsigned n = 0; n < 2; ++n)
            {
                const int fd = socket(settings::ptp_ipv6 ? AF_INET6 : AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                membership_sockets[n] = fd;
                bool ok = fd >= 0;
                if (ok && settings::ptp_ipv6)
                {
                    const int only_v6 = 1;
                    sockaddr_in6 local{};
                    local.sin6_family = AF_INET6;
                    local.sin6_port = htons(n ? 320 : 319);
                    local.sin6_scope_id = static_cast<uint32_t>(index);
                    ipv6_mreq membership{};
                    std::memcpy(&membership.ipv6mr_multiaddr, udp::ipv6_group, 16);
                    membership.ipv6mr_interface = static_cast<unsigned>(index);
                    ok = setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &only_v6, sizeof(only_v6)) == 0 &&
                         bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0 &&
                         setsockopt(fd, IPPROTO_IPV6, IPV6_JOIN_GROUP, &membership, sizeof(membership)) == 0;
                }
                else if (ok)
                {
                    sockaddr_in local{};
                    local.sin_family = AF_INET;
                    local.sin_port = htons(n ? 320 : 319);
                    ip_mreq membership{};
                    std::memcpy(&membership.imr_multiaddr, udp::ipv4_group, 4);
                    std::memcpy(&membership.imr_interface, source_ip.data(), 4);
                    ok = bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0 &&
                         setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)) == 0;
                }
                if (!ok)
                {
                    ESP_LOGE(Tag, "IPv%u multicast bind/join port %u failed: errno=%d; retry in 2 s",
                             settings::ptp_ipv6 ? 6U : 4U, n ? 320U : 319U, errno);
                    close_memberships();
                    return false;
                }
            }
            return true;
        }

        void update_network(int64_t now)
        {
            if (network_ready && network_epoch != link_epoch.load(std::memory_order_acquire))
                close_memberships();
            std::array<uint8_t, 16> address{};
            bool available = link_state.load(std::memory_order_acquire) != 0 && esp_netif_is_netif_up(netif);
            if (available && settings::ptp_ipv6)
            {
                esp_ip6_addr_t ip{};
                available = esp_netif_get_ip6_linklocal(netif, &ip) == ESP_OK;
                if (available)
                {
                    std::memcpy(address.data(), ip.addr, 16);
                    available = address[0] == 0xfe && (address[1] & 0xc0) == 0x80;
                }
            }
            else if (available)
            {
                esp_netif_ip_info_t ip{};
                available = esp_netif_get_ip_info(netif, &ip) == ESP_OK && ip.ip.addr != 0;
                if (available)
                    std::memcpy(address.data(), &ip.ip.addr, 4);
            }
            if (network_ready && (!available || address != source_ip))
            {
                close_memberships();
                link_epoch.fetch_add(1, std::memory_order_acq_rel);
                ESP_LOGW(Tag, "IP address/link changed; old PTP exchanges discarded");
            }
            if (!available || network_ready || now < network_retry_us)
                return;
            source_ip = address;
            network_retry_us = now + 2'000'000;
            if (!join_memberships())
                return;
            network_ready = true;
            network_epoch = link_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
            char text[INET6_ADDRSTRLEN]{};
            if (!inet_ntop(settings::ptp_ipv6 ? AF_INET6 : AF_INET, source_ip.data(), text, sizeof(text)))
                ESP_LOGE(Tag, "source IP formatting failed: errno=%d", errno);
            ESP_LOGI(Tag, "Hostname=%s; IPv%u=%s; joined %s UDP 319/320 (unsigned two-step E2E)",
                     settings::hostname, settings::ptp_ipv6 ? 6U : 4U, text,
                     settings::ptp_ipv6 ? "ff02::181" : "224.0.1.129");
        }

        SelectionConfig selection_config()
        {
            SelectionConfig config;
            config.capacity = settings::candidate_capacity;
            config.qualification_count = settings::announce_qualification_count;
            config.min_log_interval = settings::announce_log_interval_min;
            config.max_log_interval = settings::announce_log_interval_max;
            config.min_expiry_ns = settings::announce_expiry_min_us * 1000;
            config.expiry_intervals = settings::announce_expiry_intervals;
            config.pinned = settings::master_selection == MasterSelection::pinned;
            config.pin_mac = settings::pinned_mac_enabled;
            config.pinned_source = settings::pinned_source_identity;
            config.pinned_mac = settings::pinned_source_mac;
            return config;
        }

        const char *ptp_error_name(Error error)
        {
            switch (error)
            {
            case Error::ok:
                return "ok";
            case Error::invalid:
                return "invalid packet/field";
            case Error::unsupported:
                return "unsupported profile/field";
            case Error::truncated:
                return "truncated packet";
            case Error::wrong_domain:
                return "wrong domain";
            case Error::wrong_destination:
                return "wrong destination";
            case Error::unauthenticated:
                return "unaccepted security decision";
            case Error::stale:
                return "stale/backward sequence";
            case Error::duplicate:
                return "duplicate sequence";
            case Error::mismatch:
                return "identity/security mismatch";
            case Error::unavailable:
                return "unavailable";
            case Error::ambiguous:
                return "ambiguous";
            case Error::overflow:
                return "arithmetic overflow";
            case Error::capacity:
                return "capacity exhausted";
            }
            return "unknown";
        }

        void identity_text(const Identity &identity, char (&text)[25])
        {
            std::snprintf(text, sizeof(text), "%02x%02x%02x%02x%02x%02x%02x%02x/%02x%02x",
                          identity[0], identity[1], identity[2], identity[3], identity[4],
                          identity[5], identity[6], identity[7], identity[8], identity[9]);
        }

        class Worker
        {
        public:
            void run();

        private:
            void barrier(const ClockSnapshot &snapshot, int64_t cutoff);
            void invalidate(const char *reason);
            void report_lock(const ClockSnapshot &snapshot, int64_t now);
            void elect(const char *reason, bool refresh);
            void process(const Reception &reception, int64_t now);
            void take_pair(int64_t now);
            void send_delay(int64_t now);
            RestrictedFollowerSelection selection_{selection_config()};
            ExchangeMatcher matcher_{std::min(settings::max_exchange_age_us, settings::max_pair_age_us) * 1000,
                                     settings::delay_response_timeout_us * 1000, settings::pending_pair_capacity};
            SyncRate rate_;
            int64_t delay_rate_ppb_ = 0;
            ClockSnapshot mapping_{};
            Candidate selected_{};
            Pair pair_{};
            uint64_t selection_generation_ = UINT64_MAX, master_serial_ = 0;
            uint32_t epoch_ = 0;
            uint16_t delay_sequence_ = 0, last_sync_ = 0, last_follow_ = 0;
            int64_t cutoff_us_ = 0, minimum_phc_ns_ = 0, last_pair_us_ = -1;
            int64_t next_delay_us_ = 0, delay_started_us_ = 0, sync_timeout_us_ = settings::sync_freshness_min_us;
            bool up_ = false, have_pair_ = false, delay_active_ = false;
            bool have_sync_sequence_ = false, have_follow_sequence_ = false;
            uint32_t parsed_ = 0, parse_rejected_ = 0, announces_ = 0, selection_rejected_ = 0;
            std::array<uint32_t, 16> message_counts_{};
            uint32_t self_filtered_ = 0, stale_announces_ = 0;
            uint32_t mac_collisions_ = 0, sync_timestamp_failures_ = 0, delay_tx_failures_ = 0, exchanges_ = 0;
            uint32_t delay_timeouts_ = 0;
            int64_t last_exchange_us_ = -1;
            bool was_locked_ = false;
            Identity last_source_{};
            Mac last_source_mac_{};
            int64_t last_diagnostic_us_ = 0;
            int64_t last_collision_us_ = -5'000'000;
            const char *last_rejection_ = "none";
        };

        void Worker::barrier(const ClockSnapshot &snapshot, int64_t cutoff)
        {
            mapping_ = snapshot;
            cutoff_us_ = std::max(cutoff_us_, cutoff);
            minimum_phc_ns_ = snapshot.tai_ns;
            have_pair_ = delay_active_ = false;
            rate_.reset();
            matcher_.reset();
            if (selected_.occupied)
                matcher_.select(selected_.announce.source, local_identity, settings::ptp_domain, snapshot.generation);
            receive_generation.store(snapshot.generation, std::memory_order_release);
        }
        void Worker::invalidate(const char *reason)
        {
            last_rejection_ = reason;
            clock_invalidate();
            barrier(clock_snapshot(), esp_timer_get_time());
        }
        // Explain every LOCKED -> unlocked transition; the per-second state line cannot.
        void Worker::report_lock(const ClockSnapshot &snapshot, int64_t now)
        {
            if (was_locked_ && !snapshot.locked)
            {
                const long long since_ms = last_exchange_us_ >= 0 ? (now - last_exchange_us_) / 1000 : -1;
                ESP_LOGW(Tag, "lock lost -> %s; cause: %s; last PTP event: %s; last offset=%+" PRId64 "ns delay=%" PRId64 "ns bridge error=%+" PRId64 "ns; last E2E measurement %lld ms ago; Delay_Resp timeouts=%" PRIu32,
                         clock_state_name(snapshot.state), snapshot.unlock_reason, last_rejection_, snapshot.offset_ns,
                         snapshot.delay_ns, snapshot.bridge_error_ns, since_ms, delay_timeouts_);
            }
            was_locked_ = snapshot.locked;
        }
        void Worker::elect(const char *reason, bool refresh)
        {
            const Candidate *candidate = selection_.selected();
            if (selection_generation_ != selection_.generation())
            {
                char previous[25]{}, next[25]{};
                identity_text(selected_.announce.source, previous);
                identity_text(candidate ? candidate->announce.source : Identity{}, next);
                selection_generation_ = selection_.generation();
                selected_ = candidate ? *candidate : Candidate{};
                publish_master_mac(candidate);
                rate_.reset();
                have_sync_sequence_ = have_follow_sequence_ = false;
                last_pair_us_ = -1;
                sync_timeout_us_ = settings::sync_freshness_min_us;
                // A fresh serial also invalidates same-identity requalification and key changes.
                clock_master(0, false, 0, false, false);
                if (candidate)
                    ++master_serial_;
                const Packet &p = selected_.announce;
                ESP_LOGI(Tag, "master %s -> %s (%s); GM=%02x%02x%02x%02x%02x%02x%02x%02x rank=%u/%u/%u/%u/%u",
                         previous, next, reason, p.dataset.grandmaster[0], p.dataset.grandmaster[1],
                         p.dataset.grandmaster[2], p.dataset.grandmaster[3], p.dataset.grandmaster[4],
                         p.dataset.grandmaster[5], p.dataset.grandmaster[6], p.dataset.grandmaster[7],
                         p.dataset.priority1, p.dataset.clock_class, p.dataset.accuracy, p.dataset.variance, p.dataset.priority2);
                if (candidate)
                {
                    char address[INET6_ADDRSTRLEN]{};
                    if (inet_ntop(settings::ptp_ipv6 ? AF_INET6 : AF_INET, p.source_ip.data(), address, sizeof(address)))
                        ESP_LOGI(Tag, "Selected master UDP/IPv%u address=%s", settings::ptp_ipv6 ? 6U : 4U, address);
                    else
                        ESP_LOGE(Tag, "master IP formatting failed: errno=%d", errno);
                }
                refresh = candidate != nullptr;
            }
            if (candidate && refresh)
            {
                selected_ = *candidate;
                publish_master_mac(candidate);
                const auto &properties = candidate->announce.properties;
                clock_master(master_serial_, properties.usable(), properties.utc_offset,
                             properties.leap61 || properties.leap59, properties.time_traceable);
            }
            const auto snapshot = clock_snapshot();
            if (snapshot.generation != mapping_.generation || snapshot.frequency_ppb != mapping_.frequency_ppb ||
                snapshot.lock_epoch != mapping_.lock_epoch || (!candidate && selected_.occupied))
            {
                barrier(snapshot, esp_timer_get_time());
            }
        }

        void Worker::take_pair(int64_t now)
        {
            Pair pair;
            if (matcher_.take_pair(now * 1000, pair) != Error::ok)
                return;
            if (!fresh(now * 1000, pair.captured_ns, settings::max_pair_age_us * 1000))
                return;
            pair_ = pair;
            have_pair_ = true;
            rate_.observe(pair, sync_timeout_us_ * 1000);
            last_pair_us_ = pair.captured_ns / 1000;
            int64_t master_ns = 0, received_ns = 0, delta = 0;
            if (!timestamp_ns(pair.t1, master_ns) || !timestamp_ns(pair.t2, received_ns) ||
                difference_ns(pair.t1, pair.t2, delta) != Error::ok)
            {
                invalidate("invalid Sync/Follow_Up timestamp pair");
                return;
            }
            // q17 E2E arithmetic cannot represent boot-sized offsets. Only a coherent
            // hardware pair may seed the owner, never a lone Announce or software time.
            if (!delay_active_ && (!mapping_.utc_valid || delta > 60 * billion || delta < -60 * billion))
            {
                const int64_t correction = pair.correction / 65536;
                if ((correction > 0 && master_ns > INT64_MAX - correction) ||
                    (correction < 0 && master_ns < INT64_MIN - correction))
                {
                    invalidate("Sync correction overflow");
                    return;
                }
                if (clock_seed(master_ns + correction, received_ns, mapping_.generation))
                {
                    barrier(clock_snapshot(), esp_timer_get_time());
                }
            }
        }

        void Worker::process(const Reception &reception, int64_t now)
        {
            if (!up_ || reception.link_epoch != epoch_ || reception.generation != mapping_.generation ||
                reception.received_us <= cutoff_us_ || !fresh(now, reception.received_us, settings::max_pair_age_us))
                return;
            Packet packet;
            const ParseConfig config{settings::ptp_domain, local_mac, settings::ptp_ipv6};
            const Error parsed = parse_ptp(reception.frame.data(), reception.length, config, packet);
            if (parsed != Error::ok)
            {
                ++parse_rejected_;
                last_rejection_ = ptp_error_name(parsed);
                return;
            }
            ++parsed_;
            ++message_counts_[uint8_t(packet.type) & 15];
            last_source_ = packet.source;
            last_source_mac_ = packet.source_mac;
            if (master_mac_collision(packet, local_mac))
            {
                ++mac_collisions_;
                last_rejection_ = "master MAC equals device MAC";
                if (now - last_collision_us_ >= 5'000'000)
                {
                    last_collision_us_ = now;
                    ESP_LOGE(Tag, "PTP master-type traffic has the SAME MAC as this device: %02x:%02x:%02x:%02x:%02x:%02x; assign unique MACs; packet rejected (identity/authentication not trusted)",
                             local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4], local_mac[5]);
                }
                return;
            }
            if (packet.source == local_identity)
            {
                ++self_filtered_;
                last_rejection_ = "source equals local port identity; check duplicate MAC/identity or loopback";
                return;
            }
            // Acceptance means structural validation, NOT authentication. UDP PTP
            // has no key negotiation; deploy on a trusted Ethernet segment.
            const Security security{true, false, 0, 0};
            if (packet.type == MessageType::announce)
            {
                if (!fresh(now, reception.received_us, settings::utc_metadata_validity_us))
                {
                    ++stale_announces_;
                    last_rejection_ = "Announce exceeded UTC metadata age limit";
                    return;
                }
                const Error result = selection_.announce(packet, security, reception.received_us * 1000);
                if (result == Error::ok)
                {
                    ++announces_;
                    const Candidate *candidate = selection_.selected();
                    const bool refresh = candidate && candidate->announce.source == packet.source;
                    elect("qualified Announce", refresh);
                }
                else
                {
                    ++selection_rejected_;
                    last_rejection_ = ptp_error_name(result);
                }
                return;
            }
            if (!selected_.occupied || packet.source != selected_.announce.source ||
                packet.source_mac != selected_.announce.source_mac ||
                packet.source_ip != selected_.announce.source_ip || !(security == selected_.security))
                return;
            const int64_t captured_ns = reception.received_us * 1000;
            if (packet.type == MessageType::sync)
            {
                if (packet.log_interval < settings::sync_log_interval_min || packet.log_interval > settings::sync_log_interval_max)
                    return;
                if (have_sync_sequence_ && !sequence_after(packet.sequence, last_sync_))
                    return;
                int64_t hardware_ns = 0;
                if (!reception.hardware || !timestamp_ns(reception.timestamp, hardware_ns) || hardware_ns < minimum_phc_ns_)
                {
                    ++sync_timestamp_failures_;
                    invalidate("missing/invalid/stale hardware Sync RX timestamp");
                    return;
                }
                const Error error = matcher_.sync(packet, security, reception.timestamp, true, mapping_.generation, captured_ns);
                if (error == Error::ok)
                {
                    have_sync_sequence_ = true;
                    last_sync_ = packet.sequence;
                    int64_t interval_us = 1'000'000;
                    if (packet.log_interval >= 0)
                        interval_us *= int64_t(1) << packet.log_interval;
                    else
                        interval_us /= int64_t(1) << -int(packet.log_interval);
                    sync_timeout_us_ = std::max(settings::sync_freshness_min_us, interval_us * settings::sync_freshness_intervals);
                    take_pair(now);
                }
                else if (error == Error::capacity || error == Error::mismatch)
                    invalidate(error == Error::capacity ? "Sync pending-pair capacity exceeded" : "conflicting Sync");
            }
            else if (packet.type == MessageType::follow_up)
            {
                if (have_follow_sequence_ && !sequence_after(packet.sequence, last_follow_))
                    return;
                const Error error = matcher_.follow_up(packet, security, mapping_.generation, captured_ns);
                if (error == Error::ok)
                {
                    have_follow_sequence_ = true;
                    last_follow_ = packet.sequence;
                    take_pair(now);
                }
                else if (error == Error::capacity || error == Error::mismatch)
                    invalidate(error == Error::capacity ? "Follow_Up pending-pair capacity exceeded" : "conflicting Follow_Up");
            }
            else if (packet.type == MessageType::delay_resp && delay_active_)
            {
                Exchange exchange;
                const Error error = matcher_.delay_response(packet, security, mapping_.generation, captured_ns, exchange);
                if (error != Error::ok)
                    return;
                delay_active_ = false;
                Measurement measurement;
                // A capture window has one fixed PHC rate; its change is followed by a
                // barrier. No queued timestamps from the previous rate are reused.
                const Error calculated = calculate_e2e(exchange, now * 1000, measurement, settings::max_exchange_age_us * 1000,
                                                       settings::max_mean_path_delay_ns, delay_rate_ppb_);
                if (calculated != Error::ok)
                {
                    invalidate(ptp_error_name(calculated));
                    return;
                }
                const auto before = clock_snapshot();
                if (before.generation != mapping_.generation || before.frequency_ppb != mapping_.frequency_ppb ||
                    before.lock_epoch != mapping_.lock_epoch || link_epoch.load(std::memory_order_acquire) != epoch_ ||
                    overflow.load(std::memory_order_acquire))
                {
                    invalidate("clock/link changed during Delay exchange");
                    return;
                }
                ++exchanges_;
                if (clock_measurement(measurement.offset_ns(), measurement.delay_ns(), reception.received_us, mapping_.generation))
                    last_exchange_us_ = reception.received_us;
                barrier(clock_snapshot(), esp_timer_get_time());
            }
        }

        void Worker::send_delay(int64_t now)
        {
            if (!up_ || !have_pair_ || delay_active_ || now < next_delay_us_ ||
                !selected_.occupied || !selected_.announce.properties.usable() ||
                selected_.announce.properties.leap61 || selected_.announce.properties.leap59 ||
                !fresh(now * 1000, pair_.captured_ns, settings::max_pair_age_us * 1000))
                return;
            int64_t measured_rate_ppb = 0;
            if (!rate_.rate(now * 1000, settings::max_pair_age_us * 1000, measured_rate_ppb))
                return;
            next_delay_us_ = now + settings::delay_request_interval_us;
            const auto before = clock_snapshot();
            if (before.generation != mapping_.generation || before.frequency_ppb != mapping_.frequency_ppb ||
                before.lock_epoch != mapping_.lock_epoch || before.state == ClockState::Fault ||
                link_epoch.load(std::memory_order_acquire) != epoch_ || overflow.load(std::memory_order_acquire))
            {
                barrier(before, esp_timer_get_time());
                return;
            }
            std::array<uint8_t, udp::max_frame> frame{};
            uint8_t message[44]{};
            message[0] = uint8_t(MessageType::delay_req);
            message[1] = 2;
            write16(message + 2, 44);
            message[4] = settings::ptp_domain;
            std::copy(local_identity.begin(), local_identity.end(), message + 20);
            const uint16_t sequence = ++delay_sequence_;
            write16(message + 30, sequence);
            message[32] = 1;
            message[33] = 0x7f;
            const size_t length = udp::encapsulate(frame.data(), frame.size(), local_mac.data(),
                                                   source_ip.data(), settings::ptp_ipv6, message, sizeof(message));
            if (!length)
            {
                invalidate("UDP Delay_Req encoding failed");
                return;
            }
            eth_mac_time_t timestamp{};
            const esp_err_t error = esp_eth_transmit_ctrl_vargs(ethernet, &timestamp, 2, frame.data(), length);
            const int64_t sent_us = esp_timer_get_time();
            const auto after = clock_snapshot();
            if (error != ESP_OK || !hardware_time(timestamp) || after.generation != before.generation ||
                after.frequency_ppb != before.frequency_ppb || after.lock_epoch != before.lock_epoch ||
                link_epoch.load(std::memory_order_acquire) != epoch_ || overflow.load(std::memory_order_acquire) ||
                !fresh(sent_us, now, settings::delay_response_timeout_us))
            {
                ++delay_tx_failures_;
                invalidate("Delay_Req TX failed/missing hardware timestamp or changed clock/link");
                return;
            }
            const Timestamp tx{timestamp.seconds, timestamp.nanoseconds};
            if (matcher_.begin_delay(sequence, tx, true, pair_, mapping_.generation, sent_us * 1000) != Error::ok)
            {
                invalidate("Delay_Req exchange could not be started");
                return;
            }
            have_pair_ = false;
            delay_rate_ppb_ = measured_rate_ppb;
            delay_active_ = true;
            delay_started_us_ = sent_us;
        }

        void Worker::run()
        {
            barrier(clock_snapshot(), esp_timer_get_time());
            const TickType_t wait = std::max<TickType_t>(1, pdMS_TO_TICKS(10));
            unsigned batch = 0;
            for (;;)
            {
                const int64_t now = esp_timer_get_time();
                update_network(now);
                const uint32_t epoch = link_epoch.load(std::memory_order_acquire);
                if (epoch != epoch_)
                {
                    epoch_ = epoch;
                    up_ = false;
                    clock_link(false);
                    selection_.reset();
                    elect("link transition", false);
                    if (network_ready)
                    {
                        const esp_err_t error = restore_link_configuration();
                        if (error == ESP_OK)
                        {
                            up_ = true;
                            clock_link(true);
                            ESP_LOGI(Tag, "link up; awaiting qualified Announce and hardware Sync RX/Delay_Req TX");
                        }
                        else
                        {
                            ESP_LOGE(Tag, "link timing/filter qualification failed: %s", esp_err_to_name(error));
                            close_memberships();
                            link_epoch.fetch_add(1, std::memory_order_acq_rel);
                        }
                    }
                    else
                        ESP_LOGI(Tag, "Ethernet/IP/membership not ready; synchronization revoked");
                    barrier(clock_snapshot(), esp_timer_get_time());
                }
                if (overflow.exchange(0, std::memory_order_acq_rel))
                    invalidate("PTP receive queue overflow");
                if (selection_.expire(now * 1000))
                    elect("Announce expired", false);
                clock_tick();
                const auto snapshot = clock_snapshot();
                report_lock(snapshot, now);
                if (snapshot.generation != mapping_.generation || snapshot.frequency_ppb != mapping_.frequency_ppb ||
                    snapshot.lock_epoch != mapping_.lock_epoch)
                    barrier(snapshot, esp_timer_get_time());
                if (delay_active_ && !fresh(now, delay_started_us_, settings::delay_response_timeout_us))
                {
                    // One lost Delay_Resp only abandons this exchange; the PHC keeps its
                    // disciplined rate. Persistent loss revokes LOCKED via measurement_timeout_us.
                    ++delay_timeouts_;
                    last_rejection_ = "Delay_Resp timeout";
                    delay_active_ = false;
                }
                if (last_pair_us_ >= 0 && !fresh(now, last_pair_us_, sync_timeout_us_))
                {
                    last_pair_us_ = -1;
                    invalidate("Sync/Follow_Up timeout");
                }
                if (up_ && !snapshot.locked && now - last_diagnostic_us_ >= 5'000'000)
                {
                    last_diagnostic_us_ = now;
                    ESP_LOGI(Tag, "PTP discovery: rx=%" PRIu32 " dropped=%" PRIu32 " parsed=%" PRIu32 " rejected=%" PRIu32 " Announces=%" PRIu32 " E2E=%" PRIu32 "; last event=%s",
                             ptp_received.load(std::memory_order_relaxed), ptp_dropped.load(std::memory_order_relaxed),
                             parsed_, parse_rejected_, announces_, exchanges_, last_rejection_);
                }
                Reception reception;
                if (xQueueReceive(queue, &reception, wait) == pdTRUE)
                {
                    if (!overflow.load(std::memory_order_acquire) && link_epoch.load(std::memory_order_acquire) == epoch_)
                        process(reception, esp_timer_get_time());
                }
                send_delay(esp_timer_get_time());
                // Bound each busy burst without throttling the supported fast Sync rate.
                if (!uxQueueMessagesWaiting(queue))
                    batch = 0;
                else if (++batch == 8)
                {
                    batch = 0;
                    vTaskDelay(1);
                }
            }
        }
        Worker worker;
        void worker_task(void *) { worker.run(); }

        bool valid_configuration()
        {
            constexpr int64_t max_us = INT64_MAX / 1000;
            const int64_t intervals[] = {settings::announce_expiry_min_us, settings::delay_request_interval_us,
                                         settings::delay_response_timeout_us, settings::max_exchange_age_us, settings::max_pair_age_us,
                                         settings::sync_freshness_min_us, settings::clock_bridge_interval_us, settings::utc_metadata_validity_us};
            for (const int64_t interval : intervals)
                if (interval <= 0 || interval > max_us)
                    return false;
            if (settings::candidate_capacity < 1 || settings::candidate_capacity > 32 ||
                settings::pending_pair_capacity < 1 || settings::pending_pair_capacity > 32 || !settings::announce_expiry_intervals ||
                settings::sync_log_interval_min < -7 || settings::sync_log_interval_max > 4 ||
                settings::sync_log_interval_min > settings::sync_log_interval_max || !settings::sync_freshness_intervals ||
                settings::sync_freshness_intervals > unsigned(max_us / 16'000'000))
                return false;
#if CONFIG_IDF_TARGET_ESP32P4
            if (settings::ethernet_phy != EthernetPhy::ip101 || settings::ethernet_phy_address < 0 || settings::ethernet_phy_address > 31 ||
                (settings::rmii_clock != RmiiClock::external_input && settings::rmii_clock != RmiiClock::internal_output))
                return false;
#elif CONFIG_IDF_TARGET_ESP32S31
            // -1 requests driver PHY-address discovery.
            if (settings::ethernet_phy != EthernetPhy::yt8531 || settings::ethernet_phy_address < -1 || settings::ethernet_phy_address > 31)
                return false;
#endif
            return true;
        }

        // Every Ethernet pad must be a distinct, nonnegative GPIO; -1 marks an
        // optional unused signal and is ignored. Two drivers must never share a pad.
        template <std::size_t N>
        constexpr bool distinct_gpios(const int (&gpios)[N])
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (gpios[i] < -1)
                    return false;
                for (std::size_t j = i + 1; gpios[i] >= 0 && j < N; ++j)
                    if (gpios[i] == gpios[j])
                        return false;
            }
            return true;
        }
#if CONFIG_IDF_TARGET_ESP32P4
        constexpr int EthernetGpios[] = {
            settings::ethernet_mdc_gpio, settings::ethernet_mdio_gpio, settings::ethernet_phy_reset_gpio,
            settings::ethernet_rmii_clock_gpio, settings::ethernet_rmii_tx_enable_gpio, settings::ethernet_rmii_tx0_gpio,
            settings::ethernet_rmii_tx1_gpio, settings::ethernet_rmii_crs_dv_gpio, settings::ethernet_rmii_rx0_gpio,
            settings::ethernet_rmii_rx1_gpio};
#elif CONFIG_IDF_TARGET_ESP32S31
        constexpr int EthernetGpios[] = {
            settings::ethernet_mdc_gpio, settings::ethernet_mdio_gpio, settings::ethernet_phy_reset_gpio,
            settings::ethernet_rgmii_rx_clock_gpio, settings::ethernet_rgmii_tx_clock_gpio,
            settings::ethernet_rgmii_phy_ref_clock_gpio, settings::ethernet_rgmii_tx_control_gpio,
            settings::ethernet_rgmii_tx0_gpio, settings::ethernet_rgmii_tx1_gpio, settings::ethernet_rgmii_tx2_gpio,
            settings::ethernet_rgmii_tx3_gpio, settings::ethernet_rgmii_rx_control_gpio, settings::ethernet_rgmii_rx0_gpio,
            settings::ethernet_rgmii_rx1_gpio, settings::ethernet_rgmii_rx2_gpio, settings::ethernet_rgmii_rx3_gpio};
#endif
        static_assert(distinct_gpios(EthernetGpios), "Ethernet GPIO assignments must be distinct and >= -1");

#if CONFIG_IDF_TARGET_ESP32S31
        // YT8531 (ESP32-S31-Function-CoreBoard-1) post-install setup, mirroring the
        // ESP-IDF v6.1 Ethernet basic example. Run after esp_eth_driver_install()
        // (whose generic PHY reset disables autonegotiation) and before esp_eth_start().
        esp_err_t yt8531_phy_init(esp_eth_handle_t handle)
        {
            constexpr uint32_t ExtAddressRegister = 0x1E;
            constexpr uint32_t ExtDataRegister = 0x1F;
            constexpr uint32_t ChipConfig = 0xA001;   // bit 8: RX clock ~2 ns delay enable
            constexpr uint32_t RgmiiConfig1 = 0xA003; // bits 7:4 / 3:0: TX delay selectors (150 ps steps)
            constexpr uint32_t TxDelaySteps = 13;     // ~1.95 ns

            bool autonegotiation = true;
            esp_err_t error = esp_eth_ioctl(handle, ETH_CMD_S_AUTONEGO, &autonegotiation);
            if (error != ESP_OK)
            {
                ESP_LOGE(Tag, "YT8531: enable autonegotiation failed: %s", esp_err_to_name(error));
                return error;
            }
            const auto modify = [handle](uint32_t ext_register, uint32_t clear_mask, uint32_t set_mask) -> esp_err_t
            {
                uint32_t value = ext_register;
                esp_eth_phy_reg_rw_data_t access{};
                access.reg_value_p = &value;
                access.reg_addr = ExtAddressRegister;
                esp_err_t result = esp_eth_ioctl(handle, ETH_CMD_WRITE_PHY_REG, &access);
                if (result != ESP_OK)
                    return result;
                access.reg_addr = ExtDataRegister;
                result = esp_eth_ioctl(handle, ETH_CMD_READ_PHY_REG, &access);
                if (result != ESP_OK)
                    return result;
                value = (value & ~clear_mask) | set_mask;
                return esp_eth_ioctl(handle, ETH_CMD_WRITE_PHY_REG, &access);
            };
            error = modify(ChipConfig, 0, 1U << 8);
            if (error != ESP_OK)
            {
                ESP_LOGE(Tag, "YT8531: RX delay (ext 0x%04" PRIx32 ") configuration failed: %s", ChipConfig, esp_err_to_name(error));
                return error;
            }
            error = modify(RgmiiConfig1, 0x00FFU, (TxDelaySteps << 4) | TxDelaySteps);
            if (error != ESP_OK)
            {
                ESP_LOGE(Tag, "YT8531: TX delay (ext 0x%04" PRIx32 ") configuration failed: %s", RgmiiConfig1, esp_err_to_name(error));
                return error;
            }
            ESP_LOGI(Tag, "YT8531 RGMII delays configured: RX ~2 ns, TX ~1.95 ns; autonegotiation enabled");
            return ESP_OK;
        }
#endif
    }

    Mac ethernet_mac()
    {
        return local_mac;
    }

    bool ethernet_master_mac(Mac &master_mac)
    {
        portENTER_CRITICAL(&selected_master_mac_mux);
        const bool available = selected_master_mac_valid;
        if (available)
            master_mac = selected_master_mac;
        portEXIT_CRITICAL(&selected_master_mac_mux);
        return available;
    }

    esp_err_t ethernet_link_speed(uint32_t *speed_mbps)
    {
        if (!speed_mbps)
            return ESP_ERR_INVALID_ARG;

        eth_speed_t speed;
        const esp_err_t error = esp_eth_ioctl(ethernet, ETH_CMD_G_SPEED, &speed);
        if (error != ESP_OK)
            return error;

        switch (speed)
        {
        case ETH_SPEED_10M:
            *speed_mbps = 10;
            return ESP_OK;
        case ETH_SPEED_100M:
            *speed_mbps = 100;
            return ESP_OK;
        case ETH_SPEED_1000M:
            *speed_mbps = 1000;
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    esp_err_t ethernet_start()
    {
        if (attempted)
            return ESP_ERR_INVALID_STATE;
        attempted = true;
#if !(CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31) || !CONFIG_ETH_USE_ESP32_EMAC || !SOC_EMAC_IEEE1588V2_SUPPORTED || !CONFIG_ETH_TRANSMIT_MUTEX
        ESP_LOGE(Tag, "requires ESP32-P4 or ESP32-S31 native EMAC, hardware timestamps, and transmit mutex");
        return ESP_ERR_NOT_SUPPORTED;
#else
        static_assert(ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 1, 0));
        if (!valid_configuration())
        {
            ESP_LOGE(Tag, "invalid follower timing/board settings; initialization stopped");
            return ESP_ERR_INVALID_ARG;
        }
        const bool use_mac_override = settings::mac_override[0] != '\0' &&
                                      parse_mac_address(settings::mac_override, local_mac) == Error::ok;
        if (settings::mac_override[0] != '\0' && !use_mac_override)
        {
            ESP_LOGE(Tag, "invalid mac_override '%s': expected six colon-separated hex octets, nonzero unicast; initialization stopped", settings::mac_override);
            return ESP_ERR_INVALID_ARG;
        }
        ESP_LOGI(Tag, "PTP L3 UDP/IPv%u domain=%u; unsigned; qualification=%u Announces; hostname=%s",
                 settings::ptp_ipv6 ? 6U : 4U, settings::ptp_domain,
                 settings::announce_qualification_count, settings::hostname);
        esp_chip_info_t chip{};
        esp_chip_info(&chip);
#if CONFIG_IDF_TARGET_ESP32P4
        constexpr esp_chip_model_t ExpectedChip = CHIP_ESP32P4;
#ifdef CONFIG_ESP32P4_REV_MIN_FULL
        constexpr int RevisionMin = CONFIG_ESP32P4_REV_MIN_FULL;
#else
        constexpr int RevisionMin = 0;
#endif
#ifdef CONFIG_ESP32P4_REV_MAX_FULL
        constexpr int RevisionMax = CONFIG_ESP32P4_REV_MAX_FULL;
#else
        constexpr int RevisionMax = INT_MAX;
#endif
#elif CONFIG_IDF_TARGET_ESP32S31
        constexpr esp_chip_model_t ExpectedChip = CHIP_ESP32S31;
#ifdef CONFIG_ESP32S31_REV_MIN_FULL
        constexpr int RevisionMin = CONFIG_ESP32S31_REV_MIN_FULL;
#else
        constexpr int RevisionMin = 0;
#endif
#ifdef CONFIG_ESP32S31_REV_MAX_FULL
        constexpr int RevisionMax = CONFIG_ESP32S31_REV_MAX_FULL;
#else
        constexpr int RevisionMax = INT_MAX;
#endif
#endif
        // Revisions are encoded major * 100 + minor and only comparable within one chip family.
        const int revision = static_cast<int>(chip.revision);
        if (chip.model != ExpectedChip)
        {
            ESP_LOGE(Tag, "detected chip model %d does not match the ESP32-%s build target", static_cast<int>(chip.model), ESP32_CHIP_TYPE);
            return ESP_ERR_NOT_SUPPORTED;
        }
        if (revision < RevisionMin || revision > RevisionMax)
        {
            ESP_LOGE(Tag, "ESP32-%s revision %u.%u is outside this build's supported revision range",
                     ESP32_CHIP_TYPE, chip.revision / 100, chip.revision % 100);
            return ESP_ERR_NOT_SUPPORTED;
        }
        ESP_LOGI(Tag, "ESP32-%s revision %u.%u; hardware timestamp qualification required", ESP32_CHIP_TYPE, chip.revision / 100, chip.revision % 100);
        esp_err_t error = esp_event_loop_create_default();
        if (error != ESP_OK && error != ESP_ERR_INVALID_STATE)
            return error;
        error = esp_netif_init();
        if (error != ESP_OK)
            return error;
        esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
        netif = esp_netif_new(&netif_config);
        if (!netif)
            return ESP_ERR_NO_MEM;
        error = esp_netif_set_hostname(netif, settings::hostname);
        if (error != ESP_OK)
            return error;
        queue = xQueueCreateStatic(settings::ethernet_rx_queue_capacity, sizeof(Reception), queue_storage, &queue_control);
        if (!queue)
            return ESP_ERR_NO_MEM;
        eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
        mac_config.sw_reset_timeout_ms = 1000;
        mac_config.rx_task_stack_size = 4096;
        eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
        emac_config.smi_gpio.mdc_num = settings::ethernet_mdc_gpio;
        emac_config.smi_gpio.mdio_num = settings::ethernet_mdio_gpio;
#if CONFIG_IDF_TARGET_ESP32P4
        emac_config.interface = EMAC_DATA_INTERFACE_RMII;
        emac_config.clock_config.rmii.clock_mode = settings::rmii_clock == RmiiClock::external_input ? EMAC_CLK_EXT_IN : EMAC_CLK_OUT;
        emac_config.clock_config.rmii.clock_gpio = settings::ethernet_rmii_clock_gpio;
        emac_config.emac_dataif_gpio.rmii.tx_en_num = settings::ethernet_rmii_tx_enable_gpio;
        emac_config.emac_dataif_gpio.rmii.txd0_num = settings::ethernet_rmii_tx0_gpio;
        emac_config.emac_dataif_gpio.rmii.txd1_num = settings::ethernet_rmii_tx1_gpio;
        emac_config.emac_dataif_gpio.rmii.crs_dv_num = settings::ethernet_rmii_crs_dv_gpio;
        emac_config.emac_dataif_gpio.rmii.rxd0_num = settings::ethernet_rmii_rx0_gpio;
        emac_config.emac_dataif_gpio.rmii.rxd1_num = settings::ethernet_rmii_rx1_gpio;
#elif CONFIG_IDF_TARGET_ESP32S31
        emac_config.interface = EMAC_DATA_INTERFACE_RGMII;
        emac_config.clock_config.rgmii.clock_rx_gpio = settings::ethernet_rgmii_rx_clock_gpio;
        emac_config.clock_config.rgmii.clock_tx_gpio = settings::ethernet_rgmii_tx_clock_gpio;
        emac_config.clock_config.rgmii.clock_phy_ref_gpio = settings::ethernet_rgmii_phy_ref_clock_gpio;
        emac_config.emac_dataif_gpio.rgmii.tx_ctl_num = settings::ethernet_rgmii_tx_control_gpio;
        emac_config.emac_dataif_gpio.rgmii.txd0_num = settings::ethernet_rgmii_tx0_gpio;
        emac_config.emac_dataif_gpio.rgmii.txd1_num = settings::ethernet_rgmii_tx1_gpio;
        emac_config.emac_dataif_gpio.rgmii.txd2_num = settings::ethernet_rgmii_tx2_gpio;
        emac_config.emac_dataif_gpio.rgmii.txd3_num = settings::ethernet_rgmii_tx3_gpio;
        emac_config.emac_dataif_gpio.rgmii.rx_ctl_num = settings::ethernet_rgmii_rx_control_gpio;
        emac_config.emac_dataif_gpio.rgmii.rxd0_num = settings::ethernet_rgmii_rx0_gpio;
        emac_config.emac_dataif_gpio.rgmii.rxd1_num = settings::ethernet_rgmii_rx1_gpio;
        emac_config.emac_dataif_gpio.rgmii.rxd2_num = settings::ethernet_rgmii_rx2_gpio;
        emac_config.emac_dataif_gpio.rgmii.rxd3_num = settings::ethernet_rgmii_rx3_gpio;
#endif
        mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
        if (!mac)
            return ESP_ERR_NO_MEM;
        eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
        phy_config.phy_addr = settings::ethernet_phy_address;
        phy_config.reset_gpio_num = settings::ethernet_phy_reset_gpio;
        phy = esp_eth_phy_new_generic(&phy_config);
        if (!phy)
            return ESP_ERR_NO_MEM;
        const esp_eth_config_t driver_config = ETH_DEFAULT_CONFIG(mac, phy);
        error = esp_eth_driver_install(&driver_config, &ethernet);
        if (error != ESP_OK)
            return error;
#if CONFIG_IDF_TARGET_ESP32S31
        // Mandatory: without RGMII delays/autonegotiation the YT8531 link is unusable.
        // Link up/down transitions do not reset the PHY, so this runs once per install.
        error = yt8531_phy_init(ethernet);
        if (error != ESP_OK)
            return error;
#endif
        if (use_mac_override)
        {
            error = esp_eth_ioctl(ethernet, ETH_CMD_S_MAC_ADDR, local_mac.data());
            if (error != ESP_OK)
                return error;
        }
        error = esp_eth_ioctl(ethernet, ETH_CMD_G_MAC_ADDR, local_mac.data());
        if (error != ESP_OK)
            return error;
        if ((local_mac[0] & 1) || std::all_of(local_mac.begin(), local_mac.end(), [](uint8_t b)
                                              { return b == 0; }))
            return ESP_ERR_INVALID_STATE;
        ESP_LOGI(Tag, "ESP32PTPL3Follower Ethernet MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 local_mac[0], local_mac[1], local_mac[2], local_mac[3], local_mac[4], local_mac[5]);
        std::copy_n(local_mac.begin(), 3, local_identity.begin());
        local_identity[3] = 0xff;
        local_identity[4] = 0xfe;
        std::copy_n(local_mac.begin() + 3, 3, local_identity.begin() + 5);
        local_identity[9] = 1;
        error = clock_owner_init(ethernet);
        if (error != ESP_OK)
            return error;
        error = restore_link_configuration();
        if (error != ESP_OK)
            return error;
        eth_mac_time_t first{}, last{};
        if (esp_eth_mac_get_ptp_time(mac, &first) != ESP_OK)
            return ESP_ERR_NOT_SUPPORTED;
        vTaskDelay(std::max<TickType_t>(2, pdMS_TO_TICKS(2)));
        if (esp_eth_mac_get_ptp_time(mac, &last) != ESP_OK || !hardware_time(last) ||
            last.seconds < first.seconds || (last.seconds == first.seconds && last.nanoseconds <= first.nanoseconds) ||
            esp_eth_mac_get_ts_resolution(mac) == 0)
            return ESP_ERR_NOT_SUPPORTED;
        // Attach the normal lwIP Ethernet driver first, then replace only its RX
        // entry point. The custom callback forwards every non-PTP frame to it.
        auto glue = esp_eth_new_netif_glue(ethernet);
        if (!glue)
            return ESP_ERR_NO_MEM;
        error = esp_netif_attach(netif, glue);
        if (error != ESP_OK)
            return error;
        error = esp_eth_update_input_path_info(ethernet, ethernet_input, netif);
        if (error != ESP_OK)
            return error;
        error = esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethernet_event, nullptr, &event_handler);
        if (error != ESP_OK)
            return error;
        error = esp_eth_start(ethernet);
        if (error != ESP_OK)
            return error;
        if (xTaskCreate(worker_task, "ptp_follower", 8192, nullptr, 5, nullptr) != pdPASS)
        {
            esp_eth_stop(ethernet);
            clock_link(false);
            return ESP_ERR_NO_MEM;
        }
        return ESP_OK;
#endif
    }
}
