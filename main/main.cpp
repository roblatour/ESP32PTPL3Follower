// ESP32 PTP L3 UDP Follower v1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32PTPL3Follower
//

#include "ESP32PTPL3FollowerSettings.h"
#include "clock_owner.h"
#include "ethernet.h"
#include "follower_core.h"
#include "app_metadata.h"

#include <atomic>
#include <cinttypes>
#include <string>
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    constexpr const char *Tag = "main.cpp";

    // FreeRTOS delays wake only on 1 ms ticks driven by the local crystal, whose
    // phase drifts against PTP time (a 0..1 ms sawtooth in the reported second).
    // Ticks cover the coarse wait; a microsecond esp_timer one-shot covers the
    // final few milliseconds, re-anchored to PTP time every second.
    constexpr int64_t coarse_wait_cap_us = 100'000;
    constexpr int64_t fine_wait_window_us = 3'000;
    constexpr int64_t fine_wait_lead_us = 2'000;
    constexpr int64_t max_catch_up_lines = 5;

    // Indicates whether the "Open for business" message has been written (i.e. the Follower has achieved its initial locked state)
    static std::atomic<bool> Open_for_business{false};

    void wake_reporter(void *task)
    {
        xTaskNotifyGive(static_cast<TaskHandle_t>(task));
    }

    void reporting_task(void *)
    {
        esp_timer_handle_t boundary_timer = nullptr;
        esp_timer_create_args_t timer_args{};
        timer_args.callback = wake_reporter;
        timer_args.arg = xTaskGetCurrentTaskHandle();
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "report_second";
        if (esp_timer_create(&timer_args, &boundary_timer) != ESP_OK)
        {
            boundary_timer = nullptr;
            ESP_LOGW(Tag, "second-boundary timer unavailable; reports use 1 ms tick resolution");
        }
        int64_t last_second = -1;
        int64_t last_observed_utc = 0;
        int64_t last_observed_mono = 0;
        int64_t last_state_report = 0;
        uint32_t generation = 0;
        bool had_valid_time = false;
        bool business_announced = false;
        for (;;)
        {
            // periodically tick the clock and capture its snapshot for reporting
            follower::clock_tick();

            // capture the current clock snapshot for comparison with the previous state
            const auto snapshot = follower::clock_snapshot();

            // Show the current time in three timescales: TAI, UTC (the TAI minus the master's UTC offset), and local time (formatted from the UTC).
            // Keep snapshot.utc_ns as the independently sampled system clock for health checks; its microsecond resolution must not truncate display UTC.
            const int64_t display_utc_ns = snapshot.utc_valid
                                               ? snapshot.tai_ns - static_cast<int64_t>(snapshot.utc_offset) * follower::billion
                                               : 0;

            // determine if there is a discontinuity in the clock or master state
            bool discontinuity = generation != snapshot.generation;
            if (snapshot.utc_valid && last_observed_mono)
            {
                const int64_t elapsed = (snapshot.monotonic_us - last_observed_mono) * 1000;
                const int64_t jump = snapshot.utc_ns - last_observed_utc - elapsed;
                discontinuity = discontinuity || jump < -10'000'000 || jump > 10'000'000;
            }

            if (discontinuity && had_valid_time)
            {
                ESP_LOGW(Tag, "clock/master discontinuity; generation=%" PRIu32 " -> %" PRIu32 "; no catch-up lines", generation, snapshot.generation);
                last_second = -1;
            }

            generation = snapshot.generation;

            if (snapshot.utc_valid)
            {

                if (!business_announced && snapshot.locked && snapshot.state == follower::ClockState::Locked)
                {
                    // (one time) announce after the Follower has locked onto the master clock to indicate the program is 'Open for business'

                    esp_chip_info_t chip_info;
                    esp_chip_info(&chip_info);
                    uint32_t speed_mbps;
                    const esp_err_t speed_error = follower::ethernet_link_speed(&speed_mbps);
                    const auto follower_mac = follower::ethernet_mac();
                    follower::Mac master_mac{};
                    const bool master_mac_available = follower::ethernet_master_mac(master_mac);

                    ESP_LOGI(Tag, "***********************************");
                    ESP_LOGI(Tag, "ESP32-%s revision %u.%u", ESP32_CHIP_TYPE, chip_info.revision / 100, chip_info.revision % 100);
                    ESP_LOGI(Tag, "");
                    ESP_LOGI(Tag, "Hostname: %s", follower::settings::hostname);
                    ESP_LOGI(Tag, "PTP transport: UDP/IPv%u %s ports 319/320; domain %u; unsigned E2E",
                             follower::settings::ptp_ipv6 ? 6U : 4U,
                             follower::settings::ptp_ipv6 ? "ff02::181" : "224.0.1.129",
                             follower::settings::ptp_domain);
                    ESP_LOGI(Tag, "Measured offset=%+" PRId64 " ns; mean path delay=%" PRId64 " ns; TAI-UTC=%d s",
                             snapshot.offset_ns, snapshot.delay_ns, snapshot.utc_offset);
                    if (master_mac_available)
                        ESP_LOGI(Tag, "PTP Master's MAC: %02x:%02x:%02x:%02x:%02x:%02x", master_mac[0], master_mac[1], master_mac[2], master_mac[3], master_mac[4], master_mac[5]);
                    else
                        ESP_LOGE(Tag, "PTP master's MAC: unavailable");
                    ESP_LOGI(Tag, "Follower's MAC:   %02x:%02x:%02x:%02x:%02x:%02x", follower_mac[0], follower_mac[1], follower_mac[2], follower_mac[3], follower_mac[4], follower_mac[5]);
                    ESP_LOGI(Tag, "");
                    if (speed_error == ESP_OK)
                        ESP_LOGI(Tag, "Ethernet link speed: %" PRIu32 " Mbps", speed_mbps);
                    else
                        ESP_LOGE(Tag, "Failed to get Ethernet link speed: %s", esp_err_to_name(speed_error));
                    ESP_LOGI(Tag, "");
                    ESP_LOGI(Tag, "ESP32PTPL3Follower is Open for business");
                    ESP_LOGI(Tag, "");
                    ESP_LOGI(Tag, "***********************************");
                    business_announced = true;

                    Open_for_business.store(true, std::memory_order_release);
                }

                const int64_t second = display_utc_ns / follower::billion;

                // If the reporter was delayed, report each missed second explicitly (marked late) instead of silently omitting it.
                // The following code handles reporting of missed seconds if the reporter was delayed.

                // However, a clock step or master change resets last_second, so no catch-up then.

                // Of note: this program, without modifications, should not have any missed seconds.
                // The code below will however report any missed seconds if they occur due to additional code being added
                // that introduces delays between reported seconds - for example, heavy higher priority processing tasks,
                // extended NVS writes, or blocking operations.

                if (follower::settings::second_reporting_enabled && last_second >= 0 && second > last_second + 1)
                {
                    const int64_t missed = second - last_second - 1;
                    const int64_t late_ms = (display_utc_ns - (last_second + 1) * follower::billion) / 1'000'000;
                    const int64_t shown = missed < max_catch_up_lines ? missed : max_catch_up_lines;
                    for (int64_t s = second - shown; s < second; ++s)
                    {
                        std::string text;
                        const follower::Timestamp tai{s + static_cast<int64_t>(snapshot.utc_offset), 0};
                        const follower::Timestamp utc{s, 0};
                        if (follower::format_time_report(tai, utc, follower::settings::timeZoneSpec, text) == follower::Error::ok)
                            ESP_LOGW(Tag, "%s - clock state: %s (catch-up: reporter woke %" PRId64 " ms late)",
                                     text.c_str(), follower::clock_state_name(snapshot.state), late_ms);
                    }
                    if (missed > shown)
                        ESP_LOGW(Tag, "reporter stalled; %" PRId64 " earlier second(s) not listed", missed - shown);
                }

                if (follower::settings::second_reporting_enabled && second != last_second)
                {
                    std::string text;
                    const follower::Timestamp tai{snapshot.tai_ns / follower::billion, static_cast<uint32_t>(snapshot.tai_ns % follower::billion)};
                    const follower::Timestamp utc{second, static_cast<uint32_t>(display_utc_ns % follower::billion)};
                    const auto error = follower::format_time_report(tai, utc, follower::settings::timeZoneSpec, text);
                    if (error == follower::Error::ok)
                    {
                        // Display the TAI, UTC and Local date and time along with the clock state. For example:
                        // TAI=2026-10-28T19:02:15.000128860 UTC=2026-10-28T19:01:38.000128860Z Local=2026-10-28 3:01:38 PM EDT (UTC-04:00) - clock state: LOCKED

                        ESP_LOGI(Tag, "%s - clock state: %s", text.c_str(), follower::clock_state_name(snapshot.state));

                        /*
                        // Additional snapshot information can be logged here if needed, this includes:
                        // snapshot.master_tag, snapshot.offset_ns, snapshot.delay_ns, snapshot.utc_offset, snapshot.time_traceable. For example:
                        // master=0000000000000001 offset=-112ns delay=21345ns TAI-UTC=37 traceable=0

                        ESP_LOGI(Tag, "master=%016" PRIx64 " offset=%+" PRId64 "ns delay=%" PRId64 "ns TAI-UTC=%d traceable=%d", snapshot.master_tag, snapshot.offset_ns, snapshot.delay_ns, snapshot.utc_offset, snapshot.time_traceable);

                        */
                    }
                    else
                        ESP_LOGE(Tag, "TAI/UTC/Local formatting failed: %d; state=%s", static_cast<int>(error), follower::clock_state_name(snapshot.state));
                    last_second = second;
                }
                had_valid_time = true;
                last_observed_utc = snapshot.utc_ns;
                last_observed_mono = snapshot.monotonic_us;
            }
            else if (snapshot.monotonic_us - last_state_report >= 1'000'000)
            {
                ESP_LOGI(Tag, "%s; TAI/UTC/Local not established", follower::clock_state_name(snapshot.state));
                last_state_report = snapshot.monotonic_us;
            }
            // Seconds without established time are never back-filled as catch-up lines.
            if (!snapshot.utc_valid)
                last_second = -1;

            // Put the reporting task to sleep until almost at the next second boundary
            // It uses two stages: coarse sleeping when the boundary is far away, and a higher-resolution timer when it is close.
            int64_t wait_us = coarse_wait_cap_us;
            if (snapshot.utc_valid)
            {
                const int64_t boundary_monotonic_us = snapshot.monotonic_us + (follower::billion - display_utc_ns % follower::billion + 999) / 1000;
                const int64_t until_boundary = boundary_monotonic_us - esp_timer_get_time();
                if (until_boundary <= 0)
                    continue;
                if (boundary_timer && until_boundary <= fine_wait_window_us)
                {
                    ulTaskNotifyTake(pdTRUE, 0);
                    const int64_t remaining_us = boundary_monotonic_us - esp_timer_get_time();
                    if (remaining_us <= 0)
                        continue;
                    if (esp_timer_start_once(boundary_timer, static_cast<uint64_t>(remaining_us + 1)) == ESP_OK)
                    {
                        if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10)))
                            esp_timer_stop(boundary_timer);
                        continue;
                    }
                }
                const int64_t coarse_us = boundary_timer ? until_boundary - fine_wait_lead_us : until_boundary;
                if (coarse_us < wait_us)
                    wait_us = coarse_us;
            }
            vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(wait_us / 1000 > 0 ? wait_us / 1000 : 1)));
        }
    }
}

void write_opening_messages_to_the_console()
{

    // set the global console log verbosity level
    esp_log_level_set("*", static_cast<esp_log_level_t>(follower::settings::log_verbosity));

    // log that the application start up
    vTaskDelay(pdMS_TO_TICKS(100));

    // the values below are drawn from the two CMakeLists.txt files (one in the root folder and one in the /main folder)
    const app_metadata_t *meta = get_app_metadata();
    ESP_LOGI(Tag, "");
    ESP_LOGI(Tag, "******************* Application Startup *******************");
    ESP_LOGI(Tag, "%s v%s", meta->project_name, meta->version);
    ESP_LOGI(Tag, "%s", meta->copyright);
    ESP_LOGI(Tag, "License: %s", meta->license);
    ESP_LOGI(Tag, "Website: %s", meta->homepage);
    ESP_LOGI(Tag, "Built: %s %s; ESP-IDF: %s", meta->build_date, meta->build_time, esp_get_idf_version());
    ESP_LOGI(Tag, "***********************************************************");
    ESP_LOGI(Tag, "");
}

extern "C" void app_main()
{

    write_opening_messages_to_the_console();

    // configure the timezone based on the settings
    if (follower::configure_timezone(follower::settings::timeZoneSpec) != follower::Error::ok)
    {
        ESP_LOGE(Tag, "invalid POSIX display timezone; initialization stopped");
        return;
    }
    // log the initial configuration and status of the follower application
    ESP_LOGI(Tag, "ESP32PTPL3Follower: follower-only two-step E2E; PHC=TAI, application=UTC, Local TZ=%s", follower::settings::timeZoneSpec);

    // warn about unsigned timing and the requirement for a trusted LAN
    ESP_LOGW(Tag, "unsigned timing requires trusted LAN; timestamp units are not accuracy");

    // start the Ethernet interface and timestamping subsystem
    const esp_err_t error = follower::ethernet_start();
    if (error != ESP_OK)
    {
        ESP_LOGE(Tag, "Ethernet/timestamp initialization failed: %s; no synchronization claimed", esp_err_to_name(error));
        return;
    }
    // create and start the reporting task for periodic status updates to the console log
    if (xTaskCreate(reporting_task, "reporter", 6144, nullptr, 3, nullptr) != pdPASS)
        ESP_LOGE(Tag, "reporter task allocation failed; second reporting unavailable");

    // Additional logic can be added below as needed

    // Check for the "Open for business" status is you want to have logic dependant on the initial lock being achieved
    //
    //   if (Open_for_business.load(std::memory_order_acquire))
    //   {
    //
    //   }
}