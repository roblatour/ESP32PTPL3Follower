#pragma once

#include <cstdint>
#include <ctime>

using esp_err_t = int;
using esp_eth_handle_t = void*;
inline constexpr esp_err_t ESP_OK = 0;
inline constexpr esp_err_t ESP_ERR_INVALID_STATE = 1;
inline constexpr esp_err_t ESP_ERR_NO_MEM = 2;
inline constexpr esp_err_t ESP_ERR_INVALID_ARG = 3;
inline constexpr esp_err_t ESP_ERR_NOT_SUPPORTED = 4;
inline constexpr esp_err_t ESP_FAIL = 5;
inline constexpr int host_realtime = 0;
inline constexpr int CLOCK_PTP_SYSTEM = 1;

struct StaticSemaphore_t { bool held = false; };
using SemaphoreHandle_t = StaticSemaphore_t*;
inline constexpr unsigned portMAX_DELAY = ~0u;
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage);
int xSemaphoreTake(SemaphoreHandle_t mutex, unsigned timeout);
int xSemaphoreGive(SemaphoreHandle_t mutex);

struct esp_eth_clock_cfg_t { int clock_id; };
esp_err_t esp_eth_clock_init(esp_eth_handle_t ethernet, const esp_eth_clock_cfg_t* config);
int64_t esp_timer_get_time();
inline constexpr int ETH_PTP_DIGITAL_ROLLOVER = 0;
struct eth_mac_ptp_config_t {
    int roll_type;
    double required_accuracy_ns;
    double clk_src_period_ns;
};
#define ETH_MAC_ESP_PTP_DEFAULT_CONFIG() eth_mac_ptp_config_t{ETH_PTP_DIGITAL_ROLLOVER, 40.0, 20.0}

namespace clock_host {
struct Calls {
    unsigned init = 0;
    unsigned phc_reads = 0;
    unsigned utc_reads = 0;
    unsigned phc_steps = 0;
    unsigned utc_steps = 0;
    unsigned frequency = 0;
    unsigned adjustments = 0;
    unsigned takes = 0;
    unsigned gives = 0;
    unsigned writes() const { return phc_steps + utc_steps + frequency + adjustments; }
};
struct Model {
    int64_t now_us = 10'000'000;
    int64_t tai_ns = 100'000'000'000;
    int64_t utc_ns = 0;
    int64_t pending_adjustment_us = 0;
    // Simulates the reading task being preempted: after each PHC read while
    // preempt_reads is nonzero, every clock advances by preempt_us.
    int64_t preempt_us = 0;
    unsigned preempt_reads = 0;
    uint32_t addend = 0x80000000u;
    bool initialized = false;
    bool fail_mutex = false;
    bool fail_read = false;
    bool fail_step = false;
    esp_err_t init_result = ESP_OK;
    esp_eth_handle_t ethernet = nullptr;
    Calls calls;
};
Model& model();
void advance(int64_t microseconds);
}
