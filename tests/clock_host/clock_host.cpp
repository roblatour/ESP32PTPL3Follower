#include <cstdlib>
#include "clock_host.h"
#include "sys/time.h"
#include "sys/timex.h"

namespace {
constexpr int64_t ns = 1'000'000'000;
void require(bool condition) { if (!condition) std::abort(); }
StaticSemaphore_t* active_mutex = nullptr;
void held() { require(active_mutex && active_mutex->held); }
}

namespace clock_host {
Model& model() { static Model value; return value; }
void advance(int64_t microseconds) {
    require(microseconds >= 0);
    auto& value = model();
    value.now_us += microseconds;
    value.tai_ns += microseconds * 1000;
    value.utc_ns += microseconds * 1000;
}
}

SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage) {
    if (clock_host::model().fail_mutex) return nullptr;
    active_mutex = storage;
    return storage;
}
int xSemaphoreTake(SemaphoreHandle_t mutex, unsigned timeout) {
    require(mutex == active_mutex && !mutex->held && timeout == portMAX_DELAY);
    mutex->held = true;
    ++clock_host::model().calls.takes;
    return 1;
}
int xSemaphoreGive(SemaphoreHandle_t mutex) {
    require(mutex == active_mutex && mutex->held);
    mutex->held = false;
    ++clock_host::model().calls.gives;
    return 1;
}
esp_err_t esp_eth_clock_init(esp_eth_handle_t ethernet, const esp_eth_clock_cfg_t* config) {
    held();
    require(config && config->clock_id == CLOCK_PTP_SYSTEM);
    auto& value = clock_host::model();
    ++value.calls.init;
    value.ethernet = ethernet;
    value.initialized = value.init_result == ESP_OK;
    return value.init_result;
}
int64_t esp_timer_get_time() { held(); return clock_host::model().now_us; }
int clock_host_gettime(int clock, timespec* result) {
    held();
    auto& value = clock_host::model();
    require(result && (clock == CLOCK_PTP_SYSTEM || clock == CLOCK_REALTIME));
    if (clock == CLOCK_PTP_SYSTEM) ++value.calls.phc_reads;
    else ++value.calls.utc_reads;
    if (!value.initialized || value.fail_read) return -1;
    const int64_t time = clock == CLOCK_PTP_SYSTEM ? value.tai_ns : value.utc_ns;
    result->tv_sec = static_cast<time_t>(time / ns);
    result->tv_nsec = static_cast<long>(time % ns);
    if (clock == CLOCK_PTP_SYSTEM && value.preempt_reads) {
        --value.preempt_reads;
        clock_host::advance(value.preempt_us);
    }
    return 0;
}
int clock_host_settime(int clock, const timespec* time) {
    held();
    auto& value = clock_host::model();
    require(clock == CLOCK_PTP_SYSTEM && time && value.initialized);
    ++value.calls.phc_steps;
    if (value.fail_step) return -1;
    value.tai_ns = static_cast<int64_t>(time->tv_sec) * ns + time->tv_nsec;
    return 0;
}
int clock_host_settimeofday(const clock_host_timeval* time, const void* zone) {
    held();
    require(time && !zone);
    auto& value = clock_host::model();
    ++value.calls.utc_steps;
    value.utc_ns = static_cast<int64_t>(time->tv_sec) * ns + time->tv_usec * 1000;
    return 0;
}
int clock_host_adjtime(const clock_host_timeval* correction, clock_host_timeval* previous) {
    held();
    require(correction && !previous);
    auto& value = clock_host::model();
    ++value.calls.adjustments;
    value.pending_adjustment_us = static_cast<int64_t>(correction->tv_sec) * 1'000'000 + correction->tv_usec;
    return 0;
}
int clock_host_adjclock(int clock, clock_host_timex* correction) {
    held();
    require(clock == CLOCK_PTP_SYSTEM && correction && correction->modes == ADJ_FREQUENCY);
    auto& value = clock_host::model();
    ++value.calls.frequency;
    const int64_t next = static_cast<int64_t>(value.addend) * (ns + correction->freq) / ns;
    require(next > 0 && next <= UINT32_MAX);
    value.addend = static_cast<uint32_t>(next);
    return 0;
}
