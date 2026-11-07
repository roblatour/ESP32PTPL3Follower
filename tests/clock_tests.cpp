#include "clock_owner.h"
#include "ESP32PTPL3FollowerSettings.h"
#include "clock_host.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>

namespace {
using namespace follower;
namespace config = follower::settings;
constexpr int64_t ns = 1'000'000'000;
constexpr uint64_t master = 0x1234;
int checks = 0;
int failures = 0;

void check(bool result, const char* expression, int line) {
    ++checks;
    if (!result) {
        ++failures;
        std::cerr << line << ": " << expression << '\n';
    }
}
#define CHECK(...) check((__VA_ARGS__), #__VA_ARGS__, __LINE__)

auto& model() { return clock_host::model(); }
void unchanged(const ClockSnapshot& before, unsigned writes) {
    const auto after = clock_snapshot();
    CHECK(model().calls.writes() == writes);
    CHECK(after.mutations == before.mutations);
    CHECK(after.generation == before.generation);
    CHECK(after.lock_epoch == before.lock_epoch);
    CHECK(after.master_tag == before.master_tag);
}
void announce() { clock_master(master, true, 37, false, true); }
void initialize() {
    CHECK(clock_snapshot().state == ClockState::Starting);
    CHECK(model().calls.writes() == 0);
    static int ethernet;
    CHECK(clock_owner_init(&ethernet) == ESP_OK);
    CHECK(model().calls.init == 1);
    CHECK(model().ethernet == &ethernet);
    CHECK(clock_snapshot().state == ClockState::LinkDown);
    CHECK(clock_owner_init(&ethernet) == ESP_ERR_INVALID_STATE);
    CHECK(model().calls.init == 1);
    CHECK(model().calls.writes() == 0);
    clock_link(true);
    CHECK(clock_snapshot().state == ClockState::Listening);
    announce();
    CHECK(clock_snapshot().state == ClockState::Acquiring);
}
void seed() {
    const auto before = clock_snapshot();
    const int64_t received = model().tai_ns;
    clock_host::advance(25'000);
    const int64_t current = model().tai_ns;
    const int64_t master_time = 1'700'000'037LL * ns + 123'456'789;
    CHECK(clock_seed(master_time, received, before.generation));
    const auto after = clock_snapshot();
    CHECK(after.tai_ns == current + master_time - received);
    CHECK(after.utc_ns == (after.tai_ns - 37 * ns) / 1000 * 1000);
    CHECK(after.tai_ns - after.utc_ns >= 37 * ns);
    CHECK(after.tai_ns - after.utc_ns < 37 * ns + 1000);
    CHECK(after.utc_valid && !after.locked);
    CHECK(after.generation == before.generation + 1);
    CHECK(after.master_tag == master);
    CHECK(after.mutations == before.mutations + 2);
    CHECK(model().calls.phc_steps == 1);
    CHECK(model().calls.utc_steps == 1);
    CHECK(model().pending_adjustment_us == 0);
    const unsigned writes = model().calls.writes();
    CHECK(!clock_measurement(0, 1000, model().now_us, before.generation));
    CHECK(!clock_seed(master_time, received, before.generation));
    CHECK(!clock_seed(model().tai_ns, model().tai_ns, after.generation));
    unchanged(after, writes);
}
void sample(int64_t offset = 0) {
    clock_host::advance(10'000);
    const auto token = clock_snapshot();
    CHECK(clock_measurement(offset, 1000, model().now_us, token.generation));
}
void lock() {
    for (unsigned index = 0; index < config::lock_sample_count; ++index) {
        sample();
        CHECK(clock_snapshot().locked == (index + 1 == config::lock_sample_count));
    }
    const auto token = clock_snapshot();
    CHECK(token.state == ClockState::Locked);
    CHECK(token.utc_valid && token.time_traceable && !token.leap_ambiguous);
}
void setup_locked() { initialize(); seed(); lock(); }
void lifecycle() {
    initialize();
    const auto unseeded = clock_snapshot();
    CHECK(!unseeded.utc_valid && !unseeded.locked);
    seed();
    lock();
    const auto before = clock_snapshot();
    const unsigned writes = model().calls.writes();
    CHECK(!clock_measurement(0, 1000, model().now_us, before.generation));
    CHECK(!clock_measurement(0, config::max_mean_path_delay_ns + 1, model().now_us + 1, before.generation));
    CHECK(!clock_measurement(0, 1000, model().now_us - config::max_exchange_age_us - 1, before.generation));
    unchanged(before, writes);
    clock_link(false);
    const auto down = clock_snapshot();
    CHECK(!down.locked && down.utc_valid && down.state == ClockState::LinkDown);
    CHECK(down.master_tag == 0 && !down.time_traceable);
    CHECK(down.generation == before.generation + 1);
    CHECK(down.lock_epoch == before.lock_epoch + 1);
    CHECK(model().calls.writes() == writes);
    CHECK(!clock_seed(model().tai_ns + ns, model().tai_ns, down.generation));
    clock_link(false);
    CHECK(clock_snapshot().generation == down.generation);
    clock_link(true);
    CHECK(clock_snapshot().state == ClockState::Listening);
}

void metadata() {
    setup_locked();
    auto before = clock_snapshot();
    unsigned writes = model().calls.writes();
    clock_host::advance(1000);
    clock_master(master, true, 37, false, false);
    CHECK(clock_snapshot().locked && !clock_snapshot().time_traceable);
    unchanged(before, writes);
    for (int variant = 0; variant < 6; ++variant) {
        before = clock_snapshot();
        writes = model().calls.writes();
        const uint64_t tag = variant == 0 ? master + 1 : (variant == 5 ? 0 : master);
        const bool valid = variant != 1;
        const int16_t offset = variant == 2 ? 38 : (variant == 4 ? -1 : 37);
        clock_master(tag, valid, offset, variant == 3, true);
        const auto changed = clock_snapshot();
        CHECK(!changed.locked && changed.utc_valid);
        CHECK(changed.generation == before.generation + 1);
        CHECK(changed.lock_epoch == before.lock_epoch + 1);
        CHECK(model().calls.writes() == writes);
        CHECK(!clock_measurement(0, 1000, model().now_us, before.generation));
        if (variant == 1 || variant >= 3) {
            CHECK(!clock_seed(model().tai_ns + ns, model().tai_ns, changed.generation));
            CHECK(!clock_measurement(0, 1000, model().now_us, changed.generation));
        }
        announce();
        lock();
    }
    before = clock_snapshot();
    clock_host::advance(config::utc_metadata_validity_us + 1);
    writes = model().calls.writes();
    CHECK(!clock_snapshot().locked);
    CHECK(!clock_seed(model().tai_ns + ns, model().tai_ns, before.generation));
    CHECK(!clock_measurement(0, 1000, model().now_us, before.generation));
    CHECK(model().calls.writes() == writes);
    clock_tick();
    CHECK(clock_snapshot().state == ClockState::Unsynchronized);
    announce();
    lock();
    before = clock_snapshot();
    clock_invalidate();
    const auto invalid = clock_snapshot();
    CHECK(!invalid.locked && invalid.utc_valid);
    CHECK(invalid.generation == before.generation);
    CHECK(invalid.lock_epoch == before.lock_epoch + 1);
    lock();
}

void snapshots() {
    setup_locked();
    const auto before = clock_snapshot();
    const auto calls = model().calls;
    const auto addend = model().addend;
    for (int index = 0; index < 20; ++index) CHECK(clock_snapshot().locked);
    CHECK(model().calls.phc_reads == calls.phc_reads + 40);
    CHECK(model().calls.utc_reads == calls.utc_reads + 20);
    unchanged(before, calls.writes());
    model().utc_ns += config::max_clock_bridge_error_ns + 1000;
    CHECK(!clock_snapshot().locked);
    CHECK(clock_snapshot().state == ClockState::Unsynchronized);
    unchanged(before, calls.writes());
    model().utc_ns -= config::max_clock_bridge_error_ns + 1000;
    CHECK(clock_snapshot().locked);
    model().fail_read = true;
    CHECK(!clock_snapshot().locked);
    CHECK(!clock_snapshot().utc_valid);
    unchanged(before, calls.writes());
    model().fail_read = false;
    CHECK(clock_snapshot().locked);
    const int64_t now = model().now_us;
    clock_host::advance(config::measurement_timeout_us + 1);
    CHECK(!clock_snapshot().locked && clock_snapshot().utc_valid);
    unchanged(before, calls.writes());
    model().now_us = now;
    CHECK(clock_snapshot().locked);
    CHECK(model().addend == addend);
    CHECK(model().calls.takes == model().calls.gives);
}

void steps() {
    setup_locked();
    auto before = clock_snapshot();
    const int64_t current = model().tai_ns;
    const int64_t delta = config::reacquire_step_threshold_ns + 123'456;
    CHECK(clock_seed(current + delta, current, before.generation));
    auto after = clock_snapshot();
    CHECK(after.tai_ns == current + delta);
    CHECK(after.generation == before.generation + 1);
    CHECK(after.lock_epoch == before.lock_epoch + 1);
    CHECK(!after.locked && after.utc_valid);
    CHECK(after.master_tag == before.master_tag);
    lock();
    before = clock_snapshot();
    clock_host::advance(10'000);
    const int64_t tai = model().tai_ns;
    CHECK(!clock_measurement(delta, 1000, model().now_us, before.generation));
    after = clock_snapshot();
    CHECK(after.tai_ns == tai - delta);
    CHECK(!after.locked && after.utc_valid);
    CHECK(after.generation == before.generation + 1);
    CHECK(after.lock_epoch == before.lock_epoch + 1);
    CHECK(after.utc_ns == (after.tai_ns - 37 * ns) / 1000 * 1000);
    lock();
}

// A learned frequency correction must survive reacquisition within one boot.
void learned_frequency() {
    initialize();
    seed();
    for (unsigned index = 0; index < config::lock_sample_count; ++index) sample(50);
    CHECK(clock_snapshot().locked);
    sample(0);
    const int32_t learned = clock_snapshot().frequency_ppb;
    CHECK(learned < -100);
    clock_invalidate();
    CHECK(!clock_snapshot().locked);
    sample(0);
    CHECK(std::abs(clock_snapshot().frequency_ppb - learned) <= 2);
    const auto before = clock_snapshot();
    const int64_t current = model().tai_ns;
    CHECK(clock_seed(current + config::reacquire_step_threshold_ns + 1, current, before.generation));
    CHECK(std::abs(clock_snapshot().frequency_ppb - learned) <= 2);
    sample(0);
    CHECK(std::abs(clock_snapshot().frequency_ppb - learned) <= 2);
}

// Field failure: a reader preempted between its PHC and CLOCK_REALTIME reads
// reported a false >100 us bridge error, steered UTC by it, and revoked LOCKED.
void preempted_bridge() {
    setup_locked();
    const auto before = clock_snapshot();
    const int64_t preemption_us = 4 * config::max_clock_bridge_error_ns / 1000;
    model().preempt_us = preemption_us;
    model().preempt_reads = 1;
    CHECK(clock_snapshot().locked);
    CHECK(model().preempt_reads == 0);
    // Due bridge maintenance must not adjtime by the false error.
    clock_host::advance(config::clock_bridge_interval_us + 1);
    const unsigned adjustments = model().calls.adjustments;
    model().pending_adjustment_us = 0;
    model().preempt_reads = 1;
    clock_tick();
    CHECK(model().calls.adjustments == adjustments + 1);
    CHECK(model().pending_adjustment_us == 0);
    CHECK(clock_snapshot().locked);
    // The LOCKED check itself is preempted (bridge maintenance not yet due).
    clock_host::advance(1000);
    model().preempt_reads = 2;
    clock_tick();
    CHECK(model().pending_adjustment_us == 0);
    auto after = clock_snapshot();
    CHECK(after.locked && after.lock_epoch == before.lock_epoch);
    CHECK(std::strcmp(after.unlock_reason, "none") == 0);

    // Persistent preemption leaves the bridge unverified; that is tolerated
    // briefly, never used for steering, and then honestly revokes LOCKED.
    model().preempt_us = config::max_clock_bridge_read_window_ns / 1000 + 10;
    model().preempt_reads = std::numeric_limits<unsigned>::max();
    const unsigned held_adjustments = model().calls.adjustments;
    clock_host::advance(config::clock_bridge_interval_us + 1);
    clock_tick();
    CHECK(clock_snapshot().locked);
    CHECK(model().calls.adjustments == held_adjustments);
    clock_host::advance(config::max_clock_bridge_unverified_us + 1);
    clock_tick();
    after = clock_snapshot();
    CHECK(!after.locked && after.utc_valid);
    CHECK(after.lock_epoch == before.lock_epoch + 1);
    CHECK(std::strstr(after.unlock_reason, "bridge") != nullptr);
    CHECK(model().calls.adjustments == held_adjustments);
    model().preempt_reads = 0;

    // A precisely measured bridge error is still a real fault and is reported.
    announce();
    lock();
    const auto relocked = clock_snapshot();
    model().utc_ns += config::max_clock_bridge_error_ns + 1000;
    clock_tick();
    after = clock_snapshot();
    CHECK(!after.locked);
    CHECK(after.lock_epoch == relocked.lock_epoch + 1);
    CHECK(std::strstr(after.unlock_reason, "bridge") != nullptr);
    CHECK(after.bridge_error_ns < -config::max_clock_bridge_error_ns);
}

void init_failure(bool mutex) {
    if (mutex) model().fail_mutex = true;
    else model().init_result = ESP_FAIL;
    CHECK(clock_owner_init(nullptr) == (mutex ? ESP_ERR_NO_MEM : ESP_FAIL));
    CHECK(clock_snapshot().state == (mutex ? ClockState::Starting : ClockState::Fault));
    CHECK(!clock_snapshot().utc_valid && !clock_snapshot().locked);
    CHECK(model().calls.init == (mutex ? 0u : 1u));
    CHECK(model().calls.writes() == 0);
}
void step_failure() {
    setup_locked();
    const auto before = clock_snapshot();
    model().fail_step = true;
    CHECK(!clock_seed(model().tai_ns + config::reacquire_step_threshold_ns + 1,
                      model().tai_ns, before.generation));
    const auto after = clock_snapshot();
    CHECK(after.state == ClockState::Fault && !after.utc_valid && !after.locked);
    CHECK(after.generation == before.generation + 1);
    CHECK(after.lock_epoch == before.lock_epoch + 1);
    const unsigned writes = model().calls.writes();
    announce();
    clock_tick();
    CHECK(!clock_measurement(0, 1000, model().now_us, after.generation));
    CHECK(model().calls.writes() == writes);
}
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const std::string name = argv[1];
    if (name == "lifecycle") lifecycle();
    else if (name == "metadata") metadata();
    else if (name == "snapshots") snapshots();
    else if (name == "steps") steps();
    else if (name == "learned_frequency") learned_frequency();
    else if (name == "init_mutex_failure") init_failure(true);
    else if (name == "init_sdk_failure") init_failure(false);
    else if (name == "step_failure") step_failure();
    else if (name == "preempted_bridge") preempted_bridge();
    else return 2;
    CHECK(model().calls.takes == model().calls.gives);
    std::cout << (failures ? "FAIL: " : "PASS: ") << name << ": " << checks
              << " checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}
