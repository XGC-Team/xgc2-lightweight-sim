#include "io/telemetry_rates.hpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace xsim;
using Json = nlohmann::json;

int main() {
  const TelemetryRates defaults;
  const Json expected{{"localization", 125}, {"local", 30}, {"imu", 30},
                      {"imu_raw", 30}, {"state", 1}, {"extended_state", 1},
                      {"target", 10}};
  assert(defaults.json() == expected);
  const auto changed = defaults.patched({{"localization", 500}, {"local", 15}, {"imu_raw", 0}});
  assert(changed.json().at("localization") == 500 && changed.json().at("local") == 15);
  assert(changed.json().at("imu") == 30 && changed.json().at("imu_raw") == 0);
  assert(changed.caps(8000000).at("localization") == 125);
  assert(changed.caps(8000000).at("local") == 15);
  assert(changed.caps(8000000).at("imu_raw") == 0);
  assert(defaults.json() == expected);

  for (const auto &invalid : std::vector<Json>{
           nullptr, Json::array(), {{"unknown", 10}}, {{"vrpn", 125}}, {{"imu", true}},
           {{"imu", "30"}}, {{"imu", nullptr}}, {{"imu", -1}},
           {{"imu", 1001}}, {{"imu", 1e-20}},
           {{"local", 15}, {"localization", std::numeric_limits<double>::infinity()}},
           {{"imu", std::numeric_limits<double>::quiet_NaN()}}}) {
    bool rejected = false;
    try { (void)defaults.patched(invalid); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected && defaults.json() == expected);
  }

  // All channels share the source frames, while preserving their own phases.
  TelemetrySchedule schedule;
  std::array<size_t, TelemetryRates::count> counts{};
  for (int64_t stamp = 0; stamp < 10000000000LL; stamp += 8000000)
    for (size_t i = 0; i < counts.size(); ++i)
      if (schedule.due(TelemetryGroup(i), stamp, defaults)) ++counts[i];
  assert((counts == std::array<size_t, TelemetryRates::count>{1250, 300, 300, 300, 10, 10, 100}));

  // A 30 Hz request on a 100 Hz source must not drift down to 25 Hz.
  TelemetrySchedule noninteger;
  size_t local_count = 0;
  for (int64_t stamp = 0; stamp < 10000000000LL; stamp += 10000000)
    if (noninteger.due(TelemetryGroup::Local, stamp, defaults)) ++local_count;
  assert(local_count == 300);

  // Snapshot jitter must not make a same-frequency output skip fresh frames.
  // Different provider start times still use the world's snapshot phase.
  const int64_t epoch = 1700000000000000123LL;
  TelemetrySchedule jittered(epoch), later(epoch);
  for (int64_t n = 1; n <= 12; ++n) {
    const auto stamp = epoch + n * 8000000 + (n % 2 ? 1900000 : 100000);
    assert(jittered.due(TelemetryGroup::Localization, stamp, defaults));
    assert(!jittered.due(TelemetryGroup::Localization, stamp, defaults));
    if (n >= 3) assert(later.due(TelemetryGroup::Localization, stamp, defaults));
  }

  TelemetrySchedule limited;
  size_t high_count = 0, disabled_count = 0;
  for (int64_t stamp = 0; stamp < 1000000000; stamp += 8000000) {
    high_count += limited.due(TelemetryGroup::Localization, stamp, changed);
    disabled_count += limited.due(TelemetryGroup::RawImu, stamp, changed);
  }
  assert(high_count == 125 && disabled_count == 0);

  TelemetrySchedule paused;
  assert(paused.due(TelemetryGroup::Local, 0, defaults));
  assert(!paused.due(TelemetryGroup::Local, 0, defaults));
  assert(!paused.due(TelemetryGroup::Local, 0, changed));
  assert(paused.due(TelemetryGroup::Local, 8000000, changed));
  assert(!paused.due(TelemetryGroup::Local, 8000000, changed));
  assert(!paused.due(TelemetryGroup::Local, 16000000, changed));

  // Skipped source frames yield one current sample, never a burst of backfill.
  TelemetrySchedule delayed;
  assert(delayed.due(TelemetryGroup::Local, 0, defaults));
  assert(delayed.due(TelemetryGroup::Local, 2000000000, defaults));
  assert(!delayed.due(TelemetryGroup::Local, 2000000000, defaults));
  assert(!delayed.due(TelemetryGroup::Local, 2000000001, defaults));

  // Each Entity owns its schedule; resetting a generation does not affect peers.
  TelemetrySchedule one, two;
  assert(one.due(TelemetryGroup::Imu, 1000000000, defaults));
  assert(two.due(TelemetryGroup::Imu, 1000000000, defaults));
  one.reset();
  assert(one.due(TelemetryGroup::Imu, 1000000000, defaults));
  assert(!two.due(TelemetryGroup::Imu, 1000000000, defaults));
  one.reset();
  assert(one.due(TelemetryGroup::Imu, 0, defaults));
  assert(!one.due(TelemetryGroup::Imu, 0, defaults));

  const auto maximum = std::numeric_limits<int64_t>::max();
  TelemetrySchedule ending;
  assert(ending.due(TelemetryGroup::Localization, maximum - 8000000, defaults));
  assert(ending.due(TelemetryGroup::Localization, maximum, defaults));
  assert(!ending.due(TelemetryGroup::Localization, maximum, defaults));
  ending.reset();
  assert(ending.due(TelemetryGroup::Localization, maximum, defaults));
  assert(!ending.due(TelemetryGroup::Localization, maximum, defaults));

  std::cout << "PASS: telemetry rates, absolute cadence, source cap, pause, reset, and overflow\n";
}
