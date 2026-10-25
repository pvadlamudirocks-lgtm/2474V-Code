#include "anti_tip.hpp"
#include "api.h"
#include <cstdio>

namespace anti_tip {
namespace {
pros::Mutex snapshot_mutex;
Snapshot latest;
stability::RateFilter pitch_filter, roll_filter;
bool sampled = false;
unsigned last_sample = 0;
}
const char* state_name(stability::State state) {
  switch (state) {
    case stability::State::DISABLED: return "DISABLED";
    case stability::State::STABLE: return "STABLE";
    case stability::State::WARNING: return "WARNING";
    case stability::State::RECOVERY: return "RECOVERY";
    case stability::State::SEVERE: return "SEVERE";
  }
  return "INVALID";
}
void update() {
  snapshot_mutex.take();
  const unsigned now = pros::millis();
  if (sampled && now - last_sample < 20) { snapshot_mutex.give(); return; }
  const double dt = sampled ? (now - last_sample) / 1000.0 : .05;
  auto previous = latest.state;
  if (!sampled || now - last_sample > 200) {
    pitch_filter.reset(); roll_filter.reset(); previous = stability::State::DISABLED;
  }
  sampled = true; last_sample = now;
  latest = Snapshot{};
  latest.timestamp_ms = now;
  latest.orientation = robot_orientation();
  if (!latest.orientation.valid) {
    pitch_filter.reset(); roll_filter.reset();
    latest.reason = "IMU UNAVAILABLE/CALIBRATING/INVALID";
  } else {
    latest.pitch_rate = pitch_filter.update(latest.orientation.pitch, dt);
    latest.roll_rate = roll_filter.update(latest.orientation.roll, dt);
    const stability::Input input{latest.orientation.pitch, latest.orientation.roll,
                                 latest.pitch_rate, latest.roll_rate};
    if (config::mode == Mode::DISABLED) latest.reason = "MONITOR DISABLED";
    else if (!stability::valid(input)) latest.reason = "RATE WARMUP";
    else if (!robot_imu_config::AXES_VERIFIED) latest.reason = "AXES UNVERIFIED";
    else if (!stability::valid(config::settings)) latest.reason = "THRESHOLDS UNCALIBRATED";
    else {
      latest.state = stability::classify(previous, input, config::settings);
      latest.proposed_correction = stability::correction(latest.state, input, config::settings);
      latest.reason = !stability::gains_valid(config::settings) ? "GAINS UNCALIBRATED" : "MONITOR ONLY";
    }
  }
  snapshot_mutex.give();
}
Snapshot snapshot() {
  snapshot_mutex.take();
  auto result = latest;
  if (!sampled || pros::millis() - result.timestamp_ms > 200) {
    result = Snapshot{}; result.reason = "STALE/NO SAMPLE";
  }
  snapshot_mutex.give();
  return result;
}
void print_snapshot() {
  const auto s = snapshot();
  std::printf("ANTI-TIP MONITOR t=%u pitch=%.3f roll=%.3f pitch_rate=%.3f roll_rate=%.3f\n"
              "state=%s proposed=%.3f applied=0 reason=%s axes_verified=%d\n",
              s.timestamp_ms, s.orientation.pitch, s.orientation.roll, s.pitch_rate,
              s.roll_rate, state_name(s.state), s.proposed_correction, s.reason,
              robot_imu_config::AXES_VERIFIED);
}
}
