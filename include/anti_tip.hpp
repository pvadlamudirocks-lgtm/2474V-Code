#pragma once
#include "robot_imu.hpp"

namespace anti_tip {
enum class Mode { DISABLED, MONITOR_ONLY };
namespace config {
inline constexpr Mode mode = Mode::MONITOR_ONLY;
// All angles, signed gains, rate horizon and output limit are UNCALIBRATED/NAN.
// Populate only from staged physical testing; no competition defaults are guessed.
inline constexpr stability::Settings settings{};
// Future low-power takeover requires these gates AND a reviewed owner handoff.
inline constexpr bool ENABLE_LOW_POWER_TEST = false;
inline constexpr double LOW_POWER_OUTPUT_LIMIT = stability::invalid;
inline constexpr bool DRIVER_RECOVERY_VALIDATED = false;
inline constexpr bool AUTON_TAKEOVER_IMPLEMENTED = false;
}
struct Snapshot {
  unsigned timestamp_ms = 0;
  RobotOrientation orientation;
  double pitch_rate = stability::invalid, roll_rate = stability::invalid;
  double proposed_correction = 0;
  stability::State state = stability::State::DISABLED;
  const char* reason = "NO SAMPLE";
};
// Read-only sampler, serviced by the existing display task (50 ms), even disabled.
// No motor writes, navigation cancellation, extra task, or automatic serial output.
void update();
Snapshot snapshot(); // Thread-safe copy, stale samples invalidate after 200 ms.
const char* state_name(stability::State state);
void print_snapshot(); // On demand; no motor command.
// Future integration contract: only the current drivetrain owner may apply a
// stability::propose_driver_command result. No active application is implemented.
// Autonomous must cancel under navigation_lock, confirm DISABLE and stop,
// then latch route abort before a recovery owner can ever start. Never auto-resume.
}
