#include "telemetry.hpp"
#include "telemetry_format.hpp"
#include "anti_tip.hpp"
#include "lift_height.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "subsystems.hpp"
#include <cstdio>

namespace {
unsigned captured_at = 0, written_at = 0, next_row = 0;
bool captured = false, written = false;
std::array<telemetry_format::Row, 3> rows{};
}
void telemetry_reset() {
  captured = written = false; next_row = 0;
  lift_height_telemetry_reset();
}
void telemetry_update(pros::Controller& controller) {
  lift_height_telemetry_update();
  const unsigned now = pros::millis();
  if (!captured || now - captured_at >= 100) {
    captured = true; captured_at = now;
    const auto pose = nav_pose_get();
    const bool pose_valid = nav_sensors_ready();
    rows = telemetry_format::dashboard(pose_valid ? pose.theta : NAN,
        pose_valid ? pose.x : NAN, pose_valid ? pose.y : NAN, lift_position_deg());
  }
  // Installed PROS misc.h: wireless writes faster than 50 ms may be discarded.
  // One row per 50 ms gives all three rows a reliable ~150 ms refresh.
  if (written && now - written_at < 50) return;
  written = true; written_at = now;
  if (controller.print(next_row, 0, "%s", rows[next_row].data()) != PROS_ERR)
    next_row = (next_row + 1) % rows.size();
}
void print_robot_telemetry() {
  lift_height_telemetry_update();
  const auto pose = nav_pose_get();
  const bool pose_valid = nav_sensors_ready();
  const auto tilt = anti_tip::snapshot();
  std::printf("2474V TELEMETRY\nPose X=%.3f Y=%.3f in H=%.3f deg valid=%d\n"
              "IMU pitch=%.3f roll=%.3f deg valid=%d\nLIFT POSITION: %.1f deg\n",
              pose_valid ? pose.x : NAN, pose_valid ? pose.y : NAN,
              pose_valid ? pose.theta : NAN, pose_valid,
              tilt.orientation.pitch, tilt.orientation.roll, tilt.orientation.valid,
              lift_position_deg());
  anti_tip::print_snapshot();
}
void telemetry_monitor_test() {
  // Existing cancellation stops EZ before this no-motion test starts.
  // Never requests nonzero output or dispatches a route.
  nav_cancel_sequence();
  stop_all_mechanisms();
  telemetry_reset();
  std::printf("MONITOR ONLY: supported robot, controlled tilts; no recovery motor output.\n");
  const unsigned began = pros::millis();
  unsigned printed_at = began - 500;
  while (pros::millis() - began < 30000 && !pros::competition::is_disabled() &&
         pros::competition::is_autonomous()) {
    lift_height_telemetry_update();
    if (pros::millis() - printed_at >= 500) {
      printed_at = pros::millis(); print_robot_telemetry();
    }
    pros::delay(20);
  }
}
