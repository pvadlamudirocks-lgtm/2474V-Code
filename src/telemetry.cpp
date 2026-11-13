#include "telemetry.hpp"
#include "telemetry_format.hpp"
#include "anti_tip.hpp"
#include "lift_height.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "subsystems.hpp"
#include <cstdio>

void telemetry_reset() {
  lift_height_telemetry_reset();
}
void telemetry_update(pros::Controller&) {
  // Preserve sensor sampling without writing/clearing the competition display.
  lift_height_telemetry_update();
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
