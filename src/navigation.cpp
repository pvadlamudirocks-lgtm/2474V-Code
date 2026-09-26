#include "navigation.hpp"
#include "motion_exit_trace.hpp"

#include <cmath>
#include <cstdio>

namespace {
double wrap_heading(double degrees) {
  const double wrapped = std::fmod(degrees, 360.0);
  return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// Field axes stay fixed; only the designated robot front changes.
// A half-turn is its own inverse. Preserve EZ's XY-only target sentinel.
ez::pose transform_frame(ez::pose pose) {
  return {pose.x, pose.y,
          pose.theta == ez::ANGLE_NOT_SET ? ez::ANGLE_NOT_SET
                                         : wrap_heading(pose.theta + 180.0)};
}

double angle_error(double target, double current) {
  double error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}
}  // namespace

double nav_game_to_internal_heading(double game_theta_deg) {
  return wrap_heading(game_theta_deg + 180.0);
}

double nav_internal_to_game_heading(double internal_theta_deg) {
  return wrap_heading(internal_theta_deg - 180.0);
}

ez::pose nav_game_to_internal_pose(ez::pose game_pose) {
  return transform_frame(game_pose);
}

ez::pose nav_internal_to_game_pose(ez::pose internal_pose) {
  return transform_frame(internal_pose);
}

ez::pose nav_pose_get() {
  return nav_internal_to_game_pose(chassis.odom_pose_get());
}

double nav_theta_get() {
  return nav_internal_to_game_heading(chassis.odom_theta_get());
}

void nav_set_start(double x_in, double y_in, double theta_deg) {
  const auto pose = nav_game_to_internal_pose({x_in, y_in, theta_deg});
  chassis.odom_xyt_set(pose.x, pose.y, pose.theta);
  // EZ 3.2.2 sets the IMU here, but odom_current.theta is only refreshed
  // by ez_tracking_task(). Yield before any caller starts a movement.
  const auto start_ms = pros::millis();
  ez::pose actual;
  double imu_heading;
  bool heading_ready;
  do {
    pros::delay(10);
    actual = chassis.odom_pose_get();
    imu_heading = chassis.drive_imu_get();
    heading_ready = std::isfinite(actual.theta) && std::isfinite(imu_heading) &&
        std::fabs(angle_error(pose.theta, actual.theta)) <= 0.5 &&
        std::fabs(angle_error(pose.theta, imu_heading)) <= 0.5;
  } while (!heading_ready && pros::millis() - start_ms < 250);
  // Startup readback is printed by the no-motion autonomous snapshot test.
}

void nav_go_to(double x_in, double y_in, int speed) {
  const ez::odom movement{nav_game_to_internal_pose({x_in, y_in}), ez::rev, speed};
  motion_exit_trace_begin(movement);
  std::printf("=== BEFORE PID_ODOM_SET ===\n");
  std::fflush(stdout);
  motion_controller_snapshot_start();
  chassis.pid_odom_set(movement);
  std::printf("=== AFTER PID_ODOM_SET ===\n");
  std::fflush(stdout);
  std::printf("=== ENTER PID_WAIT ===\n");
  std::fflush(stdout);
  chassis.pid_wait();
  std::printf("=== PID_WAIT RETURNED ===\n");
  std::fflush(stdout);
  motion_exit_trace_finish();  // Captures before stopping; no-op unless armed.
}

void nav_go_to_pose(double x_in, double y_in, double theta_deg, int speed) {
  chassis.pid_odom_set(
      {nav_game_to_internal_pose({x_in, y_in, theta_deg}), ez::rev, speed});
  chassis.pid_wait();
}

void nav_go_to_reverse(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({nav_game_to_internal_pose({x_in, y_in}), ez::fwd, speed});
  chassis.pid_wait();
}

void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg,
                            int speed) {
  chassis.pid_odom_set(
      {nav_game_to_internal_pose({x_in, y_in, theta_deg}), ez::fwd, speed});
  chassis.pid_wait();
}

void nav_turn_to(double theta_deg, int speed) {
  turn_velocity_trace_begin();
  chassis.pid_turn_set(nav_game_to_internal_heading(theta_deg), speed);
  chassis.pid_wait();
}

void nav_drive_distance(double inches, int speed) {
  // EZ positive distance leads with the old front; game positive is claw-first.
  chassis.pid_drive_set(-inches, speed);
  chassis.pid_wait();
}

bool nav_validate_pose(double expected_x, double expected_y,
                       double expected_theta, double xy_tolerance,
                       double theta_tolerance) {
  const auto pose = nav_pose_get();
  const double x_error = std::fabs(pose.x - expected_x);
  const double y_error = std::fabs(pose.y - expected_y);
  const double theta_error =
      std::fabs(angle_error(expected_theta, pose.theta));

  return x_error <= xy_tolerance && y_error <= xy_tolerance &&
         theta_error <= theta_tolerance;
}

void nav_print_pose() {
  const auto pose = nav_pose_get();
  std::printf("Claw/game pose | x: %.2f in, y: %.2f in, claw/game theta: %.2f deg\n",
              pose.x, pose.y, pose.theta);
}
