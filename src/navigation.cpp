#include "navigation.hpp"
#include "motion_exit_trace.hpp"
#include "path_motion_trace.hpp"

#include <cmath>
#include <cstdio>

namespace {
double wrap_heading(double degrees) {
  const double wrapped = std::fmod(degrees, 360.0);
  return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

// Public and native chassis coordinates share one FRONT-based field frame.
// Normalize heading only; preserve EZ's XY-only target sentinel.
ez::pose normalize_pose_heading(ez::pose pose) {
  return {pose.x, pose.y,
          pose.theta == ez::ANGLE_NOT_SET ? ez::ANGLE_NOT_SET
                                         : wrap_heading(pose.theta)};
}

double angle_error(double target, double current) {
  double error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}
}  // namespace

// ----- Coordinate interface -----
// Route code reads FRONT-based poses here; EZ remains the source of odometry.
double nav_game_to_internal_heading(double game_theta_deg) {
  return wrap_heading(game_theta_deg);
}

double nav_internal_to_game_heading(double internal_theta_deg) {
  return wrap_heading(internal_theta_deg);
}

ez::pose nav_game_to_internal_pose(ez::pose game_pose) {
  return normalize_pose_heading(game_pose);
}

ez::pose nav_internal_to_game_pose(ez::pose internal_pose) {
  return normalize_pose_heading(internal_pose);
}

ez::pose nav_pose_get() {
  return nav_internal_to_game_pose(chassis.odom_pose_get());
}

double nav_theta_get() {
  return nav_internal_to_game_heading(chassis.odom_theta_get());
}

void nav_set_start(double x_in, double y_in, double theta_deg) {
  // Set the software reference after physically aligning the robot at the start.
  // This changes the reported pose; it does not drive the robot into position.
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

// ----- Blocking navigation wrappers -----
// Send the requested movement to EZ and wait for its exit conditions.
// Returning from a wait alone does not prove that the physical target was reached.
void nav_go_to(double x_in, double y_in, int speed) {
  const ez::odom movement{nav_game_to_internal_pose({x_in, y_in}), ez::fwd, speed};
  motion_exit_trace_begin(movement);
  chassis.pid_odom_set(movement);
  chassis.pid_wait();
  motion_exit_trace_finish();  // No-op unless separately armed.
}

void nav_go_to_pose(double x_in, double y_in, double public_heading_deg, int speed) {
  const auto target = nav_game_to_internal_pose({x_in, y_in, public_heading_deg});
  const ez::odom movement{target, ez::fwd, speed};  // FRONT-first, as in nav_go_to.
  const bool interfered_before = chassis.interfered;
  std::printf("\nPOSE requested PUBLIC = (%.3f, %.3f, %.3f deg)\n"
              "POSE converted EZ = (%.3f, %.3f, %.3f deg) | EZ fwd | speed %d\n",
              x_in, y_in, public_heading_deg, target.x, target.y, target.theta, speed);
  std::fflush(stdout);
  pose_exit_trace_begin();
  // Explicit single-pose API; uses the existing global slew/Boomerang settings.
  chassis.pid_odom_boomerang_set(movement);
  chassis.pid_wait();
  const auto final_pose = nav_pose_get();
  const auto exits = pose_exit_trace_finish();
  std::printf("POSE final PUBLIC = (%.3f, %.3f, %.3f deg) [at pid_wait return]\n"
              "POSE position error = %.3f in | heading error (target-current) = %+.3f deg\n"
              "POSE exit XY = %s | angular = %s | interference before/after = %s/%s\n",
              final_pose.x, final_pose.y, final_pose.theta,
              std::hypot(x_in - final_pose.x, y_in - final_pose.y),
              angle_error(public_heading_deg, final_pose.theta),
              ez::exit_to_string(exits.first).c_str(),
              ez::exit_to_string(exits.second).c_str(),
              interfered_before ? "true" : "false", chassis.interfered ? "true" : "false");
  std::fflush(stdout);
}

void nav_go_to_reverse(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({nav_game_to_internal_pose({x_in, y_in}), ez::rev, speed});
  chassis.pid_wait();
}

void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg,
                            int speed) {
  chassis.pid_odom_set(
      {nav_game_to_internal_pose({x_in, y_in, theta_deg}), ez::rev, speed});
  chassis.pid_wait();
}

void nav_turn_to(double theta_deg, int speed) {
  turn_velocity_trace_begin();
  const double target = nav_game_to_internal_heading(theta_deg);
  final_turn_trace_target(target);
  chassis.pid_turn_set(target, speed);
  chassis.pid_wait();
  final_turn_trace_wait_returned();
}

void nav_drive_distance(double inches, int speed) {
  // Positive distance moves FRONT-first in the native chassis frame.
  chassis.pid_drive_set(inches, speed);
  chassis.pid_wait();
}

namespace {
bool follow_path(const std::vector<nav_waypoint>& waypoints, int speed,
                 nav_path_direction direction, bool use_final_heading,
                 double final_public_heading) {
  // Build one native path from the whole point list, rather than stopping at each point.
  if (waypoints.empty() || speed < 1 || speed > 127 ||
      (direction != nav_path_direction::forward &&
       direction != nav_path_direction::reverse) ||
      (use_final_heading && !std::isfinite(final_public_heading))) {
    return false;
  }
  // Validate the complete request before touching the controller.
  for (const auto& point : waypoints) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
  }

  const auto ez_direction = direction == nav_path_direction::forward
                                ? ez::fwd : ez::rev;
  std::vector<ez::odom> movements;
  movements.reserve(waypoints.size());
  for (const auto& point : waypoints) {
    movements.push_back(
        {nav_game_to_internal_pose({point.x, point.y}), ez_direction, speed});
  }
  if (use_final_heading) {
    movements.back().target.theta =
        nav_game_to_internal_heading(final_public_heading);
  }
  // Native injection + lookahead following; no custom path algorithm, per-point
  // waits, tuning changes, or post-path turn. Global slew settings are retained.
  path_motion_trace_begin(movements);  // No-op unless explicitly armed.
  chassis.pid_odom_injected_pp_set(movements);
  chassis.pid_wait();  // Retains the project's existing final-waypoint wait fix.
  path_motion_trace_wait_returned();   // Capture only; never stops or prints.
  return true;
}
}  // namespace

bool nav_follow_path(const std::vector<nav_waypoint>& waypoints, int speed,
                     nav_path_direction direction) {
  return follow_path(waypoints, speed, direction, false, 0.0);
}

bool nav_follow_path_pose(const std::vector<nav_waypoint>& waypoints,
                          double final_public_heading, int speed,
                          nav_path_direction direction) {
  return follow_path(waypoints, speed, direction, true, final_public_heading);
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

#include "nav_sequence.hpp"
// ----- Explicit CHAIN / SETTLE interface -----
// These overloads hand control to NavSequence for timing, events and cleanup.
// XY sequence moves use injected paths, unlike the basic wrapper above.
namespace {
bool valid_motion(double x, double y, double h, int speed, NavEndMode end) {
  if (!nav_ok()) return false;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(h) || speed < 1 || speed > 127 ||
      (end != NavEndMode::SETTLE && end != NavEndMode::CHAIN)) {
    nav_abort("invalid movement"); return false;
  }
  return true;
}
bool sequence_path(const std::vector<nav_waypoint>& points, double h, int speed,
                   NavEndMode end, nav_path_direction direction, bool pose) {
  if (points.empty() || (direction != nav_path_direction::forward && direction != nav_path_direction::reverse)) {
    nav_abort("invalid path"); return false;
  }
  if (pose && end == NavEndMode::CHAIN) { nav_abort("pose motion requires SETTLE"); return false; }
  std::vector<ez::odom> movements;
  for (const auto& p : points) {
    if (!valid_motion(p.x,p.y,h,speed,end)) return false;
    movements.push_back({nav_game_to_internal_pose({p.x,p.y}),
      direction == nav_path_direction::forward ? ez::fwd : ez::rev,speed});
  }
  if (pose) movements.back().target.theta = nav_game_to_internal_heading(h);
  const ez::pose final{points.back().x,points.back().y,pose ? h : ez::ANGLE_NOT_SET};
  return nav_detail::move([&]{ chassis.pid_odom_injected_pp_set(movements); },end,final);
}
}
bool nav_follow_path(const std::vector<nav_waypoint>& points, int speed, NavEndMode end, nav_path_direction direction) {
  return sequence_path(points,0,speed,end,direction,false);
}
bool nav_follow_path_pose(const std::vector<nav_waypoint>& points, double h, int speed, NavEndMode end, nav_path_direction direction) {
  return sequence_path(points,h,speed,end,direction,true);
}
bool nav_go_to(double x, double y, int speed, NavEndMode end) {
  return nav_follow_path({{x,y}},speed,end);
}
bool nav_go_to_reverse(double x, double y, int speed, NavEndMode end) {
  return nav_follow_path({{x,y}},speed,end,nav_path_direction::reverse);
}
bool nav_go_to_pose(double x, double y, double h, int speed, NavEndMode end) {
  if (!valid_motion(x,y,h,speed,end)) return false;
  if (end != NavEndMode::SETTLE) { nav_abort("pose motion requires SETTLE"); return false; }
  return nav_detail::move([&]{ chassis.pid_odom_boomerang_set({nav_game_to_internal_pose({x,y,h}),ez::fwd,speed}); },end,{x,y,h});
}
bool nav_go_to_pose_reverse(double x, double y, double h, int speed, NavEndMode end) {
  if (!valid_motion(x,y,h,speed,end)) return false;
  if (end != NavEndMode::SETTLE) { nav_abort("pose motion requires SETTLE"); return false; }
  return nav_detail::move([&]{ chassis.pid_odom_boomerang_set({nav_game_to_internal_pose({x,y,h}),ez::rev,speed}); },end,{x,y,h});
}
bool nav_turn_to(double h, int speed, NavEndMode end) {
  if (!valid_motion(0,0,h,speed,end)) return false;
  return nav_detail::move([&]{ chassis.pid_turn_set(nav_game_to_internal_heading(h),speed); },end,{0,0,h},0,false,true);
}
bool nav_drive_distance(double inches, int speed, NavEndMode end) {
  if (!valid_motion(inches,0,0,speed,end)) return false;
  return nav_detail::move([&]{ chassis.pid_drive_set(inches,speed); },end,{0,0},inches,true,false);
}
