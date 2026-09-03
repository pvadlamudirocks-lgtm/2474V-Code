#include "navigation.hpp"

#include "distance_correction.hpp"

#include <cmath>
#include <cstdio>

namespace {
constexpr int NAV_FAST_SPEED = 110;
constexpr int NAV_CONTROLLED_SPEED = 80;
constexpr int NAV_PRECISE_SPEED = 50;

// Returns the shortest signed difference between two headings in degrees.
// This lets pose validation treat 270 and -90 as the same direction.
double angle_error(double target, double current) {
  double error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}
}  // namespace

// Sets the absolute odom pose at auton start or after trusted correction.
// This is the only navigation helper that intentionally resets odometry.
void nav_set_start(double x_in, double y_in, double theta_deg) {
  // Sets the fixed absolute coordinate origin for autonomous. Do not call this
  // after each movement; only call it at auton start or after trusted correction.
  chassis.odom_xyt_set(x_in, y_in, theta_deg);
}

// Drives forward to an absolute point using EZ Template odometry.
// x_in and y_in are fixed field coordinates from the starting origin.
void nav_go_to(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({{x_in, y_in}, fwd, speed});
  chassis.pid_wait();
}

// Drives forward to an absolute point and final heading.
// This is useful when the robot must arrive lined up for a later action.
void nav_go_to_pose(double x_in, double y_in, double theta_deg, int speed) {
  chassis.pid_odom_set({{x_in, y_in, theta_deg}, fwd, speed});
  chassis.pid_wait();
}

// Drives backward to an absolute point using EZ Template odometry.
// The target stays absolute even though the robot travels in reverse.
void nav_go_to_reverse(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({{x_in, y_in}, rev, speed});
  chassis.pid_wait();
}

// Drives backward to an absolute point and final heading.
// Use this when the scoring/intake side needs to stay oriented while backing up.
void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg, int speed) {
  chassis.pid_odom_set({{x_in, y_in, theta_deg}, rev, speed});
  chassis.pid_wait();
}

// Turns to an absolute heading without changing the stored X/Y origin.
// Heading follows the project convention: 0 = +Y, 90 = +X.
void nav_turn_to(double theta_deg, int speed) {
  chassis.pid_turn_set(theta_deg, speed);
  chassis.pid_wait();
}

// Verifies that the current odom pose is close enough to an expected pose.
// Returns true only if X, Y, and heading are all inside their tolerances.
bool nav_validate_pose(double expected_x, double expected_y, double expected_theta,
                       double xy_tolerance, double theta_tolerance) {
  const double x_error = std::fabs(chassis.odom_x_get() - expected_x);
  const double y_error = std::fabs(chassis.odom_y_get() - expected_y);
  const double theta_error = std::fabs(angle_error(expected_theta, chassis.odom_theta_get()));

  return x_error <= xy_tolerance && y_error <= xy_tolerance && theta_error <= theta_tolerance;
}

// Reports current odom X, Y, and heading for autonomous debugging.
// This is intended for test autons before adding final match routes.
void nav_print_pose() {
  const double x = chassis.odom_x_get();
  const double y = chassis.odom_y_get();
  const double theta = chassis.odom_theta_get();

  std::printf("Odom pose | x: %.2f in, y: %.2f in, theta: %.2f deg\n", x, y, theta);
  ez::screen_print("x: " + util::to_string_with_precision(x) +
                       "\ny: " + util::to_string_with_precision(y) +
                       "\nt: " + util::to_string_with_precision(theta),
                   1);
}

// Calls the distance layer to trim final front distance after odom movement.
bool nav_correct_front_distance(double target_distance_in,
                                double tolerance_in,
                                int max_speed,
                                int timeout_ms) {
  return dist_correct_front_distance(target_distance_in, tolerance_in, max_speed, timeout_ms);
}

// Calls the distance layer to trim final back distance for scoring alignment.
bool nav_correct_back_distance(double target_distance_in,
                               double tolerance_in,
                               int max_speed,
                               int timeout_ms) {
  return dist_correct_back_distance(target_distance_in, tolerance_in, max_speed, timeout_ms);
}

// Calls the distance layer to square to a flat surface seen by the front sensors.
bool nav_square_to_front_surface(double tolerance_in,
                                 int max_turn_speed,
                                 int timeout_ms) {
  return dist_square_to_front_surface(tolerance_in, max_turn_speed, timeout_ms);
}

// Calls the distance layer to square to a flat surface seen by the back sensors.
bool nav_square_to_back_surface(double tolerance_in,
                                int max_turn_speed,
                                int timeout_ms) {
  return dist_square_to_back_surface(tolerance_in, max_turn_speed, timeout_ms);
}

// Calls the distance layer to correct only the Y coordinate from a known reference.
bool nav_reset_y_with_front_distance(double known_reference_y_in,
                                     double robot_front_offset_in,
                                     bool robot_facing_positive_y) {
  return dist_reset_y_from_front_reference(known_reference_y_in,
                                          robot_front_offset_in,
                                          robot_facing_positive_y);
}

// Calls the distance layer to correct only the X coordinate from a known reference.
bool nav_reset_x_with_front_distance(double known_reference_x_in,
                                     double robot_front_offset_in,
                                     bool robot_facing_positive_x) {
  return dist_reset_x_from_front_reference(known_reference_x_in,
                                          robot_front_offset_in,
                                          robot_facing_positive_x);
}

// Calls the distance layer to correct only the Y coordinate from a back reference.
bool nav_reset_y_with_back_distance(double known_reference_y_in,
                                    double robot_back_offset_in,
                                    bool robot_back_facing_positive_y) {
  return dist_reset_y_from_back_reference(known_reference_y_in,
                                         robot_back_offset_in,
                                         robot_back_facing_positive_y);
}

// Calls the distance layer to correct only the X coordinate from a back reference.
bool nav_reset_x_with_back_distance(double known_reference_x_in,
                                    double robot_back_offset_in,
                                    bool robot_back_facing_positive_x) {
  return dist_reset_x_from_back_reference(known_reference_x_in,
                                         robot_back_offset_in,
                                         robot_back_facing_positive_x);
}

// Calls the distance layer to correct only X from the right sensor.
bool nav_reset_x_with_right_distance(double known_reference_x_in,
                                     double robot_right_offset_in,
                                     bool right_sensor_facing_positive_x) {
  return dist_reset_x_from_right_reference(known_reference_x_in,
                                          robot_right_offset_in,
                                          right_sensor_facing_positive_x);
}

// Calls the distance layer to correct only X from the left sensor.
bool nav_reset_x_with_left_distance(double known_reference_x_in,
                                    double robot_left_offset_in,
                                    bool left_sensor_facing_positive_x) {
  return dist_reset_x_from_left_reference(known_reference_x_in,
                                         robot_left_offset_in,
                                         left_sensor_facing_positive_x);
}

// Starts a forward absolute odom movement and exits early for chaining.
void nav_chain_to(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({{x_in, y_in}, fwd, speed});
  chassis.pid_wait_quick_chain();
}

// Starts a reverse absolute odom movement and exits early for chaining.
void nav_chain_to_reverse(double x_in, double y_in, int speed) {
  chassis.pid_odom_set({{x_in, y_in}, rev, speed});
  chassis.pid_wait_quick_chain();
}

// Fast forward travel to an absolute point.
void nav_fast_to(double x_in, double y_in) {
  nav_go_to(x_in, y_in, NAV_FAST_SPEED);
}

// Fast reverse travel to an absolute point.
void nav_fast_to_reverse(double x_in, double y_in) {
  nav_go_to_reverse(x_in, y_in, NAV_FAST_SPEED);
}

// Medium-speed forward travel to an absolute point.
void nav_controlled_to(double x_in, double y_in) {
  nav_go_to(x_in, y_in, NAV_CONTROLLED_SPEED);
}

// Medium-speed reverse travel to an absolute point.
void nav_controlled_to_reverse(double x_in, double y_in) {
  nav_go_to_reverse(x_in, y_in, NAV_CONTROLLED_SPEED);
}

// Low-speed forward move to a precise absolute pose.
void nav_precise_to_pose(double x_in, double y_in, double theta_deg) {
  nav_go_to_pose(x_in, y_in, theta_deg, NAV_PRECISE_SPEED);
}

// Low-speed reverse move to a precise absolute pose.
void nav_precise_to_pose_reverse(double x_in, double y_in, double theta_deg) {
  nav_go_to_pose_reverse(x_in, y_in, theta_deg, NAV_PRECISE_SPEED);
}
