#pragma once

#include "main.h"
#include <vector>

// Public and EZ X/Y share one field frame: at heading 0, FRONT is +Y, RIGHT is +X.
// Heading describes FRONT: 0 = +Y, 90 = +X, 180 = -Y, 270 = -X.
// Public heading equals EZ heading, wrapped to [0,360).
// Pose conversion preserves X/Y, wraps heading, and preserves the
// ANGLE_NOT_SET sentinel for XY-only EZ targets.
ez::pose nav_game_to_internal_pose(ez::pose game_pose);
ez::pose nav_internal_to_game_pose(ez::pose internal_pose);
ez::pose nav_pose_get();  // Entire current pose in the robot frame.
// Heading conversions only wrap to [0,360).
double nav_game_to_internal_heading(double game_theta_deg);
double nav_internal_to_game_heading(double internal_theta_deg);
double nav_theta_get();

void nav_set_start(double x_in, double y_in, double theta_deg);

// Default travel is FRONT-first (EZ fwd); reverse is BACK-first (EZ rev).
// Pose targets also express the final FRONT heading, regardless of travel direction.
void nav_go_to(double x_in, double y_in, int speed);
// One continuous Boomerang motion; waits using the existing odom exits.
void nav_go_to_pose(double x_in, double y_in, double public_heading_deg, int speed);
void nav_go_to_reverse(double x_in, double y_in, int speed);
void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg,
                            int speed);
void nav_turn_to(double theta_deg, int speed);
// Positive inches = FRONT-first; negative inches = BACK-first. Waits for PID.
void nav_drive_distance(double inches, int speed);

struct nav_waypoint {
  double x;
  double y;
};
enum class nav_path_direction { forward, reverse };

// One native injected Pure Pursuit motion, then one existing pid_wait().
// Absolute public X/Y in inches; intermediate points do not require stops.
// Speed: 1..127. False rejects invalid input without starting a motion.
// True means dispatched/wait returned, NOT verified arrival or heading success.
bool nav_follow_path(
    const std::vector<nav_waypoint>& waypoints, int speed,
    nav_path_direction direction = nav_path_direction::forward);
// Same path, with native Boomerang on its last waypoint. No separate turn.
// Final heading is the FRONT heading in degrees; arrival is not guaranteed.
bool nav_follow_path_pose(
    const std::vector<nav_waypoint>& waypoints, double final_public_heading,
    int speed,
    nav_path_direction direction = nav_path_direction::forward);

bool nav_validate_pose(double expected_x, double expected_y,
                       double expected_theta, double xy_tolerance,
                       double theta_tolerance);
// Disable reusable reports and the normal live Brain pose display for competition.
inline constexpr bool NAV_DIAGNOSTICS = true;
// One-shot CURRENT pose snapshots; these do not claim a completed movement.
void nav_show_pose();
void nav_print_pose();
// Call only AFTER a blocking navigation wrapper / pid_wait() / sequence.finish().
// Read-only: no motor commands, resets, or extra pid_wait(). Observe rest for
// 100 ms (all drive motors <3 RPM), with a 2 s timeout, before capturing results.
// Omit target_heading for XY-only moves: no target heading/error is displayed.
// Heading error is signed shortest target-minus-actual, in [-180,180] degrees.
// False = report rejected (invalid values or rest timeout), not navigation success.
// When disabled these are no-ops returning true. Brain results latch until opcontrol.
bool nav_show_result(double target_x, double target_y,
                     double target_heading = ez::ANGLE_NOT_SET);
bool nav_print_result(double target_x, double target_y,
                      double target_heading = ez::ANGLE_NOT_SET);
// Brain and terminal use the same captured pose.
bool nav_debug_result(double target_x, double target_y,
                      double target_heading = ez::ANGLE_NOT_SET);

// Explicit intent API. Requires a NavSequence scope; old diagnostic overloads remain.
enum class NavEndMode { SETTLE, CHAIN };
bool nav_go_to(double x, double y, int speed, NavEndMode end);
bool nav_go_to_reverse(double x, double y, int speed, NavEndMode end);
bool nav_drive_distance(double inches, int speed, NavEndMode end);
bool nav_turn_to(double heading, int speed, NavEndMode end);
// Pose CHAIN requests are rejected; use XY travel for continuation.
bool nav_go_to_pose(double x, double y, double heading, int speed, NavEndMode end);
bool nav_go_to_pose_reverse(double x, double y, double heading, int speed, NavEndMode end);
bool nav_follow_path(const std::vector<nav_waypoint>& points, int speed,
                     NavEndMode end,
                     nav_path_direction direction = nav_path_direction::forward);
bool nav_follow_path_pose(const std::vector<nav_waypoint>& points, double heading,
                          int speed, NavEndMode end,
                          nav_path_direction direction = nav_path_direction::forward);
