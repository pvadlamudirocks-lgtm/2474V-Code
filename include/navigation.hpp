#pragma once

#include "main.h"
#include <vector>

// Public and EZ X/Y share fixed field axes: +Y forward, +X right.
// Public heading describes the CLAW: 0 = +Y, 90 = +X, 180 = -Y, 270 = -X.
// EZ heading describes the OLD front: game 0 -> EZ 180, game 90 -> EZ 270.
// Pose conversion preserves X/Y, shifts heading 180 degrees, and preserves the
// ANGLE_NOT_SET sentinel for XY-only EZ targets.
ez::pose nav_game_to_internal_pose(ez::pose game_pose);
ez::pose nav_internal_to_game_pose(ez::pose internal_pose);
ez::pose nav_pose_get();  // Entire current pose in the claw/game frame.
// Heading conversions apply the front offset and wrap to [0,360).
double nav_game_to_internal_heading(double game_theta_deg);
double nav_internal_to_game_heading(double internal_theta_deg);
double nav_theta_get();

void nav_set_start(double x_in, double y_in, double theta_deg);

// Default travel is claw-leading (EZ rev); reverse is old-front-leading (EZ fwd).
// Pose targets also express the final CLAW heading, regardless of travel direction.
void nav_go_to(double x_in, double y_in, int speed);
// One continuous Boomerang motion; waits using the existing odom exits.
void nav_go_to_pose(double x_in, double y_in, double public_heading_deg, int speed);
void nav_go_to_reverse(double x_in, double y_in, int speed);
void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg,
                            int speed);
void nav_turn_to(double theta_deg, int speed);
// Positive inches = claw-first; negative inches = old-front-first. Waits for PID.
void nav_drive_distance(double inches, int speed);

struct nav_waypoint {
  double x;
  double y;
};
enum class nav_path_direction { claw_forward, claw_reverse };

// One native injected Pure Pursuit motion, then one existing pid_wait().
// Absolute public X/Y in inches; intermediate points do not require stops.
// Speed: 1..127. False rejects invalid input without starting a motion.
// True means dispatched/wait returned, NOT verified arrival or heading success.
bool nav_follow_path(
    const std::vector<nav_waypoint>& waypoints, int speed,
    nav_path_direction direction = nav_path_direction::claw_forward);
// Same path, with native Boomerang on its last waypoint. No separate turn.
// Final heading is the public CLAW heading in degrees; arrival is not guaranteed.
bool nav_follow_path_pose(
    const std::vector<nav_waypoint>& waypoints, double final_public_heading,
    int speed,
    nav_path_direction direction = nav_path_direction::claw_forward);

bool nav_validate_pose(double expected_x, double expected_y,
                       double expected_theta, double xy_tolerance,
                       double theta_tolerance);
void nav_print_pose();

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
                     nav_path_direction direction = nav_path_direction::claw_forward);
bool nav_follow_path_pose(const std::vector<nav_waypoint>& points, double heading,
                          int speed, NavEndMode end,
                          nav_path_direction direction = nav_path_direction::claw_forward);
