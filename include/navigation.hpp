#pragma once

#include "main.h"

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
void nav_go_to_pose(double x_in, double y_in, double theta_deg, int speed);
void nav_go_to_reverse(double x_in, double y_in, int speed);
void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg,
                            int speed);
void nav_turn_to(double theta_deg, int speed);
// Positive inches = claw-first; negative inches = old-front-first. Waits for PID.
void nav_drive_distance(double inches, int speed);

bool nav_validate_pose(double expected_x, double expected_y,
                       double expected_theta, double xy_tolerance,
                       double theta_tolerance);
void nav_print_pose();
