#pragma once

#include "main.h"

// Absolute coordinate convention:
// +Y = forward from start, -Y = backward from start.
// +X = right from start, -X = left from start.
// 0 deg = +Y, 90 deg = +X, 180 deg = -Y, 270/-90 deg = -X.
// Target coordinates represent the robot center, not game object centers.

// Sets the robot's trusted absolute starting pose.
// Use this at the beginning of autonomous, or later only after a trusted
// physical/sensor correction. Do not call it after every movement.
void nav_set_start(double x_in, double y_in, double theta_deg);

// Drives forward to an absolute X/Y coordinate and waits until the motion ends.
// The target is measured from the original start point, not from the robot's
// current position.
void nav_go_to(double x_in, double y_in, int speed);

// Drives forward to an absolute X/Y coordinate while also aiming for a final
// absolute heading at the target pose.
void nav_go_to_pose(double x_in, double y_in, double theta_deg, int speed);

// Drives backward to an absolute X/Y coordinate and waits until the motion ends.
// Use this when the robot should back into a point without redefining the map.
void nav_go_to_reverse(double x_in, double y_in, int speed);

// Drives backward to an absolute X/Y coordinate while also aiming for a final
// absolute heading at the target pose.
void nav_go_to_pose_reverse(double x_in, double y_in, double theta_deg, int speed);

// Turns to an absolute heading while leaving the current X/Y odom position alone.
// Example: 90 means face +X/right from the starting direction.
void nav_turn_to(double theta_deg, int speed);

// Checks whether the current odom pose is close enough to an expected absolute
// pose. xy_tolerance is in inches; theta_tolerance is in degrees.
bool nav_validate_pose(double expected_x, double expected_y, double expected_theta,
                       double xy_tolerance, double theta_tolerance);

// Prints the current odom X, Y, and heading to the terminal and brain screen.
// Use this after test movements to confirm that coordinates did not reset.
void nav_print_pose();

// Optional distance wrapper: corrects final front distance without replacing odom.
bool nav_correct_front_distance(double target_distance_in,
                                double tolerance_in,
                                int max_speed,
                                int timeout_ms);

// Optional distance wrapper: corrects final back distance for scoring-side approach.
bool nav_correct_back_distance(double target_distance_in,
                               double tolerance_in,
                               int max_speed,
                               int timeout_ms);

// Optional distance wrapper: squares to a flat front surface.
bool nav_square_to_front_surface(double tolerance_in,
                                 int max_turn_speed,
                                 int timeout_ms);

// Optional distance wrapper: squares to a flat back surface for scoring alignment.
bool nav_square_to_back_surface(double tolerance_in,
                                int max_turn_speed,
                                int timeout_ms);

// Optional distance wrapper: corrects only odom Y from a known front reference.
bool nav_reset_y_with_front_distance(double known_reference_y_in,
                                     double robot_front_offset_in,
                                     bool robot_facing_positive_y);

// Optional distance wrapper: corrects only odom X from a known front reference.
bool nav_reset_x_with_front_distance(double known_reference_x_in,
                                     double robot_front_offset_in,
                                     bool robot_facing_positive_x);

// Optional distance wrappers: correct one axis using back or side sensors.
bool nav_reset_y_with_back_distance(double known_reference_y_in,
                                    double robot_back_offset_in,
                                    bool robot_back_facing_positive_y);
bool nav_reset_x_with_back_distance(double known_reference_x_in,
                                    double robot_back_offset_in,
                                    bool robot_back_facing_positive_x);
bool nav_reset_x_with_right_distance(double known_reference_x_in,
                                     double robot_right_offset_in,
                                     bool right_sensor_facing_positive_x);
bool nav_reset_x_with_left_distance(double known_reference_x_in,
                                    double robot_left_offset_in,
                                    bool left_sensor_facing_positive_x);

// Chained helpers are for fast travel only. Do not chain before scoring,
// distance correction, or pose resets.
void nav_chain_to(double x_in, double y_in, int speed);
void nav_chain_to_reverse(double x_in, double y_in, int speed);

// Speed presets for common absolute odom movements.
void nav_fast_to(double x_in, double y_in);
void nav_fast_to_reverse(double x_in, double y_in);
void nav_controlled_to(double x_in, double y_in);
void nav_controlled_to_reverse(double x_in, double y_in);
void nav_precise_to_pose(double x_in, double y_in, double theta_deg);
void nav_precise_to_pose_reverse(double x_in, double y_in, double theta_deg);
