#pragma once

// Sets drive, turn, swing, and odom PID constants used by test routines.
void default_constants();

// Runs a simple relative forward/backward drive PID test.
void drive_test();

// Runs a simple turn PID test through several headings.
void turn_test();

// Runs intake and outtake briefly for mechanism testing.
void intake_test();

// Runs each placeholder mechanism briefly for wiring checks.
void full_system_test();

// Moves to an absolute forward coordinate and returns to the start coordinate.
void odom_forward_absolute_test();

// Drives a square using fixed absolute X/Y coordinates.
void odom_square_test();

// Turns through cardinal headings to check odom heading accuracy.
void odom_turn_accuracy_test();

// Prints distance sensor readings without moving the robot.
void distance_debug_test();

// Tests front distance correction to a placeholder target distance.
void distance_front_correct_test();

// Tests squaring to a flat front surface with two front distance sensors.
void distance_square_test();

// Drives with odom first, then trims final distance with front sensors.
void odom_then_distance_correct_test();

// Prints all distance readings, including back and side sensors, without moving.
void back_distance_debug_test();

// Uses back sensors to correct to a placeholder scoring distance.
void back_distance_correct_test();

// Uses back sensors to square to a flat scoring-side surface.
void back_distance_square_test();

// Tests reverse approach, back squaring, and back distance correction.
void back_scoring_alignment_test();

// Tests one-axis X reset from a side distance sensor.
void side_distance_x_reset_test();

// Tests quick-chained travel followed by a precise final pose.
void chained_motion_test();

// Tests fast travel near a goal followed by precise reverse approach.
void precise_back_approach_test();
