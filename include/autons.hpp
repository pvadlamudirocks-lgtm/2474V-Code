#pragma once

void default_constants();

// Calibration/geometry diagnostics below use INTERNAL / OLD-FRAME headings and sides.
// Official EZ-Template 3.2.2 example method, with report-only results for
// FRONT (P5) and physical RIGHT (P7), even if P7 is registered as LEFT.
void measure_offsets();

// Compatibility entry point for the same calibration.
void measure_tracking_offsets();

// Runs one absolute-heading turn and reports the resulting odometry drift.
void run_turn_drift_test(double target_heading, int speed);

// Individually callable turn-drift diagnostics.
// Isolated geometry validation: reset before turn, stop and retain final pose.
void odom_geometry_turn_90_test();
void odom_turn_90_speed_40_test();
void odom_turn_90_speed_60_test();
void odom_turn_90_speed_100_test();
void odom_turn_180_speed_40_test();
void odom_turn_360_speed_40_test();

// Game-facing tests: claw-front headings, positive distance claw-first; inches.
// Call directly from autonomous(); X/Y use the new claw-front game frame.
void test_go_to(double x, double y, int speed);
void test_go_to_pose(double x, double y, double theta, int speed);
void test_turn_to(double theta, int speed);
void test_drive_distance(double inches, int speed);

// Keeps the normal pose task from overwriting diagnostic results on the Brain.
bool odom_diagnostic_screen_is_active();
void odom_diagnostic_screen_release();
// Disable normal LCD writes, including synchronizing with any in-flight draw.
void odom_display_freeze();

// Isolated game-frame diagnostic, speed 70; buffered 25 ms CSV telemetry.
void odom_clean_point_test();

// Existing first test: (0, 0, 0) -> (0, 24), then stop.
void odom_first_coordinate_test();

// Optional follow-up tests. Neither is called automatically.
void odom_forward_return_test();
void odom_two_point_coordinate_test();

// Verify synchronized start heading, then compare poses for (12,0), speed 30.
void odom_frame_exit_test();

// No-motion Brain snapshots before/after nav_set_start; AFTER remains latched.
void odom_start_screen_test();

// Temporary no-motion state trace; latches the Brain display.
void odom_trace_screen_show(const char* label);
