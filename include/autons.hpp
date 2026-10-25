#pragma once

// Brain selector entries bind the permanent route functions below.
void register_autons();
void auton_two_move_chain_test(); // Existing diagnostic reused by the advanced DevTest dispatcher.

void default_constants();

// Calibration/geometry diagnostics below use canonical robot headings and configured tracker slots.
// Official EZ-Template 3.2.2 example method, with report-only results for
// BACK perpendicular P5 and RIGHT parallel P7.
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

// Robot-frame tests: FRONT headings, positive distance FRONT-first; inches.
// Call directly from autonomous(); X/Y share the native field frame.
void test_go_to(double x, double y, int speed);
void test_go_to_pose(double x, double y, double theta, int speed);
void test_turn_to(double theta, int speed);
void test_drive_distance(double inches, int speed);

// Keeps the normal pose task from overwriting diagnostic results on the Brain.
bool odom_diagnostic_screen_is_active();
void odom_diagnostic_screen_release();
// Disable normal LCD writes, including synchronizing with any in-flight draw.
void odom_display_freeze();

// Isolated robot-frame diagnostic, speed 70; buffered 25 ms CSV telemetry.
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

// One compact timeline, identical geometry/tuning in A=false and B=true.
void motion_chaining_test(bool chained);
// Individually called steps; no guessed duration, movement, or automatic selection.
// Caller must issue Stop/Finish before leaving the test (disabled also stops motors).
enum class ClawFlexTestStep { Stop, Open, Close, Intake, Outtake };
enum class PickupTestStep { Prepare, Capture, Finish };
bool claw_flex_test(ClawFlexTestStep step, int power = 0);
bool toggle_test(bool run = false);
bool pickup_sequence_test(PickupTestStep step);

// Inert future competition route entry points; implement measured routes here.
void auton_left();
void auton_right();
void auton_safe();
void skills();
