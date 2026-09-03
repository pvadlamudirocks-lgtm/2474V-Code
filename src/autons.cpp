#include "main.h"
#include "autons.hpp"
#include "distance_correction.hpp"
#include "navigation.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;

// Sets all drive and odom PID constants used by test autonomous routines.
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);
  chassis.odom_boomerang_distance_set(16_in);
  chassis.odom_boomerang_dlead_set(0.625);

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// Runs a simple relative drive test using pid_drive_set().
void drive_test() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

// Runs a simple turn test using absolute heading targets.
void turn_test() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

// Runs intake and outtake briefly to confirm motor directions.
void intake_test() {
  intake_on();
  pros::delay(1000);
  outtake_on();
  pros::delay(1000);
  intake_stop();
}

// Runs all placeholder mechanisms briefly for wiring and port checks.
void full_system_test() {
  intake_on();
  pros::delay(500);
  intake_stop();

  clamp_toggle();
  pros::delay(500);
  descore_toggle();
  pros::delay(500);

  lift_up();
  pros::delay(500);
  lift_down();
  pros::delay(500);
  lift_stop();

  scoring_on();
  pros::delay(500);
  scoring_reverse();
  pros::delay(500);
  scoring_stop();
}

// Tests that +Y is forward and returning to (0, 0) uses the fixed origin.
void odom_forward_absolute_test() {
  nav_set_start(0, 0, 0);

  nav_go_to(0, 24, 80);
  nav_print_pose();
  pros::delay(500);

  nav_go_to_reverse(0, 0, 80);
  nav_print_pose();
}

// Tests fixed absolute coordinates by driving a square from the start origin.
void odom_square_test() {
  nav_set_start(0, 0, 0);

  nav_go_to(0, 24, 80);
  nav_print_pose();

  nav_go_to(24, 24, 80);
  nav_print_pose();

  nav_go_to(24, 0, 80);
  nav_print_pose();

  nav_go_to(0, 0, 80);
  nav_print_pose();
}

// Tests absolute heading control at the four cardinal directions.
void odom_turn_accuracy_test() {
  nav_set_start(0, 0, 0);

  nav_turn_to(90, 70);
  nav_print_pose();

  nav_turn_to(180, 70);
  nav_print_pose();

  nav_turn_to(270, 70);
  nav_print_pose();

  nav_turn_to(0, 70);
  nav_print_pose();
}

// Prints distance readings only. Use this first after sensors are installed.
void distance_debug_test() {
  dist_print_debug();
  pros::delay(1000);
}

// Tests front-distance correction to a placeholder 10 inch target.
void distance_front_correct_test() {
  // TODO: Tune this target distance after the actual front mechanism is built.
  dist_print_debug();
  dist_correct_front_distance(10.0, 1.0, 40, 2000);
  nav_print_pose();
}

// Tests squaring to a flat wall or object using both front distance sensors.
void distance_square_test() {
  // TODO: Flip DIST_SQUARE_TURN_SIGN in distance_correction.cpp if turn direction is wrong.
  dist_square_to_front_surface(1.0, 35, 2000);
  nav_print_pose();
}

// Test routine only: odom drives first, then distance trims final spacing.
void odom_then_distance_correct_test() {
  nav_set_start(0, 0, 0);

  nav_go_to(0, 24, 70);
  nav_print_pose();

  // TODO: Tune this 10 inch placeholder after real scoring geometry is known.
  dist_correct_front_distance(10.0, 1.0, 40, 2000);
  nav_print_pose();
}

// Prints every distance sensor reading without moving the robot.
void back_distance_debug_test() {
  dist_print_debug();
  pros::delay(1000);
}

// Tests back distance correction to a placeholder scoring distance.
void back_distance_correct_test() {
  // TODO: Tune this target after the real back scoring geometry is built.
  dist_correct_back_distance(10.0, 1.0, 40, 2000);
  nav_print_pose();
}

// Tests back squaring against a flat goal, wall, or practice target.
void back_distance_square_test() {
  // TODO: Flip DIST_BACK_SQUARE_TURN_SIGN if the robot turns the wrong way.
  dist_square_to_back_surface(1.0, 35, 2000);
  nav_print_pose();
}

// Tests scoring-side alignment without running the final scoring mechanism.
void back_scoring_alignment_test() {
  nav_set_start(0, 0, 0);

  nav_go_to_reverse(0, 24, 70);
  nav_square_to_back_surface(1.0, 35, 2000);
  nav_correct_back_distance(10.0, 1.0, 40, 2000);
  nav_print_pose();

  // scoring_on();  // Add later after scoring geometry and mechanism are proven.
}

// Tests correcting only X from a side wall/reference using a placeholder target.
void side_distance_x_reset_test() {
  nav_set_start(0, 0, 0);
  dist_print_debug();

  // TODO: Replace 48 and 6 with a real wall coordinate and sensor offset.
  nav_reset_x_with_right_distance(48.0, 6.0, true);
  nav_print_pose();
}

// Tests fast chained travel, then finishes with a precise non-chained pose.
void chained_motion_test() {
  nav_set_start(0, 0, 0);

  nav_chain_to(0, 24, 100);
  nav_chain_to(24, 24, 100);
  nav_precise_to_pose(24, 0, 180);
  nav_print_pose();
}

// Tests fast travel near a goal followed by a precise reverse back approach.
void precise_back_approach_test() {
  nav_set_start(0, 0, 0);

  nav_fast_to(0, 18);
  nav_precise_to_pose_reverse(0, 30, 180);

  // Optional once sensors are mounted:
  // nav_square_to_back_surface(1.0, 35, 2000);
  // nav_correct_back_distance(10.0, 1.0, 40, 2000);
  nav_print_pose();
}

// . . .
// Future autonomous routines go here.
// . . .
