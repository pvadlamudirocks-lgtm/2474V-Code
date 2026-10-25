#include "main.h"
#include "telemetry.hpp"
#include "anti_tip.hpp"
#include "liblvgl/llemu.hpp" // Explicit LCD callback declarations for editor and build.

#include <cstdio>
#include <cmath>
#include <atomic>

#include "autons.hpp"
#include "dev_tests.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "motion_exit_trace.hpp"
#include "path_motion_trace.hpp"
#include "subsystems.hpp"
#include "pose_recorder.hpp"
#include "route_config.hpp"

namespace {
// ----- Tracking geometry and driver settings -----
// Sensor placement lives here; motion PID settings live in default_constants().
constexpr int HORIZONTAL_TRACKER_PORT = 5;
constexpr int VERTICAL_TRACKER_PORT = 7;
constexpr int IMU_PORT = 20;

constexpr double TRACKING_WHEEL_DIAMETER_IN = 2.00;
// Tested-candidate magnitudes from the latest ten-turn report-only calibration.
// EZ applies the side sign: BACK P5 is negative; RIGHT P7 is positive.
constexpr double HORIZONTAL_TRACKER_OFFSET_IN = 2.290;
constexpr double VERTICAL_TRACKER_OFFSET_IN = 0.152;
constexpr double TRACKER_GEAR_RATIO = 1.0;

constexpr double TURN_MULTIPLIER = 1.30;
constexpr double DRIVER_ACTIVE_BRAKE_KP = 3.0;
constexpr int DRIVER_DEADZONE = 5;
constexpr int DRIVER_LOOP_MS = 10;
constexpr int ODOM_DISPLAY_REFRESH_MS = 50;
std::atomic<const char*> development_mode{"DRIVER"};

bool imu_calibrated = false;
bool driver_coast_mode = false;
void restore_driver_brakes() {
  if (!driver_coast_mode) return;
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  driver_coast_mode = false;
}
pros::Mutex odom_display_mutex;
// The selector and live pose display share the Brain screen.
// These flags decide which one may draw while the mutex protects a screen update.
bool odom_display_frozen = false;
bool auton_selector_screen_active = false; // Protected by odom_display_mutex.

// Selector and odometry share the existing screen-ownership lock.
void auton_selector_page_down() {
  odom_display_mutex.take();
  if (auton_selector_screen_active) ez::as::page_down();
  odom_display_mutex.give();
}
void auton_selector_page_up() {
  odom_display_mutex.take();
  if (auton_selector_screen_active) ez::as::page_up();
  odom_display_mutex.give();
}
void show_auton_selector() {
  odom_display_mutex.take();
  odom_display_frozen = true;
  odom_diagnostic_screen_release();
  auton_selector_screen_active = true;
  ez::as::auton_selector.selected_auton_print();
  odom_display_mutex.give();
}

int apply_driver_deadzone(int input) {
  if (input >= -DRIVER_DEADZONE && input <= DRIVER_DEADZONE)
    return 0;
  return input;
}

int clamp_driver_output(double output) {
  if (output > 127.0)
    return 127;
  if (output < -127.0)
    return -127;
  return static_cast<int>(output);
}

void driver_arcade_control(int forward_input, int turn_input) {
  // Mix forward and turn stick inputs into left/right drive commands.
  // The deadzone keeps small stick noise from moving the robot.
  // Native chassis forward leads with FRONT; positive turn is clockwise.
  const int forward = apply_driver_deadzone(forward_input);
  const int turn = apply_driver_deadzone(turn_input);
  const int amplified_turn = clamp_driver_output(turn * TURN_MULTIPLIER);
  const int left_output = clamp_driver_output(forward + amplified_turn);
  const int right_output = clamp_driver_output(forward - amplified_turn);

  chassis.opcontrol_joystick_threshold_iterate(left_output, right_output);
}

void clear_brain_lines() {
  for (int line = 0; line < 8; ++line)
    pros::lcd::set_text(line, "");
}

// Hand screen ownership back to the existing live odometry task.
void show_live_odometry() {
  odom_display_mutex.take();
  auton_selector_screen_active = false;
  clear_brain_lines();
  odom_display_frozen = false;
  odom_diagnostic_screen_release();
  odom_display_mutex.give();
}

void display_odom_pose() {
  if (!imu_calibrated) {
    pros::lcd::set_text(0, "IMU CALIBRATION FAILED");
    return;
  }

  const auto pose = nav_pose_get();
  pros::lcd::set_text(0, "2474V");
  pros::lcd::print(1, "X:     %7.2f in", pose.x);
  pros::lcd::print(2, "Y:     %7.2f in", pose.y);
  pros::lcd::print(3, "HEADING: %7.2f deg", pose.theta);  // FRONT heading.
  pros::lcd::print(4, "Lift: %7.2f deg", lift_position_deg());
  pros::lcd::print(5, "Mode: %s", development_mode.load());
}

void odom_display_task() {
  while (true) {
    anti_tip::update(); // Read-only, even when another Brain page owns the screen.
    odom_display_mutex.take();
    if (NAV_DIAGNOSTICS && !pros::competition::is_disabled() &&
        !odom_display_frozen && !odom_diagnostic_screen_is_active())
      display_odom_pose();
    odom_display_mutex.give();
    pros::delay(ODOM_DISPLAY_REFRESH_MS);
  }
}

}  // namespace

void odom_display_freeze() {
  odom_display_mutex.take();
  odom_display_frozen = true;
  odom_display_mutex.give();
}

// ----- Drivetrain configuration -----
// This is the shared EZ-Template drive used by both driver control and autonomous.
ez::Drive chassis(
    {-3, 2},          // LEFT viewed facing FRONT; positive output moves FRONT-first.
    {-1, 4},          // RIGHT viewed facing FRONT; positive output moves FRONT-first.
    IMU_PORT,         // Real inertial sensor on Smart Port 20.
    2.75,             // Drive-wheel diameter in inches.
    600,              // Blue motor cartridge speed in RPM.
    600.0 / 450.0);   // External ratio for 450 RPM wheel speed.

// Preserve the physically verified tracking-wheel signs. Offset arguments are
// positive physical magnitudes; the attachment methods apply the side convention.
ez::tracking_wheel horizontal_tracker(HORIZONTAL_TRACKER_PORT,
                                      TRACKING_WHEEL_DIAMETER_IN,
                                      HORIZONTAL_TRACKER_OFFSET_IN,
                                      TRACKER_GEAR_RATIO);
ez::tracking_wheel vertical_tracker(VERTICAL_TRACKER_PORT,
                                    TRACKING_WHEEL_DIAMETER_IN,
                                    VERTICAL_TRACKER_OFFSET_IN,
                                    TRACKER_GEAR_RATIO);

pros::Controller master(pros::E_CONTROLLER_MASTER);

void initialize() {
  // ----- Startup -----
  // Initialize mechanisms, attach trackers, calibrate the IMU, then show the selector.
  chassis.pid_drive_toggle(true);  // Restore normal powered calibration output.
  pros::lcd::initialize();
  clear_brain_lines();
  mechanisms_initialize();

  chassis.odom_tracker_back_set(&horizontal_tracker);
  chassis.odom_tracker_right_set(&vertical_tracker);

  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);
  default_constants();

  // EZ-Template's initialize() performs these same three operations, but its
  // void return discards drive_imu_calibrate()'s synchronous success result.
  chassis.opcontrol_curve_sd_initialize();
  imu_calibrated = chassis.drive_imu_calibrate(true);
  chassis.drive_sensor_reset();
  master.rumble(imu_calibrated ? "." : "---");

  register_autons();
  ez::as::initialize();
  pros::lcd::register_btn0_cb(auton_selector_page_down);
  pros::lcd::register_btn2_cb(auton_selector_page_up);
  show_auton_selector();

  static pros::Task odom_display_task_handle(odom_display_task);
  (void)odom_display_task_handle;
}

void disabled() {
  auton::lifecycle_stop("disabled");
  restore_driver_brakes();
  show_auton_selector();
}

void competition_initialize() {
  show_auton_selector();
}

// ----- Autonomous mode selection -----
// Change these switches to choose one test mode; all false uses the Brain selector.
// Development bypasses selector dispatch; the installed Brain selector is for finalized routes.
constexpr bool RUN_DEV_TEST = false;
constexpr DevTest ACTIVE_DEV_TEST = DevTest::MANUAL_ODOM;
constexpr bool RUN_CUSTOM_AUTON_TEST = true;
// Set false ONLY for custom code containing no drivetrain/navigation commands.
constexpr bool CUSTOM_AUTON_USES_NAVIGATION = true;
constexpr bool RUN_ROUTE_SEGMENT_TEST = false;
constexpr auton::RouteSegmentTest ACTIVE_ROUTE_SEGMENT = auton::RouteSegmentTest::SNAPSHOT;
static_assert(static_cast<int>(RUN_DEV_TEST)+static_cast<int>(RUN_CUSTOM_AUTON_TEST)+
              static_cast<int>(RUN_ROUTE_SEGMENT_TEST)<=1,"Select only one development mode");
static_assert(!(RUN_DEV_TEST && RUN_CUSTOM_AUTON_TEST), "Select only one development mode");

void custom_auton_test() {
  // Intentional temporary P6/P19 toggle test; keep route experiments in this function.
  // Existing temporary test, now isolated from selector dispatch.
nav_set_start(0, 0, 0);
toggle_run();
pros::delay(395);
toggle_stop();
// claw_close();
//  flex_intake(127);//flex_intake(13);
// nav_go_to(-17,15,127);//change to whatever you get from driving to position
// nav_turn_to(301.41, 127);
//make sure to also do reverse coordinates here.
// flex_stop();
// pros::delay(200);
// claw_open();
// pros::delay(300);
//toggle-auton
// toggle_run();
// pros::delay(500);
// toggle_stop();

//skills auton
// nav_go_to(0,55,127);

}


void autonomous() {
  // ----- Autonomous dispatch -----
  // Each development branch returns here so it cannot also run a competition route.
  development_mode = RUN_ROUTE_SEGMENT_TEST ? "ROUTE SEGMENT" : RUN_DEV_TEST ? "DEV TEST" : RUN_CUSTOM_AUTON_TEST ? "CUSTOM TEST" : "COMPETITION";
  show_live_odometry();
  stop_all_mechanisms();
  restore_driver_brakes();

  // Cleanup on every normal return; disabled()/opcontrol() handle task shutdown.
  struct Cleanup {
    ~Cleanup() { nav_cancel_sequence(); stop_all_mechanisms(); }
  } cleanup;
  auton::preflight(false); // Optional mechanism/distance snapshot never blocks unrelated tests.

  if (RUN_ROUTE_SEGMENT_TEST) {
    auton::run_route_segment_test(ACTIVE_ROUTE_SEGMENT,route_config::segment_route);
    return;
  }

  if (RUN_DEV_TEST) {
    if (!imu_calibrated && ACTIVE_DEV_TEST != DevTest::STABILITY_MONITOR) return;
    run_dev_test(ACTIVE_DEV_TEST);
    return;
  }

  if (RUN_CUSTOM_AUTON_TEST) {
    if (CUSTOM_AUTON_USES_NAVIGATION && (!imu_calibrated || !nav_print_sensor_health())) return;
    custom_auton_test();
    return;
  }

  if (!imu_calibrated || !nav_print_sensor_health()) return;
  ez::as::auton_selector.selected_auton_call();
}

void opcontrol() {
  // ----- Driver control -----
  // This loop owns driver motor commands and services the mechanism state machines.
  auton::lifecycle_stop("opcontrol");
  development_mode = pose_recorder::enabled ? "POSE RECORDER" : "DRIVER";
  show_live_odometry();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  // EZ 3.2.2 also rebases both active-brake targets to current drive positions.
  // Preserve encoders and odometry; do not depend on the AUTON_RAN flag.
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);

  driver_coast_mode = false;
  telemetry_reset();
  while (true) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
      stop_all_mechanisms();
      restore_driver_brakes();
      return;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && !driver_coast_mode) {
      driver_coast_mode = true;
      chassis.opcontrol_drive_activebrake_set(0.0);
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    }
    const int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    driver_arcade_control(left_y, right_x);

    // Temporary full-voltage commissioning: held L1 up, held L2 down.
    // Release or both held stops both motors; no presets or automatic return.
    test_lift_motor_response(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1),
                             master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
    claw_driver_control(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1),
                        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2));
    // B and X request opposite toggle directions; both or neither means stop.
    const bool toggle_ccw = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    const bool toggle_cw = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (toggle_ccw && !toggle_cw) toggle_run();
    else if (toggle_cw && !toggle_ccw) toggle_run_reverse();
    else toggle_stop();
    mechanisms_update();
    pose_recorder::update(master);

    telemetry_update(master);

    pros::delay(DRIVER_LOOP_MS);
  }
}


