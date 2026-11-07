
#include "main.h"
#include "telemetry.hpp"
#include "anti_tip.hpp"
#include "pros/screen.hpp"
#include "liblvgl/llemu.hpp" // Explicit LCD callback declarations for editor and build.

#include <cstdio>
#include <cmath>
#include <atomic>

#include "autons.hpp"
#include "dev_tests.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "EZ-Template/navigation_hooks.hpp"
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

constexpr double HORIZONTAL_TRACKER_DIAMETER_IN = 2.000;
constexpr double VERTICAL_TRACKER_DIAMETER_IN = 1.9585;
// EZ applies the side sign: BACK P5 is negative; RIGHT P7 is positive.
constexpr double HORIZONTAL_TRACKER_OFFSET_IN = 1.400;
constexpr double VERTICAL_TRACKER_OFFSET_IN = 0.800;
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
  // Recovery mode: never access an uninitialized LCD.
  if (!pros::lcd::is_initialized()) return;

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
  const int amplified_turn = clamp_driver_output(-turn * TURN_MULTIPLIER);
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
  // Recovery mode: direct-screen coordinates are handled in opcontrol().
  if (!pros::lcd::is_initialized()) return;

  odom_display_mutex.take();
  auton_selector_screen_active = false;
  clear_brain_lines();
  odom_display_frozen = false;
  odom_diagnostic_screen_release();
  odom_display_mutex.give();
}

void display_odom_pose() {
  pros::lcd::print(4, "LIFT POSITION: %.1f deg", lift_position_deg());
  if (!imu_calibrated) {
    pros::lcd::set_text(0, "IMU CALIBRATION FAILED");
    return;
  }

  const auto pose = nav_pose_get();
  pros::lcd::set_text(0, "2474V");
  pros::lcd::print(1, "X:     %7.2f in", pose.x);
  pros::lcd::print(2, "Y:     %7.2f in", pose.y);
  pros::lcd::print(3, "HEADING: %7.2f deg", pose.theta);  // FRONT heading.
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
    {-1, 4},        // LEFT viewed facing FRONT; positive output moves FRONT-first.
    {-3 , 2},       // RIGHT viewed facing FRONT; positive output moves FRONT-first.
    IMU_PORT,         // Real inertial sensor on Smart Port 20.
    2.75,             // Drive-wheel diameter in inches.
    600,              // Blue motor cartridge speed in RPM.
    600.0 / 450.0);   // External ratio for 450 RPM wheel speed.

// Preserve the physically verified tracking-wheel signs. Offset arguments are
// positive physical magnitudes; the attachment methods apply the side convention.
ez::tracking_wheel horizontal_tracker(HORIZONTAL_TRACKER_PORT,
                                      HORIZONTAL_TRACKER_DIAMETER_IN,
                                      HORIZONTAL_TRACKER_OFFSET_IN,
                                      TRACKER_GEAR_RATIO);

ez::tracking_wheel vertical_tracker(VERTICAL_TRACKER_PORT,
                                    VERTICAL_TRACKER_DIAMETER_IN,
                                    VERTICAL_TRACKER_OFFSET_IN,
                                    TRACKER_GEAR_RATIO);

extern pros::Controller master;


void initialize() {
  // ============================================
  // 2474V STARTUP RECOVERY
  // Driver-control startup without PROS LCD/LVGL
  // ============================================

  // Use the direct Brain screen API only.
  // Do not initialize pros::lcd in recovery mode.
  pros::screen::set_eraser(0x00000000);
  pros::screen::erase();
  pros::screen::set_pen(0x00FFFFFF);
  pros::screen::print(
      pros::E_TEXT_MEDIUM, 1, "2474V RECOVERY START");

  std::printf("RECOVERY: initialize entered\n");
  std::fflush(stdout);

  // 1. Prepare drivetrain control.
  chassis.pid_drive_toggle(true);

  // 2. Initialize existing mechanisms.
  mechanisms_initialize();
  if (mechanism_config::ENABLE_LIFT_STARTUP_ZERO) {

  // If a physical bottom switch is installed,
  // require it to confirm home.
  // Without a switch, the physical home
  // position must be verified by the team.
  const LiftBottomLimit bottom =
      lift_bottom_limit_read();

  const bool bottom_allows_zero =
      bottom == LiftBottomLimit::Unconfigured ||
      bottom == LiftBottomLimit::Pressed;

  if (bottom_allows_zero &&
      lift_zero_at_current_position()) {

    std::printf(
        "LIFT STARTUP ZERO SET: %.2f deg\n",
        lift_position_deg()
    );

  } else {

    std::printf(
        "LIFT STARTUP ZERO FAILED\n"
    );
  }

} else {

  std::printf(
      "LIFT STARTUP ZERO DISABLED\n"
  );
}
  std::printf("RECOVERY: mechanisms initialized\n");
  std::fflush(stdout);

  // 3. Preserve calibrated tracking-wheel setup.
  chassis.odom_tracker_front_set(&horizontal_tracker);
  chassis.odom_tracker_left_set(&vertical_tracker);

  // 4. Preserve driver-control configuration.
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  chassis.opcontrol_drive_activebrake_set(
      DRIVER_ACTIVE_BRAKE_KP);

  // 5. Preserve existing PID constants.
  default_constants();


  // 6. Reset drivetrain sensors.
  chassis.drive_sensor_reset();

  // 7. Calibrate the IMU without the EZ-Template
  // loading animation or PROS LCD interface.
  imu_calibrated = chassis.drive_imu_calibrate(false);

  // 8. Enable odometry only if calibration succeeds.
  if (imu_calibrated) {
    chassis.odom_enable(true);
    std::printf("RECOVERY: IMU OK, odometry enabled\n");
  } else {
    chassis.odom_enable(false);
    std::printf("RECOVERY: IMU FAILED, odometry disabled\n");
  }

  std::fflush(stdout);

  // 7. Report successful completion.
  std::printf("RECOVERY: initialize completed\n");
  std::fflush(stdout);

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 3, "RECOVERY STARTUP OK");
  pros::screen::print(
      pros::E_TEXT_MEDIUM, 5, "DRIVER MODE TEST");

  master.rumble(".");
}


void disabled() {
  ez::autonomous_emergency_stop();

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
constexpr DevTest ACTIVE_DEV_TEST = DevTest::TURN_90;
constexpr bool RUN_CUSTOM_AUTON_TEST = true;
constexpr bool RUN_ROUTE_SEGMENT_TEST = false;
constexpr auton::RouteSegmentTest ACTIVE_ROUTE_SEGMENT = auton::RouteSegmentTest::SNAPSHOT;
static_assert(static_cast<int>(RUN_DEV_TEST)+static_cast<int>(RUN_CUSTOM_AUTON_TEST)+
              static_cast<int>(RUN_ROUTE_SEGMENT_TEST)<=1,"Select only one development mode");
static_assert(!(RUN_DEV_TEST && RUN_CUSTOM_AUTON_TEST), "Select only one development mode");

void custom_auton_test() {
  // Temporarily isolate geometry measurement; the diagnostic initializes its pose.
  // No toggle or other mechanism commands in this custom test.
nav_set_start(0, 0, 0);
toggle_run();
pros::delay(395);
toggle_stop();

}


void autonomous() {
  pros::screen::set_eraser(0x00000000);
  pros::screen::erase();
  pros::screen::set_pen(0x00FFFFFF);

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 1, "AUTON ENTERED");
                
  ez::drive_stop(&chassis);
  ez::autonomous_begin();  // ----- Autonomous dispatch -----
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
    run_dev_test(ACTIVE_DEV_TEST);
    return;
  }

  if (RUN_CUSTOM_AUTON_TEST) {

    pros::screen::print(
        pros::E_TEXT_MEDIUM, 2, "90 DEG TEST RUNNING");

    custom_auton_test();

    const auto pose = nav_pose_get();

    pros::screen::print(
        pros::E_TEXT_MEDIUM, 2, "90 DEG TEST FINISHED");

    pros::screen::print(
        pros::E_TEXT_MEDIUM, 3,
        "X: %.2f in", pose.x);

    pros::screen::print(
        pros::E_TEXT_MEDIUM, 4,
        "Y: %.2f in", pose.y);

    pros::screen::print(
        pros::E_TEXT_MEDIUM, 5,
        "HEADING: %.2f deg", pose.theta);

    return;
}

  nav_print_sensor_health(); // Informational, not a selector gate.
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
mechanisms_preflight(false, false, false);


    // Fill the Brain's display area with green.
  // Fill the Brain's display area with green.
  pros::screen::set_pen(0x0000FF00);
  pros::screen::fill_rect(0, 0, 479, 239);

  // Keep green background with black text.
  pros::screen::set_eraser(0x0000FF00);
  pros::screen::set_pen(0x00000000);

pros::screen::print(
    pros::E_TEXT_MEDIUM, 1, "2474V VALKYRIE");

int display_tick = 0;

while (true) {
    if (pros::competition::is_disabled() ||
        pros::competition::is_autonomous()) {
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
    driver_arcade_control(left_y, -right_x);

    // Temporary full-voltage commissioning: held L1 up, held L2 down.
    // Release or both held stops both motors; no presets or automatic return.
    // L1 requests up; L2 requests down. Motor configuration is unchanged.
    // Manual lift controls.
const bool lift_up =
    master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

const bool lift_down =
    master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

test_lift_motor_response(lift_up, lift_down);


// Y alone moves the lift to its calibrated Intake position.
// The zero reference is established only during startup.


if (!mechanism_config::ENABLE_LIFT_CALIBRATION_MODE &&
    master.get_digital_new_press(
        pros::E_CONTROLLER_DIGITAL_DOWN)) {

    if (!lift_up && !lift_down) {

        if (lift_move_preset(
                lift_calibration::Position::INTAKE)) {
            std::printf("INTAKE PRESET ACCEPTED\n");
        } else {
            std::printf("INTAKE PRESET REJECTED\n");
        }

    } else {
        std::printf("INTAKE REJECTED: release L1 and L2\n");
    }
}

    claw_driver_control(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2),
                        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1));
    // B and X request opposite toggle directions; both or neither means stop.
    const bool toggle_ccw = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    const bool toggle_cw = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (toggle_ccw && !toggle_cw) toggle_run();
    else if (toggle_cw && !toggle_ccw) toggle_run_reverse();
    else toggle_stop();
    mechanisms_update();
    mechanisms_print_debug();
    pose_recorder::update(master);


telemetry_update(master);

// Refresh the live coordinates every ~100 ms.
if (++display_tick >= 10) {
  display_tick = 0;

  const auto pose = nav_pose_get();

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 2,
      "X:       %8.2f in   ", pose.x);

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 3,
      "Y:       %8.2f in   ", pose.y);

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 4,
      "HEADING:  %8.2f deg ", pose.theta);


  const double lift_deg = lift_position_deg();

  if (std::isfinite(lift_deg)) {
    pros::screen::print(
        pros::E_TEXT_MEDIUM, 5,
        "LIFT: %8.1f deg     ", lift_deg);
  } else {
    pros::screen::print(
        pros::E_TEXT_MEDIUM, 5,
        "LIFT: SENSOR ERROR      ");
  }

  // Display whether IMU calibration succeeded.
pros::screen::print(
    pros::E_TEXT_MEDIUM, 6,
    "IMU: %s             ",
    imu_calibrated ? "CALIBRATED" : "FAILED");

// Existing driver-control label.
pros::screen::print(
    pros::E_TEXT_MEDIUM, 7,
    "DRIVER CONTROL");
}

pros::delay(DRIVER_LOOP_MS);
}   
}
