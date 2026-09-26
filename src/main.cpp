#include "main.h"

#include <cstdio>
#include <cmath>

#include "autons.hpp"
#include "navigation.hpp"
#include "motion_exit_trace.hpp"
#include "subsystems.hpp"

namespace {
constexpr int HORIZONTAL_TRACKER_PORT = 5;
constexpr int VERTICAL_TRACKER_PORT = 7;
constexpr int IMU_PORT = 20;

constexpr double TRACKING_WHEEL_DIAMETER_IN = 2.00;
// Tested-candidate magnitudes from the latest ten-turn report-only calibration.
// EZ 3.2.2 stores magnitudes; FRONT/RIGHT leave the offset sign unflipped.
constexpr double HORIZONTAL_TRACKER_OFFSET_IN = 2.290;
constexpr double VERTICAL_TRACKER_OFFSET_IN = 0.152;
constexpr double TRACKER_GEAR_RATIO = 1.0;

constexpr double TURN_MULTIPLIER = 1.30;
constexpr double DRIVER_ACTIVE_BRAKE_KP = 2.0;
constexpr int DRIVER_DEADZONE = 5;
constexpr int DRIVER_LOOP_MS = 10;
constexpr int ODOM_DISPLAY_REFRESH_MS = 50;

bool imu_calibrated = false;
pros::Mutex odom_display_mutex;
bool odom_display_frozen = false;

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
  // New front is the claw/old back. Rotation keeps its clockwise-positive sign.
  const int forward = -apply_driver_deadzone(forward_input);
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

void display_odom_pose() {
  if (!imu_calibrated) {
    pros::lcd::set_text(0, "IMU CALIBRATION FAILED");
    return;
  }

  const auto pose = nav_pose_get();
  pros::lcd::set_text(0, "CLAW/GAME ODOMETRY");
  pros::lcd::print(1, "X:     %7.2f in", pose.x);
  pros::lcd::print(2, "Y:     %7.2f in", pose.y);
  pros::lcd::print(3, "CLAW THETA: %7.2f deg", pose.theta);  // Game heading.
}

// Read-only manual-push telemetry. Never resets pose/sensors or commands motors.
void manual_odom_sign_diagnostic(bool enabled) {
  static bool sampled = false;
  static std::uint32_t last_ms = 0;
  static double last_vertical = 0.0, last_horizontal = 0.0;
  static ez::pose last_pose;
  if (!enabled) {
    sampled = false;
    return;
  }
  const auto now = pros::millis();
  if (sampled && now - last_ms < 250) return;
  auto* vertical = chassis.odom_tracker_right;
  auto* horizontal = chassis.odom_tracker_front;
  if (vertical == nullptr || horizontal == nullptr) return;
  if (!vertical->smart_encoder.is_installed() ||
      !horizontal->smart_encoder.is_installed()) return;
  const double raw_vertical = vertical->smart_encoder.get_position();
  const double raw_horizontal = horizontal->smart_encoder.get_position();
  const auto ez_pose = chassis.odom_pose_get();
  const auto public_pose = nav_internal_to_game_pose(ez_pose);
  if (!sampled) {
    std::printf("\n=== MANUAL PUSH DIAGNOSTIC ===\n"
                "RAW values are Rotation API centidegrees (configured reversal included).\n"
                "VERTICAL: EZ RIGHT, port=%d reversed=%ld ticks/in=%.6f\n"
                "HORIZONTAL: EZ FRONT, port=%d reversed=%ld ticks/in=%.6f\n"
                "Deltas are since previous sample. Verify PUBLIC H~0 / EZ H~180.\n"
                "No pose reset or motor command is issued by this diagnostic.\n",
                vertical->smart_encoder.get_port(),
                static_cast<long>(vertical->smart_encoder.get_reversed()),
                vertical->ticks_per_inch(), horizontal->smart_encoder.get_port(),
                static_cast<long>(horizontal->smart_encoder.get_reversed()),
                horizontal->ticks_per_inch());
    last_vertical = raw_vertical;
    last_horizontal = raw_horizontal;
    last_pose = ez_pose;
  }
  std::printf("ms=%lu | RAW VERTICAL: %.0f dRAW_V: %+.0f V_IN: %+.4f | "
              "EZ Y: %+.4f dEZ_Y: %+.4f PUBLIC Y: %+.4f | "
              "RAW HORIZONTAL: %.0f dRAW_H: %+.0f H_IN: %+.4f | "
              "EZ X: %+.4f dEZ_X: %+.4f PUBLIC X: %+.4f | "
              "EZ H: %.3f PUBLIC H: %.3f IMU H: %.3f\n",
              static_cast<unsigned long>(now), raw_vertical,
              raw_vertical - last_vertical, vertical->get(),
              ez_pose.y, ez_pose.y - last_pose.y, public_pose.y,
              raw_horizontal, raw_horizontal - last_horizontal, horizontal->get(),
              ez_pose.x, ez_pose.x - last_pose.x, public_pose.x,
              ez_pose.theta, public_pose.theta, chassis.drive_imu_get());
  last_ms = now;
  last_vertical = raw_vertical;
  last_horizontal = raw_horizontal;
  last_pose = ez_pose;
  sampled = true;
}
void odom_display_task() {
  while (true) {
    odom_display_mutex.take();
    if (!odom_display_frozen && !odom_diagnostic_screen_is_active())
      display_odom_pose();
    odom_display_mutex.give();
    manual_odom_sign_diagnostic(false);  // Temporarily silence repeating terminal output.
    pros::delay(ODOM_DISPLAY_REFRESH_MS);
  }
}

}  // namespace

void odom_display_freeze() {
  odom_display_mutex.take();
  odom_display_frozen = true;
  odom_display_mutex.give();
}

ez::Drive chassis(
    {1, -4},          // Left drive: verified motor directions.
    {3, -2},          // Right drive: verified motor directions.
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
  chassis.pid_drive_toggle(true);  // Restore normal powered calibration output.
  pros::lcd::initialize();
  clear_brain_lines();
  mechanisms_initialize();

  chassis.odom_tracker_front_set(&horizontal_tracker);
  horizontal_tracker.distance_to_center_flip_set(true);
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

  clear_brain_lines();
  display_odom_pose();

  static pros::Task odom_display_task_handle(odom_display_task);
  (void)odom_display_task_handle;
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
  std::printf("EFFECTIVE HORIZONTAL OFFSET = %.3f\nEFFECTIVE VERTICAL OFFSET = %.3f\n",
              horizontal_tracker.distance_to_center_get(), vertical_tracker.distance_to_center_get());
  std::fflush(stdout);
  if (!imu_calibrated) return;
  chassis.pid_drive_toggle(true);
  motion_exit_trace_arm();
  nav_set_start(0.0, 0.0, 0.0);
  const auto before_ez = chassis.odom_pose_get();
  const auto before = nav_internal_to_game_pose(before_ez);
  const double horizontal_before = horizontal_tracker.smart_encoder.get_position();
  const double vertical_before = vertical_tracker.smart_encoder.get_position();
  std::printf("\n===== BEFORE 12-IN +X MOVEMENT =====\n"
              "PUBLIC X/Y/H = %.3f %.3f %.3f\nEZ X/Y/H = %.3f %.3f %.3f\n"
              "raw horizontal = %.0f\nraw vertical = %.0f\n",
              before.x, before.y, before.theta, before_ez.x, before_ez.y, before_ez.theta,
              horizontal_before, vertical_before);
  std::fflush(stdout);

  nav_go_to(12.0, 0.0, 30);

  const auto after_ez = chassis.odom_pose_get();
  const auto after = nav_internal_to_game_pose(after_ez);
  std::printf("\n===== AFTER GO_TO =====\nX = %.3f\nY = %.3f\nH = %.3f\n",
              after.x, after.y, after.theta);
  std::fflush(stdout);
  const double horizontal_after = horizontal_tracker.smart_encoder.get_position();
  const double vertical_after = vertical_tracker.smart_encoder.get_position();
  std::printf("\n===== AFTER 12-IN +X MOVEMENT =====\n"
              "PUBLIC X/Y/H = %.3f %.3f %.3f\nEZ X/Y/H = %.3f %.3f %.3f\n"
              "dX = %.3f\ndY = %.3f\nheading = %.3f\n"
              "raw horizontal delta = %.0f\nraw vertical delta = %.0f\n"
              "=================================\n",
              after.x, after.y, after.theta, after_ez.x, after_ez.y, after_ez.theta,
              after.x - before.x, after.y - before.y, after.theta,
              horizontal_after - horizontal_before, vertical_after - vertical_before);
  std::fflush(stdout);
  motion_exit_trace_show();

  nav_turn_to(90.0, 20);
  const auto corrected = nav_pose_get();
  std::printf("\n===== AFTER FINAL HEADING CORRECTION =====\n"
              "X = %.3f\nY = %.3f\nH = %.3f\n"
              "heading error from 90 = %.3f deg\n"
              "position change caused by final turn = %.3f in\n"
              "turn exit reason = %s\n",
              corrected.x, corrected.y, corrected.theta,
              std::remainder(90.0 - corrected.theta, 360.0),
              std::hypot(corrected.x - after.x, corrected.y - after.y),
              ez::exit_to_string(turn_velocity_trace_result()).c_str());
  std::fflush(stdout);
}
void opcontrol() {
  odom_display_mutex.take();
  odom_display_frozen = false;
  odom_diagnostic_screen_release();
  odom_display_mutex.give();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  // EZ 3.2.2 also rebases both active-brake targets to current drive positions.
  // Preserve encoders and odometry; do not depend on the AUTON_RAN flag.
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);

  while (true) {
    const int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    driver_arcade_control(left_y, right_x);

    // Temporary manual lift commissioning: L1 up, L2 down.
    const bool lift_up = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool lift_down = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    test_lift_manual(lift_up, lift_down);
    mechanisms_update();

    pros::delay(DRIVER_LOOP_MS);
  }
}
