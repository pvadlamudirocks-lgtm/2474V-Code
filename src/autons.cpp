#include "autons.hpp"
#include "field_points.hpp"
#include "route_config.hpp"

#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "main.h"
#include "navigation.hpp"
#include "motion_exit_trace.hpp"

namespace {
constexpr int OFFSET_CALIBRATION_ITERATIONS = 10;
constexpr int OFFSET_CALIBRATION_TURN_SPEED = 63;
constexpr int OFFSET_SETTLE_TIME_MS = 250;
constexpr double MIN_VALID_TURN_RADIANS = 0.01;

std::atomic<bool> diagnostic_screen_active{false};

double wrapped_angle_error(double target, double current) {
  double error = target - current;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

void show_turn_drift_on_brain(double target_heading, int speed,
                              double delta_x, double delta_y,
                              double drift, double heading_error) {
  pros::lcd::set_text(0, "ROBOT TURN DRIFT");
  pros::lcd::print(1, "Target: %7.2f deg", target_heading);
  pros::lcd::print(2, "Speed:  %3d", speed);
  pros::lcd::print(3, "dX: %8.2f in", delta_x);
  pros::lcd::print(4, "dY: %8.2f in", delta_y);
  pros::lcd::print(5, "Drift: %5.2f in", drift);
  pros::lcd::print(6, "Head err: %6.2f deg", heading_error);
  pros::lcd::set_text(7, "Full data in terminal");
}
}  // namespace

// ----- Brain selector entries -----
// Register permanent route names here; route settings live in route_config.hpp.
void register_autons() {
  // Route contents remain separate from the existing selector UI.
  ez::as::auton_selector.autons_add({
      {"2474V LEFT (not configured)", auton_left},
      {"2474V RIGHT (not configured)", auton_right},
      {"2474V SAFE (not configured)", auton_safe},
      {"2474V SKILLS (not configured)", skills},
  });
}

// ----- EZ motion tuning -----
// Shared PID, exit, slew and path settings are grouped here for controlled field tuning.
void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);

  chassis.pid_turn_exit_condition_set(150_ms, 1_deg, 250_ms, 7_deg,
                                      500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg,
                                       500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in,
                                       500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg,
                                           500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in,
                                            500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);
  chassis.odom_boomerang_distance_set(16_in);
  chassis.odom_boomerang_dlead_set(0.625);
  chassis.pid_angle_behavior_set(ez::shortest);
}

bool odom_diagnostic_screen_is_active() {
  return diagnostic_screen_active.load();
}

void odom_diagnostic_screen_release() {
  diagnostic_screen_active.store(false);
}

// Based on https://github.com/EZ-Robotics/EZ-Template/blob/v3.2.2/src/autons.cpp
// Report only. Use actual unwrapped rotation for alternating raw +90/-90 trials.
void measure_offsets() {
  auto* back = chassis.odom_tracker_back;
  auto* right = chassis.odom_tracker_right;
  if (back == nullptr || right == nullptr ||
      back->smart_encoder.get_port() != 5 || right->smart_encoder.get_port() != 7 ||
      !back->smart_encoder.is_installed() || !right->smart_encoder.is_installed()) {
    std::printf("CALIBRATION ERROR: require installed BACK P5 and RIGHT P7.\n");
    std::fflush(stdout);
    return;
  }
  const auto previous_brake = chassis.drive_brake_get();
  const bool previous_print = chassis.pid_print_toggle_get();
  chassis.pid_print_toggle(false);
  std::printf("\n===== TRACKER OFFSET CALIBRATION =====\n"
              "10 trials: raw EZ +90/-90 degrees, speed 63; heading reset per trial.\n"
              "Raw deltas: centidegrees. Offsets: magnitudes in inches. NOT APPLIED.\n"
              "Angle divisor: actual unwrapped IMU rotation.\n\n");
  std::fflush(stdout);
  double h_sum = 0, v_sum = 0;
  double h_min = INFINITY, h_max = -INFINITY;
  double v_min = INFINITY, v_max = -INFINITY;
  int valid = 0;
  double directional_h_sum[2] = {}, directional_v_sum[2] = {};
  int directional_valid[2] = {};
  double horizontal_values[OFFSET_CALIBRATION_ITERATIONS];
  double vertical_values[OFFSET_CALIBRATION_ITERATIONS];
  for (int i = 0; i < OFFSET_CALIBRATION_ITERATIONS; ++i) {
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    pros::delay(20);  // Allow reset readings to reach the tracking task.
    const double heading_before = chassis.drive_imu_get();
    const double h_before = back->smart_encoder.get_position();
    const double v_before = right->smart_encoder.get_position();
    const double target = i % 2 == 0 ? 90.0 : -90.0;
    turn_velocity_trace_begin();
    chassis.pid_turn_set(target, OFFSET_CALIBRATION_TURN_SPEED, ez::raw);
    chassis.pid_wait();
    pros::delay(OFFSET_SETTLE_TIME_MS);
    chassis.drive_set(0, 0);
    const double heading_change = chassis.drive_imu_get() - heading_before;
    const double h_raw = back->smart_encoder.get_position() - h_before;
    const double v_raw = right->smart_encoder.get_position() - v_before;
    const double radians = std::fabs(ez::util::to_rad(heading_change));
    const double h_inches = h_raw / back->ticks_per_inch();
    const double v_inches = v_raw / right->ticks_per_inch();
    const bool usable = std::isfinite(radians) && radians >= MIN_VALID_TURN_RADIANS &&
                        std::isfinite(h_inches) && std::isfinite(v_inches);
    const double horizontal = usable ? std::fabs(h_inches) / radians : NAN;
    const double vertical = usable ? std::fabs(v_inches) / radians : NAN;
    horizontal_values[i] = horizontal;
    vertical_values[i] = vertical;
    std::printf("TRIAL %d\nEZ target = %.0f\nactual unwrapped heading change = %.5f deg\n"
                "horizontal raw delta = %.0f\nvertical raw delta = %.0f\n"
                "horizontal travel inches = %.5f\nvertical travel inches = %.5f\n"
                "calculated horizontal offset magnitude = %.5f in\ncalculated vertical offset magnitude = %.5f in\n"
                "turn exit reason = %s\n"
                "measurement valid = %s\n\n", i + 1, target, heading_change, h_raw, v_raw,
                h_inches, v_inches, horizontal, vertical,
                ez::exit_to_string(turn_velocity_trace_result()).c_str(),
                usable ? "true" : "false (excluded)");
    std::fflush(stdout);
    if (!usable) continue;
    h_sum += horizontal; v_sum += vertical; ++valid;
    directional_h_sum[i % 2] += horizontal;
    directional_v_sum[i % 2] += vertical;
    ++directional_valid[i % 2];
    h_min = std::min(h_min, horizontal); h_max = std::max(h_max, horizontal);
    v_min = std::min(v_min, vertical); v_max = std::max(v_max, vertical);
  }
  chassis.drive_brake_set(previous_brake);
  chassis.pid_print_toggle(previous_print);
  std::printf("\n+90 trials horizontal average = %.5f\n+90 trials vertical average = %.5f\n"
              "-90 trials horizontal average = %.5f\n-90 trials vertical average = %.5f\n"
              "valid directional trials: +90 = %d/5, -90 = %d/5\n",
              directional_valid[0] ? directional_h_sum[0] / directional_valid[0] : NAN,
              directional_valid[0] ? directional_v_sum[0] / directional_valid[0] : NAN,
              directional_valid[1] ? directional_h_sum[1] / directional_valid[1] : NAN,
              directional_valid[1] ? directional_v_sum[1] / directional_valid[1] : NAN,
              directional_valid[0], directional_valid[1]);
  std::printf("\nHORIZONTAL:\nall 10 values (in):");
  for (double value : horizontal_values) std::printf(" %.5f", value);
  std::printf("\nVERTICAL:\nall 10 values (in):");
  for (double value : vertical_values) std::printf(" %.5f", value);
  std::printf("\n");
  std::printf("FINAL:\nvalid trials = %d/%d\n"
              "horizontal average = %.5f\nhorizontal min = %.5f\nhorizontal max = %.5f\n"
              "horizontal spread = %.5f\n\n"
              "vertical average = %.5f\nvertical min = %.5f\nvertical max = %.5f\n"
              "vertical spread = %.5f\n"
              "NOT APPLIED: configured offsets remain unchanged.\n================================\n",
              valid, OFFSET_CALIBRATION_ITERATIONS,
              valid ? h_sum / valid : NAN, valid ? h_min : NAN, valid ? h_max : NAN,
              valid ? h_max - h_min : NAN,
              valid ? v_sum / valid : NAN, valid ? v_min : NAN, valid ? v_max : NAN,
              valid ? v_max - v_min : NAN);
  std::fflush(stdout);
}
// Preserve the existing diagnostic entry point.
void measure_tracking_offsets() { measure_offsets(); }

// ----- Turn and tracking diagnostics -----
// Compare heading change with odometry drift to investigate tracking geometry.
void run_turn_drift_test(double target_heading, int speed) {
  diagnostic_screen_active.store(true);

  const double start_x = chassis.odom_x_get();
  const double start_y = chassis.odom_y_get();
  const double start_theta = chassis.odom_theta_get();

  // Raw absolute behavior is intentional so a target of 360 performs a full
  // revolution instead of being treated as equivalent to 0 degrees.
  chassis.pid_turn_set(target_heading, speed, ez::raw);
  chassis.pid_wait();

  const double end_x = chassis.odom_x_get();
  const double end_y = chassis.odom_y_get();
  const double end_theta = chassis.odom_theta_get();
  const double delta_x = end_x - start_x;
  const double delta_y = end_y - start_y;
  const double drift = std::hypot(delta_x, delta_y);
  const double heading_error = wrapped_angle_error(target_heading, end_theta);

  chassis.drive_set(0, 0);

  std::printf("\nROBOT FRAME TURN DRIFT TEST\n");
  std::printf("Target heading: %.2f deg\n", target_heading);
  std::printf("Speed: %d\n", speed);
  std::printf("Start: X %.3f in | Y %.3f in | Theta %.3f deg\n",
              start_x, start_y, start_theta);
  std::printf("End:   X %.3f in | Y %.3f in | Theta %.3f deg\n",
              end_x, end_y, end_theta);
  std::printf("Delta X: %.3f in\n", delta_x);
  std::printf("Delta Y: %.3f in\n", delta_y);
  std::printf("Position drift magnitude: %.3f in\n", drift);
  std::printf("Heading error: %.3f deg\n\n", heading_error);

  show_turn_drift_on_brain(target_heading, speed, delta_x, delta_y, drift,
                           heading_error);
}

void odom_turn_90_speed_40_test() { run_turn_drift_test(90.0, 40); }
void odom_turn_90_speed_60_test() { run_turn_drift_test(90.0, 60); }
void odom_turn_90_speed_100_test() { run_turn_drift_test(90.0, 100); }
void odom_turn_180_speed_40_test() { run_turn_drift_test(180.0, 40); }
void odom_turn_360_speed_40_test() { run_turn_drift_test(360.0, 40); }

void odom_geometry_turn_90_test() {
  // Static snapshots and an independent touch task survive autonomous ending.
  // Pages contain captured text only: browsing never rereads or resets sensors.
  static std::string pages[8][8];
  static int page = 0;
  static bool pages_ready = false;  // Protected by page_mutex.
  static pros::Mutex page_mutex;
  page_mutex.take();
  pages_ready = false;
  page_mutex.give();
  auto show_page = []() {
    if (!pages_ready || !odom_diagnostic_screen_is_active()) return;
    for (int line = 0; line < 8; ++line)
      pros::lcd::set_text(line, pages[page][line]);
  };
  diagnostic_screen_active.store(true);
  // Let any in-progress normal screen refresh finish before taking the screen.
  pros::delay(60);
  for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
  pros::lcd::set_text(0, "ROBOT 90 DEG TURN");
  pros::lcd::set_text(6, "RUNNING - SPEED 40");

  chassis.drive_set(0, 0);
  auto* back = chassis.odom_tracker_back;
  auto* right = chassis.odom_tracker_right;
  if (back == nullptr || right == nullptr ||
      !back->smart_encoder.is_installed() || !right->smart_encoder.is_installed()) {
    pros::lcd::set_text(6, "STOPPED: tracker missing");
    std::printf("RAW TURN TEST ABORTED: BACK/RIGHT rotation sensor missing.\n");
    return;
  }
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  pros::delay(50);
  const auto start = chassis.odom_pose_get();
  // Direct Rotation sensor accumulated position, in signed centidegrees.
  // EZ tracking_wheel::get_raw() delegates to this same get_position() API.
  const double back_raw_start = back->smart_encoder.get_position();
  const double right_raw_start = right->smart_encoder.get_position();
  const double back_tpi = back->ticks_per_inch();
  const double right_tpi = right->ticks_per_inch();
  const double imu_start = chassis.drive_imu_get();
  pros::lcd::print(6, "B cd: %.0f -> ...", back_raw_start);
  pros::lcd::print(7, "R cd: %.0f -> ...", right_raw_start);
  std::printf("\nROBOT FRAME: 90 DEG TURN | speed 40\n");
  std::printf("RAW DIAGNOSTIC: cd = signed sensor centidegrees, 36000/revolution.\n");
  std::printf("BACK P%d: diameter %.3f ratio %.3f ticks/in %.6f offset %+.4f flip %d reversed %ld\n",
              back->smart_encoder.get_port(), back->wheel_diameter_get(),
              back->ratio_get(), back_tpi, back->distance_to_center_get(),
              back->distance_to_center_flip_get(), static_cast<long>(back->smart_encoder.get_reversed()));
  std::printf("RIGHT P%d: diameter %.3f ratio %.3f ticks/in %.6f offset %+.4f flip %d reversed %ld\n",
              right->smart_encoder.get_port(), right->wheel_diameter_get(),
              right->ratio_get(), right_tpi, right->distance_to_center_get(),
              right->distance_to_center_flip_get(), static_cast<long>(right->smart_encoder.get_reversed()));
  std::printf("Start: X %.3f in | Y %.3f in | THETA %.3f deg\n",
              start.x, start.y, start.theta);

  chassis.pid_turn_set(90_deg, 40, ez::raw);
  chassis.pid_wait();
  // drive_set disables the motion PID in EZ 3.2.2. Keep odometry running
  // through settling, then capture the actual pose without resetting it.
  chassis.drive_set(0, 0);
  pros::delay(250);
  const auto final_pose = chassis.odom_pose_get();
  const double back_raw_end = back->smart_encoder.get_position();
  const double right_raw_end = right->smart_encoder.get_position();
  const double imu_end = chassis.drive_imu_get();
  const double heading_delta_rad = ez::util::to_rad(imu_end - imu_start);
  const double back_inches = (back_raw_end - back_raw_start) / back_tpi;
  const double right_inches = (right_raw_end - right_raw_start) / right_tpi;
  // EZ uses internal angle = -IMU. A positive-heading pure turn needs
  // BACK travel = +back_offset * angle, RIGHT travel = -right_offset * angle.
  const double back_expected = back->distance_to_center_get() * heading_delta_rad;
  const double right_expected = -right->distance_to_center_get() * heading_delta_rad;
  const double x_drift = final_pose.x - start.x;
  const double y_drift = final_pose.y - start.y;

  pros::lcd::print(1, "X: %.3f in", final_pose.x);
  pros::lcd::print(2, "Y: %.3f in", final_pose.y);
  pros::lcd::print(3, "THETA: %.3f deg", final_pose.theta);
  pros::lcd::print(4, "X DRIFT: %.3f in", x_drift);
  pros::lcd::print(5, "Y DRIFT: %.3f in", y_drift);
  pros::lcd::print(6, "B cd: %.0f -> %.0f", back_raw_start, back_raw_end);
  pros::lcd::print(7, "R cd: %.0f -> %.0f", right_raw_start, right_raw_end);
  std::printf("BACK raw cd: %.0f -> %.0f | delta %+.0f | travel %+.5f in | expected %+.5f in\n",
              back_raw_start, back_raw_end, back_raw_end - back_raw_start, back_inches, back_expected);
  std::printf("RIGHT raw cd: %.0f -> %.0f | delta %+.0f | travel %+.5f in | expected %+.5f in\n",
              right_raw_start, right_raw_end, right_raw_end - right_raw_start, right_inches, right_expected);
  std::printf("IMU: %.5f -> %.5f deg | unwrapped delta %.6f rad\n", imu_start, imu_end, heading_delta_rad);
  if (std::fabs(heading_delta_rad) > 0.01) {
    std::printf("Signed pure-turn radii: BACK %+.5f in | RIGHT %+.5f in (NOT APPLIED)\n",
                back_inches / heading_delta_rad, -right_inches / heading_delta_rad);
  }
  std::printf("Final: X %.3f in | Y %.3f in | THETA %.3f deg\n",
              final_pose.x, final_pose.y, final_pose.theta);
  std::printf("X DRIFT: %.3f in | Y DRIFT: %.3f in\n",
              x_drift, y_drift);
  std::printf("STOPPED. Final pose retained; no post-turn XY reset.\n");
  const int terminal_flush_status = std::fflush(stdout);
  const int terminal_error_flag = std::ferror(stdout);
  auto text_value = [](const char* format, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), format, value);
    return std::string(buffer);
  };
  page_mutex.take();
  for (int p = 0; p < 8; ++p) {
    for (int line = 0; line < 8; ++line) pages[p][line].clear();
    pages[p][7] = "ROBOT | TAP < / >";
  }
  pages[0][0] = "1/8 STOPPED: 90 DEG";
  pages[0][1] = text_value("X: %+.3f in", final_pose.x);
  pages[0][2] = text_value("Y: %+.3f in", final_pose.y);
  pages[0][3] = text_value("THETA: %.3f deg", final_pose.theta);
  pages[0][4] = text_value("X DRIFT: %+.3f in", x_drift);
  pages[0][5] = text_value("Y DRIFT: %+.3f in", y_drift);
  pages[0][6] = "90 deg / speed 40";

  pages[1][0] = "2/8 BACK RAW";
  pages[1][1] = text_value("BEFORE: %+.0f cd", back_raw_start);
  pages[1][2] = text_value("AFTER:  %+.0f cd", back_raw_end);
  pages[1][3] = text_value("DELTA:  %+.0f cd", back_raw_end - back_raw_start);
  pages[1][4] = text_value("TRAVEL: %+.5f in", back_inches);
  pages[1][5] = text_value("EXPECT: %+.5f in", back_expected);
  pages[1][6] = "cd = 1/100 degree";

  pages[2][0] = "3/8 RIGHT RAW";
  pages[2][1] = text_value("BEFORE: %+.0f cd", right_raw_start);
  pages[2][2] = text_value("AFTER:  %+.0f cd", right_raw_end);
  pages[2][3] = text_value("DELTA:  %+.0f cd", right_raw_end - right_raw_start);
  pages[2][4] = text_value("TRAVEL: %+.5f in", right_inches);
  pages[2][5] = text_value("EXPECT: %+.5f in", right_expected);
  pages[2][6] = "36000 cd = 1 rev";

  pages[3][0] = "4/8 IMU HEADING";
  pages[3][1] = text_value("BEFORE: %.4f deg", imu_start);
  pages[3][2] = text_value("AFTER:  %.4f deg", imu_end);
  pages[3][3] = text_value("CHANGE: %+.4f deg", imu_end - imu_start);
  pages[3][4] = text_value("CHANGE: %+.6f rad", heading_delta_rad);
  pages[3][5] = "Unwrapped EZ IMU";
  pages[3][6] = "Saved after settling";

  pages[4][0] = "5/8 BACK CONFIG";
  pages[4][1] = text_value("PORT: %.0f / BACK", back->smart_encoder.get_port());
  pages[4][2] = text_value("OFFSET: %+.4f in", back->distance_to_center_get());
  pages[4][3] = text_value("REVERSED: %.0f", back->smart_encoder.get_reversed());
  pages[4][4] = text_value("OFFSET FLIP: %.0f", back->distance_to_center_flip_get());
  pages[4][5] = "0 = NO / 1 = YES";
  pages[4][6] = "Flags, not direction proof";

  pages[5][0] = "6/8 RIGHT CONFIG";
  pages[5][1] = text_value("PORT: %.0f / RIGHT", right->smart_encoder.get_port());
  pages[5][2] = text_value("OFFSET: %+.4f in", right->distance_to_center_get());
  pages[5][3] = text_value("REVERSED: %.0f", right->smart_encoder.get_reversed());
  pages[5][4] = text_value("OFFSET FLIP: %.0f", right->distance_to_center_flip_get());
  pages[5][5] = "0 = NO / 1 = YES";
  pages[5][6] = "Flags, not direction proof";

  pages[6][0] = "7/8 BACK SCALE";
  pages[6][1] = text_value("DIAMETER: %.4f in", back->wheel_diameter_get());
  pages[6][2] = text_value("GEAR RATIO: %.4f", back->ratio_get());
  pages[6][3] = text_value("TICKS/REV: %.0f", back->ticks_per_rev_get());
  pages[6][4] = text_value("TICKS/IN: %.4f", back_tpi);
  pages[6][5] = "Raw = sensor position";
  pages[6][6] = "Snapshot: no live updates";

  pages[7][0] = "8/8 RIGHT SCALE";
  pages[7][1] = text_value("DIAMETER: %.4f in", right->wheel_diameter_get());
  pages[7][2] = text_value("GEAR RATIO: %.4f", right->ratio_get());
  pages[7][3] = text_value("TICKS/REV: %.0f", right->ticks_per_rev_get());
  pages[7][4] = text_value("TICKS/IN: %.4f", right_tpi);
  pages[7][5] = text_value("STDOUT FLUSH: %.0f", terminal_flush_status);
  pages[7][6] = text_value("STDOUT ERROR: %.0f", terminal_error_flag);
  page = 0;
  pages_ready = true;
  show_page();
  page_mutex.give();
  // Bypass the legacy LCD button callbacks. The entire 480x240 touch surface
  // is two large navigation zones. Count new presses, not held samples.
  // This task only draws frozen strings; it never issues a motion or pose reset.
  static pros::Task diagnostic_touch_task([]() {
    std::int32_t last_press = pros::screen::touch_status().press_count;
    while (true) {
      const auto touch = pros::screen::touch_status();
      if (touch.touch_status != pros::E_TOUCH_ERROR) {
        const bool new_press = touch.press_count != last_press;
        last_press = touch.press_count;
        if (new_press && touch.x >= 0 && touch.x < 480 &&
            touch.y >= 0 && touch.y < 240) {
          page_mutex.take();
          if (pages_ready && odom_diagnostic_screen_is_active()) {
            page = (page + (touch.x < 240 ? 7 : 1)) % 8;
            for (int line = 0; line < 8; ++line)
              pros::lcd::set_text(line, pages[page][line]);
          }
          page_mutex.give();
        }
      }
      pros::delay(25);
    }
  }, "Odom diagnostic touch");
  (void)diagnostic_touch_task;
  // No further motion. The diagnostic screen remains latched until driver mode.
}

void odom_forward_24_test() {
  diagnostic_screen_active.store(true);
  pros::delay(60);
  for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
  pros::lcd::set_text(0, "ROBOT +24 IN TEST");
  pros::lcd::set_text(1, "Target: X 0 / Y 24 in");
  pros::lcd::set_text(2, "RUNNING - SPEED 60");

  chassis.drive_set(0, 0);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_targets_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  pros::delay(50);
  // Drive distance independently of the tracking-wheel geometry under test.
  chassis.pid_drive_set(24_in, 60);
  chassis.pid_wait();
  chassis.drive_set(0, 0);
  pros::delay(250);
  const auto pose = chassis.odom_pose_get();
  pros::lcd::print(2, "X: %+.3f in", pose.x);
  pros::lcd::print(3, "Y: %+.3f in", pose.y);
  pros::lcd::print(4, "THETA: %.3f deg", pose.theta);
  pros::lcd::print(5, "Y ERROR: %+.3f in", pose.y - 24.0);
  pros::lcd::set_text(6, "STOPPED - SPEED 60");
  pros::lcd::set_text(7, "Actual odometry retained");
  std::printf("\nROBOT FRAME +24 IN | speed 60 | target X 0 Y 24 theta 0\n");
  std::printf("Final X %+.3f in | Y %+.3f in | theta %.3f deg | Y error %+.3f in\n",
              pose.x, pose.y, pose.theta, pose.y - 24.0);
  std::fflush(stdout);
}

namespace {
void test_screen_begin(const char* title) {
  chassis.drive_set(0, 0);
  diagnostic_screen_active.store(true);
  pros::delay(60);  // Allow an in-progress normal display refresh to finish.
  for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
  pros::lcd::set_text(0, title);
}

void test_screen_finish() {
  chassis.drive_set(0, 0);
  pros::delay(250);  // Capture the settled pose without resetting odometry.
  const auto pose = nav_pose_get();
  for (int line = 1; line < 8; ++line) pros::lcd::set_text(line, "");
  pros::lcd::print(1, "X: %+.3f in", pose.x);
  pros::lcd::print(2, "Y: %+.3f in", pose.y);
  pros::lcd::print(3, "HEADING: %.3f deg", pose.theta);
  pros::lcd::set_text(4, "STOPPED - FINAL POSE");
  pros::lcd::set_text(5, "Photograph these values");
}
}  // namespace

// ----- One-command navigation tests -----
// Show the requested motion, run it once, then retain the final pose on the Brain.
void test_go_to(double x, double y, int speed) {
  test_screen_begin("TEST: GO TO");
  pros::lcd::print(1, "X: %.1f in", x);
  pros::lcd::print(2, "Y: %.1f in", y);
  pros::lcd::print(3, "SPEED: %d", speed);
  pros::delay(1000);
  nav_go_to(x, y, speed);
  test_screen_finish();
}

void test_go_to_pose(double x, double y, double theta, int speed) {
  test_screen_begin("TEST: GO TO POSE");
  pros::lcd::print(1, "X: %.1f in", x);
  pros::lcd::print(2, "Y: %.1f in", y);
  pros::lcd::print(3, "THETA: %.1f deg", theta);
  pros::lcd::print(4, "SPEED: %d", speed);
  pros::delay(1000);
  nav_go_to_pose(x, y, theta, speed);
  test_screen_finish();
}

void test_turn_to(double theta, int speed) {
  test_screen_begin("TEST: TURN");
  pros::lcd::print(1, "TARGET: %.1f deg", theta);
  pros::lcd::print(2, "SPEED: %d", speed);
  pros::delay(1000);
  nav_turn_to(theta, speed);
  test_screen_finish();
}

void test_drive_distance(double inches, int speed) {
  test_screen_begin("TEST: DRIVE DISTANCE");
  pros::lcd::print(1, "DISTANCE: %.1f in", inches);
  pros::lcd::print(2, "SPEED: %d", speed);
  pros::delay(1000);
  nav_drive_distance(inches, speed);
  test_screen_finish();
}

// Temporary read-only screen trace; no sensor, pose, or motion writes.
void odom_trace_screen_show(const char* label) {
  const auto internal = chassis.odom_pose_get();
  const auto game = nav_internal_to_game_pose(internal);
  const double imu = chassis.drive_imu_get();
  if (!diagnostic_screen_active.exchange(true)) pros::delay(60);
  for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
  pros::lcd::set_text(0, label);
  pros::lcd::print(1, "PUBLIC X/Y/H: %.2f %.2f %.2f", game.x, game.y, game.theta);
  pros::lcd::print(2, "EZ X/Y/H: %.2f %.2f %.2f", internal.x, internal.y, internal.theta);
  pros::lcd::print(3, "IMU H: %.3f", imu);
}
void odom_start_screen_test() {
  diagnostic_screen_active.store(true);
  pros::delay(60);  // Let an in-progress normal screen refresh finish.
  auto show_headings = [](const char* label) {
    const auto ez_pose = chassis.odom_pose_get();
    const auto public_pose = nav_internal_to_game_pose(ez_pose);
    const double imu_heading = chassis.drive_imu_get();
    for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
    pros::lcd::set_text(0, label);
    pros::lcd::print(1, "PUBLIC H: %.4f", public_pose.theta);
    pros::lcd::print(2, "EZ INTERNAL H: %.4f", ez_pose.theta);
    pros::lcd::print(3, "IMU H: %.4f", imu_heading);
    pros::lcd::set_text(5, "NAV_SET_START SNAPSHOT");
    pros::lcd::set_text(6, "NO MOTION TEST");
  };

  show_headings("BEFORE");
  pros::delay(2000);
  nav_set_start(0.0, 0.0, 0.0);
  pros::delay(250);
  show_headings("AFTER");
  pros::lcd::set_text(7, "Photograph this result");
  pros::delay(10000);
  // Leave AFTER latched; existing opcontrol() releases the diagnostic screen.
}
void odom_frame_exit_test() {
  nav_set_start(0.0, 0.0, 0.0);
  const auto ez_before = chassis.odom_pose_get();
  const auto public_before = nav_internal_to_game_pose(ez_before);
  std::printf("PUBLIC BEFORE MOVE:\nX = %.6f\nY = %.6f\nH = %.6f\n"
              "EZ BEFORE MOVE:\nX = %.6f\nY = %.6f\nH = %.6f\n",
              public_before.x, public_before.y, public_before.theta,
              ez_before.x, ez_before.y, ez_before.theta);
  // Do not start this physical test with a stale or invalid heading/pose.
  const double imu_heading = chassis.drive_imu_get();
  if (!nav_validate_pose(0.0, 0.0, 0.0, 0.5, 0.5) ||
      !std::isfinite(imu_heading) ||
      std::fabs(wrapped_angle_error(nav_game_to_internal_heading(0.0), imu_heading)) > 0.5) {
    std::printf("TEST ABORTED: start pose readback does not match (0,0,0).\n");
    std::fflush(stdout);
    return;
  }
  nav_go_to(12.0, 0.0, 30);
  const auto ez_after = chassis.odom_pose_get();
  const auto public_after = nav_internal_to_game_pose(ez_after);
  std::printf("PUBLIC AFTER MOVE:\nX = %.6f\nY = %.6f\nH = %.6f\n"
              "EZ AFTER MOVE:\nX = %.6f\nY = %.6f\nH = %.6f\n",
              public_after.x, public_after.y, public_after.theta,
              ez_after.x, ez_after.y, ez_after.theta);
  std::fflush(stdout);
}

// Record the travel path during one point move, then print samples after stopping.
void odom_clean_point_test() {
  struct Sample {
    std::uint32_t time;
    ez::pose pose;
  };
  struct Telemetry {
    std::atomic<bool> running{true};
    std::uint32_t start = 0;
    std::vector<Sample> samples;
  };
  // Shared ownership keeps storage valid even if competition deletes autonomous.
  // Bound recording to 30 seconds, and exit on a competition mode change.
  auto data = std::make_shared<Telemetry>();
  data->samples.reserve(1201);
  nav_set_start(0.0, 0.0, 0.0);
  const auto initial_pose = nav_pose_get();
  const auto mode = pros::competition::get_status();
  pros::Task telemetry([data, mode]() {
    if (!pros::Task::notify_take(true, 1000)) return;
    while (data->running.load() &&
           pros::competition::get_status() == mode) {
      const auto elapsed = pros::millis() - data->start;
      if (elapsed >= 30000) break;
      data->samples.push_back({elapsed, nav_pose_get()});
      pros::delay(25);
    }
  }, "Clean point telemetry");

  data->start = pros::millis();
  telemetry.notify();
  nav_go_to(0.0, 24.0, 70);
  const auto movement_time = pros::millis() - data->start;
  data->running.store(false);
  const auto final_pose = nav_pose_get();
  chassis.drive_set(0, 0);
  telemetry.join();  // No worker accesses the samples while they are printed.

  double max_cross_track = 0.0;
  double max_heading_error = 0.0;
  auto print_sample = [&](std::uint32_t time, ez::pose pose) {
    const double dx = -pose.x;
    const double dy = 24.0 - pose.y;
    const double remaining = std::hypot(dx, dy);
    // Public navigation convention: 0 = +Y, 90 = +X.
    // Bearing is undefined at the target; emit NaN and exclude it from the max.
    double bearing = NAN;
    double heading_error = NAN;
    if (remaining > 1e-6) {
      bearing = std::atan2(dx, dy) * 180.0 / std::acos(-1.0);
      if (bearing < 0.0) bearing += 360.0;
      heading_error = wrapped_angle_error(bearing, pose.theta);
      if (std::fabs(heading_error) > max_heading_error)
        max_heading_error = std::fabs(heading_error);
    }
    if (std::fabs(pose.x) > max_cross_track)
      max_cross_track = std::fabs(pose.x);
    std::printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                static_cast<unsigned long>(time), pose.x, pose.y, pose.theta,
                pose.x, remaining, bearing, heading_error);
  };
  std::printf("\ntime_ms,x_in,y_in,heading_deg,cross_track_in,remaining_in,"
              "desired_heading_deg,heading_error_deg\n");
  print_sample(0, initial_pose);
  for (const auto& sample : data->samples) {
    if (sample.time <= movement_time) print_sample(sample.time, sample.pose);
  }
  print_sample(movement_time, final_pose);
  std::printf("\n=== ODOM CLEAN POINT TEST ===\n"
              "TARGET X: 0.00\nTARGET Y: 24.00\n"
              "FINAL X: %.3f\nFINAL Y: %.3f\nFINAL HEADING: %.3f\n"
              "X ERROR: %.3f\nY ERROR: %.3f\nTOTAL POSITION ERROR: %.3f\n"
              "MAX ABS CROSS-TRACK ERROR: %.3f\n"
              "MAX ABS HEADING ERROR: %.3f\nMOVEMENT TIME MS: %lu\n"
              "=============================\n",
              final_pose.x, final_pose.y, final_pose.theta, final_pose.x,
              final_pose.y - 24.0, std::hypot(final_pose.x, final_pose.y - 24.0),
              max_cross_track, max_heading_error,
              static_cast<unsigned long>(movement_time));
  if (movement_time >= 30000)
    std::printf("NOTE: periodic telemetry capped at 30000 ms; maxima cover recorded samples only.\n");
  std::fflush(stdout);
}

void odom_first_coordinate_test() {
  nav_go_to(0.0, 24.0, 60);
  chassis.drive_set(0, 0);
  nav_print_pose();
}

void odom_forward_return_test() {
  nav_go_to(0.0, 24.0, 80);
  nav_go_to_reverse(0.0, 0.0, 80);
  chassis.drive_set(0, 0);
  nav_print_pose();
}

void odom_two_point_coordinate_test() {
  nav_go_to(0.0, 24.0, 80);
  nav_go_to(24.0, 24.0, 80);
  chassis.drive_set(0, 0);
  nav_print_pose();
}

#include "nav_sequence.hpp"
void motion_chaining_test(bool chained) {
  chassis.drive_set(0, 0);
  nav_set_start(0, 0, 0); // Only reset in the entire route.
  std::printf("\n===== MOTION CHAIN TEST %s =====\n"
              "Start FRONT physically facing +Y. Centerline envelope X=0..12 Y=0..30 inches.\n"
              "Allow robot footprint and stopping clearance beyond centerline. No mechanism actuation.\n",
              chained ? "B / CHAINED" : "A / NON-CHAINED");
  std::fflush(stdout);
  NavSequence sequence(true);
  if (!nav_validate_pose(0,0,0,0.5,0.5)) nav_abort("start pose mismatch");
  const auto travel_end = chained ? NavEndMode::CHAIN : NavEndMode::SETTLE;
  nav_events({{[](const NavProgress& p) { return p.distance >= 3; },
               [] { nav_trace_marker(); return true; }}});
  // One timeline. Every rejected/faulted movement prevents all later dispatches.
  bool success = nav_drive_distance(8, 110, travel_end) &&
      nav_turn_to(25, 40, travel_end) &&
      nav_drive_distance(6, 110, travel_end) &&
      nav_follow_path({{4,17},{7,22}}, 110, travel_end) &&
      nav_follow_path({{10,27},{12,30}}, 110, NavEndMode::SETTLE) &&
      nav_turn_to(30, 40, NavEndMode::SETTLE);
  const bool finished = sequence.finish();
  std::printf("Route result=%s finish=%s fault=%s\n", success ? "OK" : "FAILED",
              finished ? "OK" : "FAILED", nav_fault());
  nav_trace_dump();
}

#include "subsystems.hpp"

// ----- Isolated mechanism tests -----
// Each call performs the requested semantic action without adding a driving route.
bool claw_flex_test(ClawFlexTestStep step, int power) {
  if (!claw_pneumatic_available() || !flex_available()) {
    flex_stop();
    std::printf("[CLAW/FLEX TEST] unavailable - check configured ports/directions\n");
    return false;
  }
  bool ok = false;
  switch (step) {
    case ClawFlexTestStep::Stop: ok = flex_stop(); break;
    case ClawFlexTestStep::Open: flex_stop(); ok = claw_open(); break;
    case ClawFlexTestStep::Close: flex_stop(); ok = claw_close(); break;
    case ClawFlexTestStep::Intake: ok = flex_intake(power); break;
    case ClawFlexTestStep::Outtake: ok = flex_outtake(power); break;
  }
  std::printf("[CLAW/FLEX TEST] step=%d power=%d accepted=%d\n", static_cast<int>(step), power, ok);
  return ok;
}
bool toggle_test(bool run) {
  if (!toggle_available()) {
    std::printf("[TOGGLE TEST] unavailable - check configured motor port\n");
    return false;
  }
  const bool ok = run ? toggle_run() : toggle_stop();
  std::printf("[TOGGLE TEST] %s accepted=%d\n", run ? "RUN (P6 + P19)" : "STOP", ok);
  return ok;
}
bool pickup_sequence_test(PickupTestStep step) {
  if (!claw_pneumatic_available() || !flex_available()) {
    flex_stop();
    std::printf("[PICKUP TEST] unavailable - claw/flex not configured\n");
    return false;
  }
  bool ok = false;
  switch (step) {
    case PickupTestStep::Prepare: ok = pickup_prepare(); break;
    case PickupTestStep::Capture: ok = pickup_capture(); break;
    case PickupTestStep::Finish: ok = pickup_finish(); break;
  }
  std::printf("[PICKUP TEST] step=%d accepted=%d\n", static_cast<int>(step), ok);
  return ok;
}
// Route usage (caller chooses positions/events, with no timings assumed):
// pickup_prepare(); nav movement toward element; pickup_capture();
// continue movement; pickup_finish();
// toggle_run(); nav movement / interaction; toggle_stop();

// Future competition routes: no coordinates or mechanisms are dispatched.
void auton_left() { auton::run_route_prefix(route_config::left); }
void auton_right() { auton::run_route_prefix(route_config::right); }
void auton_safe() { auton::run_route_prefix(route_config::safe); }
void skills() { auton::run_route_prefix(route_config::skills); }
