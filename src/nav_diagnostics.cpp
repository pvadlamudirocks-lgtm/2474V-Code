#include "nav_diagnostics.hpp"
#include "autons.hpp"

#include <cmath>
#include <cstdio>

// ----- Navigation health -----
// Silent checks are used during motion; the printed snapshot below is for setup.
bool nav_sensors_ready() {
  auto* h=chassis.odom_tracker_back;
  auto* v=chassis.odom_tracker_right;
  const auto p=nav_pose_get();
  return chassis.imu.is_installed() && chassis.imu.get_status()!=pros::ImuStatus::error &&
      !chassis.imu.is_calibrating() && std::isfinite(chassis.imu.get_rotation()) &&
      h && v && h->smart_encoder.is_installed() && v->smart_encoder.is_installed() &&
      h->smart_encoder.get_position()!=PROS_ERR && v->smart_encoder.get_position()!=PROS_ERR &&
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}

namespace {
struct Result {
  ez::pose actual;
  double position_error;
  double heading_error;
  bool interfered;
};

bool finite_pose(ez::pose pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.theta);
}

void clear_result_screen() {
  odom_display_freeze(); // Synchronize with the live LCD task before taking ownership.
  odom_diagnostic_screen_release(); // Retire legacy diagnostic touch pages.
  for (int line = 0; line < 8; ++line) pros::lcd::set_text(line, "");
}

void draw_pose(ez::pose actual) {
  pros::lcd::set_text(0, "2474V");
  pros::lcd::print(1, "X:          %7.2f in", actual.x);
  pros::lcd::print(2, "Y:          %7.2f in", actual.y);
  pros::lcd::print(3, "HEADING: %7.2f deg", actual.theta);
}

bool wait_for_rest() {
  const auto began = pros::millis();
  unsigned stable_ms = 0;
  while (pros::millis() - began < 2000) {
    bool stopped = !chassis.left_motors.empty() && !chassis.right_motors.empty();
    for (const auto* motors : {&chassis.left_motors, &chassis.right_motors}) {
      for (const auto& motor : *motors) {
        const double rpm = motor.get_actual_velocity();
        if (!std::isfinite(rpm) || std::fabs(rpm) >= 3.0) stopped = false;
      }
    }
    stable_ms = stopped ? stable_ms + 10 : 0;
    pros::delay(10);
    if (stable_ms >= 100) return true;
  }
  return false;
}

// Wait for rest before measuring final error, so a moving snapshot is not a final result.
const char* capture_result(ez::pose target, Result& result) {
  if (!finite_pose(target)) return "INVALID TARGET";
  if (!wait_for_rest()) return "REST NOT VERIFIED";
  result.actual = nav_pose_get(); // The centralized, already-working public conversion.
  if (!finite_pose(result.actual)) return "INVALID PUBLIC POSE";
  result.position_error = std::hypot(target.x - result.actual.x, target.y - result.actual.y);
  result.heading_error = target.theta == ez::ANGLE_NOT_SET ? 0.0 :
      std::remainder(target.theta - result.actual.theta, 360.0);
  result.interfered = chassis.interfered;
  return nullptr;
}

void show_result(ez::pose target, const Result& result) {
  clear_result_screen();
  draw_pose(result.actual);
  pros::lcd::print(4, "TGT X:%.2f Y:%.2f in", target.x, target.y);
  pros::lcd::print(5, "POS ERR: %7.2f in", result.position_error);
  if (target.theta != ez::ANGLE_NOT_SET) {
    pros::lcd::print(6, "TARGET H: %7.2f deg", target.theta);
    pros::lcd::print(7, "H ERR:   %+7.2f deg", result.heading_error);
  } else {
    pros::lcd::set_text(7, "STOPPED / NO FINAL H REQUEST");
  }
  if (result.interfered) pros::lcd::set_text(0, "ROBOT: EZ INTERFERED");
}

void print_result(ez::pose target, const Result& result,
                  const char* capture_label = "ROBOT FRAME (after measured rest)") {
  std::printf("\n========== NAV RESULT ==========\n"
              "%s\n"
              "X: %.3f in\nY: %.3f in\nH: %.3f deg\n"
              "TARGET PUBLIC\nX: %.3f in\nY: %.3f in\n"
              "POSITION ERROR: %.3f in\n",
              capture_label, result.actual.x, result.actual.y, result.actual.theta,
              target.x, target.y, result.position_error);
  if (target.theta != ez::ANGLE_NOT_SET) {
    std::printf("TARGET H: %.3f deg\nHEADING ERROR (target-actual, wrapped): %+.3f deg\n",
                target.theta, result.heading_error);
  }
  std::printf("EZ INTERFERED: %s\nRest is not proof of target arrival; inspect errors.\n"
              "===============================\n", result.interfered ? "YES" : "NO");
  std::fflush(stdout);
}

// Use one captured result for both displays so Brain and terminal numbers agree.
bool report(ez::pose target, bool brain, bool terminal) {
  if (!NAV_DIAGNOSTICS) return true;
  Result result{};
  if (const char* failure = capture_result(target, result)) {
    if (brain) {
      clear_result_screen();
      pros::lcd::set_text(0, "NAV RESULT UNAVAILABLE");
      pros::lcd::set_text(1, failure);
    }
    if (terminal) {
      std::printf("NAV RESULT UNAVAILABLE: %s\n", failure);
      std::fflush(stdout);
    }
    return false;
  }
  if (brain) show_result(target, result);
  if (terminal) print_result(target, result);
  return true;
}
} // namespace

void nav_show_pose() {
  if (!NAV_DIAGNOSTICS) return;
  const auto actual = nav_pose_get();
  clear_result_screen();
  if (!finite_pose(actual)) {
    pros::lcd::set_text(0, "INVALID PUBLIC POSE");
    return;
  }
  draw_pose(actual);
  pros::lcd::set_text(7, "CURRENT POSE SNAPSHOT");
}

void nav_print_pose() {
  if (!NAV_DIAGNOSTICS) return;
  const auto actual = nav_pose_get();
  if (!finite_pose(actual)) {
    std::printf("ROBOT CURRENT POSE: INVALID -- unavailable\n");
    std::fflush(stdout);
    return;
  }
  std::printf("ROBOT CURRENT POSE | X: %.3f in | Y: %.3f in | H: %.3f deg\n",
              actual.x, actual.y, actual.theta);
  std::fflush(stdout);
}

bool nav_show_result(double x, double y, double heading) {
  return report({x, y, heading}, true, false);
}

bool nav_print_result(double x, double y, double heading) {
  return report({x, y, heading}, false, true);
}

bool nav_debug_result(double x, double y, double heading) {
  return report({x, y, heading}, true, true);
}

// Read-only manual-push telemetry. Never resets pose/sensors or commands motors.
void nav_manual_odom_sample(bool enabled) {
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
  auto* horizontal = chassis.odom_tracker_back;
  if (vertical == nullptr || horizontal == nullptr) return;
  if (!vertical->smart_encoder.is_installed() ||
      !horizontal->smart_encoder.is_installed()) {
    std::printf("MANUAL ODOM: TRACKER MISSING\n");
    sampled = false; return;
  }
  const double raw_vertical = vertical->smart_encoder.get_position();
  const double raw_horizontal = horizontal->smart_encoder.get_position();
  if (raw_vertical == PROS_ERR || raw_horizontal == PROS_ERR ||
      !std::isfinite(raw_vertical) || !std::isfinite(raw_horizontal) ||
      !chassis.imu.is_installed() || chassis.imu.get_status() == pros::ImuStatus::error ||
      chassis.imu.is_calibrating()) {
    std::printf("MANUAL ODOM: SENSOR READ ERROR; sample omitted\n");
    sampled = false; return;
  }
  const auto ez_pose = chassis.odom_pose_get();
  const auto public_pose = nav_internal_to_game_pose(ez_pose);
  const double vertical_in = vertical->get(), horizontal_in = horizontal->get();
  const double imu_heading = chassis.drive_imu_get();
  if (!finite_pose(public_pose) || !std::isfinite(vertical_in) ||
      !std::isfinite(horizontal_in) || !std::isfinite(imu_heading) ||
      imu_heading == PROS_ERR_F ||
      vertical->smart_encoder.get_position() == PROS_ERR ||
      horizontal->smart_encoder.get_position() == PROS_ERR) {
    std::printf("MANUAL ODOM: INVALID SAMPLE -- omitted\n");
    sampled = false; return;
  }
  if (!sampled) {
    std::printf("\n=== MANUAL PUSH DIAGNOSTIC ===\n"
                "RAW values are Rotation API centidegrees (configured reversal included).\n"
                "VERTICAL: RIGHT, port=%d reversed=%ld ticks/in=%.6f\n"
                "HORIZONTAL: BACK, port=%d reversed=%ld ticks/in=%.6f\n"
                "Deltas are since previous sample. Verify PUBLIC H~0 / EZ H~0.\n"
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
              raw_vertical - last_vertical, vertical_in,
              ez_pose.y, ez_pose.y - last_pose.y, public_pose.y,
              raw_horizontal, raw_horizontal - last_horizontal, horizontal_in,
              ez_pose.x, ez_pose.x - last_pose.x, public_pose.x,
              ez_pose.theta, public_pose.theta, imu_heading);
  last_ms = now;
  last_vertical = raw_vertical;
  last_horizontal = raw_horizontal;
  last_pose = ez_pose;
  sampled = true;
}

namespace {
bool sensor_value_ok(double value) {
  return std::isfinite(value) && value != PROS_ERR && value != PROS_ERR_F;
}
bool print_tracker(ez::tracking_wheel* tracker, const char* label, const char* location) {
  const bool installed = tracker && tracker->smart_encoder.is_installed();
  std::printf("\n%s\ninstalled: %s\nconfigured location: %s\n", label,
              installed ? "YES" : "NO -- SENSOR MISSING", location);
  if (!installed) return false;
  const double raw = tracker->smart_encoder.get_position();
  if (!sensor_value_ok(raw)) {
    std::printf("raw position: ERROR -- UNREADABLE\n");
    return false;
  }
  std::printf("raw position: %.0f centidegrees (configured reversal included)\n", raw);
  return true;
}
}

bool nav_print_sensor_health() {
  std::printf("\n================================\n2474V SENSOR HEALTH\n================================\n");
  const bool back = print_tracker(chassis.odom_tracker_back, "P5 PERPENDICULAR TRACKER", "BACK");
  const bool right = print_tracker(chassis.odom_tracker_right, "P7 PARALLEL TRACKER", "RIGHT");
  const bool installed = chassis.imu.is_installed();
  std::printf("\nP20 IMU\ninstalled: %s\n", installed ? "YES" : "NO -- SENSOR MISSING");
  bool ready = false;
  if (installed) {
    const auto status = chassis.imu.get_status();
    if (status == pros::ImuStatus::error) {
      std::printf("calibrating: UNKNOWN -- STATUS ERROR\n");
    } else {
      const bool calibrating = chassis.imu.is_calibrating();
      std::printf("calibrating: %s\n", calibrating ? "YES" : "NO");
      if (!calibrating) {
        const double heading = chassis.imu.get_heading();
        const double rotation = chassis.imu.get_rotation();
        ready = sensor_value_ok(heading) && sensor_value_ok(rotation);
        if (ready) std::printf("heading: %.3f\nrotation: %.3f\n", heading, rotation);
        else std::printf("heading/rotation: ERROR -- UNREADABLE\n");
      } else std::printf("heading/rotation: UNAVAILABLE DURING CALIBRATION\n");
    }
  }
  std::printf("================================\n");
  std::fflush(stdout);
  return back && right && ready;
}

void nav_print_snapshot(const char* label) {
  std::printf("\n=== %s ===\n", label);
  nav_print_pose();
}

bool nav_print_test_result(const char* label, ez::pose start, ez::pose target,
                           unsigned elapsed_ms) {
  std::printf("\n================================\nTEST: %s\n"
              "START\nX: %.3f\nY: %.3f\nH: %.3f\n"
              "TIME\nelapsed: %u ms (movement call; excludes report/rest observation)\n",
              label, start.x, start.y, start.theta, elapsed_ms);
  Result result{};
  if (const char* failure = capture_result(target, result)) {
    std::printf("FINAL RESULT UNAVAILABLE: %s\n================================\n", failure);
    std::fflush(stdout);
    return false;
  }
  print_result(target, result);
  std::printf("dX (target-final): %+.3f in\ndY (target-final): %+.3f in\n"
              "================================\n", target.x-result.actual.x, target.y-result.actual.y);
  std::fflush(stdout);
  return true;
}

void nav_print_action_record(const char* route, const char* step, ez::pose start,
                             ez::pose target, ez::pose final, unsigned elapsed_ms,
                             const char* phase, bool interfered) {
  std::printf("\nAUTON ROUTE: %s | STEP: %s | %s\n"
              "START X: %.3f Y: %.3f H: %.3f | elapsed: %u ms\n",
              route, step, phase, start.x, start.y, start.theta, elapsed_ms);
  if (!finite_pose(start) || !finite_pose(target) || !finite_pose(final)) {
    std::printf("INVALID POSE: error calculation unavailable\n");
    return;
  }
  const Result result{final, std::hypot(target.x-final.x,target.y-final.y),
      target.theta == ez::ANGLE_NOT_SET ? 0.0 : std::remainder(target.theta-final.theta,360.0), interfered};
  // Reuse the result formatter with an explicit sampling phase; no rest wait/read.
  print_result(target,result,"ROBOT FRAME (captured at action return; rest not inferred)");
  std::printf("dX (target-final): %+.3f | dY (target-final): %+.3f\n",
              target.x-final.x,target.y-final.y);
  std::fflush(stdout);
}
