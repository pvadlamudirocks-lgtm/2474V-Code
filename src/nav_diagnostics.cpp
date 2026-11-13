#include "nav_diagnostics.hpp"
#include "autons.hpp"

#include <cmath>
#include <cstdio>
#include <cerrno>
#include <atomic>

namespace {
std::atomic<bool> sensor_fault{false};
std::atomic<unsigned> first_fault_ms{0};
void latch_sensor_fault() {
  if (!sensor_fault.exchange(true)) first_fault_ms.store(pros::millis());
}
}
bool nav_sensor_fault_latched() { return sensor_fault.load(); }

// Shared passive device selection: do not infer physical placement or alter geometry.
ez::tracking_wheel* nav_diagnostic_tracker(unsigned index) {
  return index == 0 ? chassis.odom_tracker_front :
         index == 1 ? chassis.odom_tracker_left : nullptr;
}
NavTrackerSample nav_read_tracker(unsigned index) {
  NavTrackerSample s{index == 0 ? "HORIZONTAL FRONT" : "VERTICAL LEFT",
                      index == 0 ? 5 : 7, 0, false, false, false,
                      NAN, NAN, NAN, 0};
  auto* tracker = nav_diagnostic_tracker(index);
  if (!tracker) { s.error = ENODEV; return s; }
  s.actual_port = tracker->smart_encoder.get_port();
  s.configuration_ok = s.actual_port == s.expected_port;
  s.installed = tracker->smart_encoder.is_installed();
  if (!s.installed) { s.error = ENODEV; return s; }
  errno = 0;
  const auto raw = tracker->smart_encoder.get_position();
  s.error = errno;
  s.ticks_per_inch = tracker->ticks_per_inch();
  const bool readable = raw != PROS_ERR && std::isfinite(static_cast<double>(raw));
  if (readable) s.raw_cd = raw;
  s.valid = s.configuration_ok && readable && std::isfinite(s.ticks_per_inch) &&
            s.ticks_per_inch > 0;
  if (s.valid) s.distance_in = s.raw_cd / s.ticks_per_inch;
  return s;
}
namespace {
bool sensor_value_ok(double value) {
  return std::isfinite(value) && value != PROS_ERR && value != PROS_ERR_F;
}
}
// ----- Navigation health -----
// Read-only, silent; retained diagnostic-history behavior, no motion commands.
bool nav_sensors_ready() {
  const auto h = nav_read_tracker(0), v = nav_read_tracker(1);
  const auto p = nav_pose_get();
  const bool ready = chassis.imu.get_port() == 20 && chassis.imu.is_installed() &&
      chassis.imu.get_status() != pros::ImuStatus::error && !chassis.imu.is_calibrating() &&
      sensor_value_ok(chassis.imu.get_rotation()) && sensor_value_ok(chassis.imu.get_heading()) &&
      h.valid && v.valid && sensor_value_ok(p.x) && sensor_value_ok(p.y) && sensor_value_ok(p.theta);
  if (!ready) latch_sensor_fault();
  return ready;
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
  static bool sampled = false, previous_valid[2] = {};
  static unsigned last_ms = 0, first_invalid[3] = {};
  static bool seen_invalid[3] = {};
  static double previous[2] = {};
  if (!enabled) { sampled = false; previous_valid[0] = previous_valid[1] = false; return; }
  const unsigned now = pros::millis();
  if (sampled && now - last_ms < 250) return;
  last_ms = now; sampled = true; // Rate-limit failures as well as healthy samples.
  for (unsigned i = 0; i < 2; ++i) {
    const auto t = nav_read_tracker(i);
    if (!t.valid) {
      latch_sensor_fault();
      if (!seen_invalid[i]) { seen_invalid[i] = true; first_invalid[i] = now; }
    }
    const double delta = t.valid && previous_valid[i] ? t.raw_cd - previous[i] : NAN;
    std::printf("ODOM ms=%u %s expected=P%d actual=P%d attached=%d installed=%d config_ok=%d raw_cd=%.0f valid=%d errno=%d d_raw_cd=%.0f distance_in=%.6f d_in=%.6f first_invalid_seen=%d first_invalid_ms=%u\n",
        now, t.label, t.expected_port, t.actual_port, nav_diagnostic_tracker(i) != nullptr,
        t.installed, t.configuration_ok, t.raw_cd, t.valid, t.error, delta, t.distance_in,
        t.valid ? delta / t.ticks_per_inch : NAN, seen_invalid[i], first_invalid[i]);
    previous_valid[i] = t.valid;
    if (t.valid) previous[i] = t.raw_cd;
  }
  const auto status = chassis.imu.get_status();
  const bool calibrating = chassis.imu.is_calibrating();
  errno = 0;
  const double heading = chassis.imu.get_heading();
  const int error = errno;
  const bool imu_installed = chassis.imu.is_installed();
  const int imu_port = chassis.imu.get_port();
  const bool imu_valid = imu_port == 20 && imu_installed && status != pros::ImuStatus::error && !calibrating && sensor_value_ok(heading);
  if (!imu_valid) {
    latch_sensor_fault();
    if (!seen_invalid[2]) { seen_invalid[2] = true; first_invalid[2] = now; }
  }
  const auto internal = chassis.odom_pose_get();
  const auto public_pose = nav_internal_to_game_pose(internal);
  const bool pose_valid = sensor_value_ok(internal.x) && sensor_value_ok(internal.y) && sensor_value_ok(internal.theta);
  if (!pose_valid) latch_sensor_fault();
  std::printf("ODOM ms=%u IMU expected=P20 actual=P%d installed=%d config_ok=%d heading=%.6f valid=%d status=%u calibrating=%d errno=%d first_invalid_seen=%d first_invalid_ms=%u | EZ x=%.6f y=%.6f h=%.6f | PUBLIC x=%.6f y=%.6f h=%.6f pose_valid=%d | fault_latched=%d first_fault_ms=%u\n",
      now, imu_port, imu_installed, imu_port == 20, imu_valid ? heading : NAN, imu_valid, static_cast<unsigned>(status), calibrating, error,
      seen_invalid[2], first_invalid[2], internal.x, internal.y, internal.theta,
      public_pose.x, public_pose.y, public_pose.theta, pose_valid, sensor_fault.load(), first_fault_ms.load());
  std::fflush(stdout);
}
namespace {
bool print_tracker(unsigned index) {
  const auto t = nav_read_tracker(index);
  std::printf("%s expected=P%d actual=P%d attached=%d installed=%d config_ok=%d raw_cd=%.0f distance_in=%.6f valid=%d errno=%d\n",
      t.label, t.expected_port, t.actual_port, nav_diagnostic_tracker(index) != nullptr,
      t.installed, t.configuration_ok, t.raw_cd, t.distance_in, t.valid, t.error);
  if (!t.configuration_ok) std::printf("TRACKER CONFIGURATION MISMATCH: expected attached %s P%d\n", t.label, t.expected_port);
  else if (!t.installed) std::printf("TRACKER MISSING: P%d\n", t.actual_port);
  else if (!t.valid) std::printf("TRACKER INVALID: unreadable position or scale\n");
  return t.valid;
}
}
bool nav_print_sensor_health() {
  std::printf("\n2474V PASSIVE SENSOR HEALTH (no reset or motor commands)\n");
  const bool horizontal = print_tracker(0), vertical = print_tracker(1);
  const int port = chassis.imu.get_port();
  const bool installed = chassis.imu.is_installed();
  const auto status = chassis.imu.get_status();
  const bool calibrating = chassis.imu.is_calibrating();
  errno = 0;
  const double heading = chassis.imu.get_heading(), rotation = chassis.imu.get_rotation();
  const int error = errno;
  const bool imu_valid = port == 20 && installed && status != pros::ImuStatus::error &&
      !calibrating && sensor_value_ok(heading) && sensor_value_ok(rotation);
  std::printf("IMU expected=P20 actual=P%d installed=%d config_ok=%d status=%u calibrating=%d heading=%.6f rotation=%.6f valid=%d errno=%d\n",
      port, installed, port == 20, static_cast<unsigned>(status), calibrating,
      imu_valid ? heading : NAN, imu_valid ? rotation : NAN, imu_valid, error);
  const auto p = nav_pose_get();
  const bool pose_valid = sensor_value_ok(p.x) && sensor_value_ok(p.y) && sensor_value_ok(p.theta);
  std::printf("CURRENT POSE X=%.6f Y=%.6f heading=%.6f pose_valid=%d (not geometry qualification)\n",
      p.x, p.y, p.theta, pose_valid);
  const bool ready = horizontal && vertical && imu_valid && pose_valid;
  if (!ready) latch_sensor_fault();
  std::printf("NAV historical_fault=%d first_fault_ms=%u current_valid=%d (report only)\n",
      sensor_fault.load(), first_fault_ms.load(), ready);
  std::fflush(stdout);
  return ready;
}
void nav_print_snapshot(const char* label) {
  const auto p = chassis.odom_pose_get();
  std::printf("\n=== %s ms=%lu EZ x=%.6f y=%.6f h=%.6f ===\n", label,
      static_cast<unsigned long>(pros::millis()), p.x, p.y, p.theta);
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
