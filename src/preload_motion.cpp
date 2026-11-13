#include "preload_motion.hpp"
#include "navigation.hpp"
#include "nav_sequence.hpp"
#include "nav_diagnostics.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
// One autonomous caller; callbacks run synchronously on that caller, never the motor task.
struct Event { unsigned at; const char* name; const char* result; double angle; };
Event events[32];
unsigned event_count = 0, dropped = 0;
PreloadLiftResult lift_result = PreloadLiftResult::Idle;
int native_xy = -1, native_angle = -1;
struct NavigationReport {
  bool available = false, native_ok = false, interfered = false;
  ez::pose requested{}, final{};
  geometry_turn::Result deadline = geometry_turn::Result::Idle;
  ez::NativeOdomWaitResult wait{};
} navigation_report;
void record(const char* name, const char* result) {
  if (event_count < 32) events[event_count++] = {pros::millis(),name,result,lift_position_deg()};
  else ++dropped;
}
const char* result_name(PreloadLiftResult result) {
  switch (result) {
    case PreloadLiftResult::Idle: return "IDLE";
    case PreloadLiftResult::Moving: return "MOVING";
    case PreloadLiftResult::Reached: return "REACHED_ACTIVE_HOLD";
    case PreloadLiftResult::Timeout: return "TARGET_NOT_REACHED_TIMEOUT_STOPPED";
    case PreloadLiftResult::Cancelled: return "CANCELLED";
    case PreloadLiftResult::Fault: return "SENSOR_MOTOR_OR_CONTROL_FAULT";
    case PreloadLiftResult::LimitViolation: return "PHYSICAL_LIMIT_VIOLATION";
    default: return "REJECTED";
  }
}
bool healthy(PreloadLiftResult result) {
  return result == PreloadLiftResult::Moving || result == PreloadLiftResult::Reached ||
         result == PreloadLiftResult::Timeout;
}
void update_lift(const char* phase) {
  const auto next = lift_preload_update();
  if (next != lift_result) {
    record(phase,result_name(next));
    if (next != PreloadLiftResult::Moving && next != PreloadLiftResult::Reached)
      record("LIFT_DETAIL",lift_auton_last_reason());
  }
  lift_result = next;
}
bool sensors_ready() { return chassis.odom_enabled() && nav_sensors_ready(); }
bool service(ez::Drive* drive) {
  if (drive != &chassis) return true;
  update_lift("UP");
  // A stopped target shortfall does not cancel healthy navigation.
  return healthy(lift_result);
}
void observe_exit(ez::Drive* drive, int xy, int angle) {
  if (drive == &chassis) { native_xy = xy; native_angle = angle; }
}
bool normal_exit(int value) { return value == ez::SMALL_EXIT || value == ez::BIG_EXIT; }
const char* exit_name(int value) {
  switch(value) {
    case ez::RUNNING: return "RUNNING";
    case ez::SMALL_EXIT: return "SMALL";
    case ez::BIG_EXIT: return "BIG";
    case ez::VELOCITY_EXIT: return "VELOCITY";
    case ez::mA_EXIT: return "OVERCURRENT";
    case -1: return "UNOBSERVED";
    default: return "ERROR";
  }
}
// No NavSequence: its finish() would impose a rest dwell and stop the lift.
struct NavigationCleanup {
  bool print_before = chassis.pid_print_toggle_get();
  bool keep_lift_hold = false;
  ~NavigationCleanup() {
    ez::drive_stop(&chassis);
    ez::wait_service_set(nullptr);
    ez::native_odom_exit_observer_set(nullptr);
    chassis.pid_print_toggle(print_before);
    if (!keep_lift_hold && !lift_auton_stop()) record("CLEANUP","LIFT_STOP_WRITE_FAILED");
  }
};
}

bool lift_boomerang_to(double lift_deg, double x, double y, double heading, int speed) {
  event_count = dropped = 0; navigation_report = {};
  native_xy = native_angle = -1;
  lift_result = PreloadLiftResult::Idle;
  if (nav_sequence_active() || ez::geometry_turn_deadline_active(&chassis)) {
    record("COMBINED","NAVIGATION_ALREADY_OWNED");
    ez::autonomous_emergency_stop(); ez::drive_stop(&chassis); lift_auton_stop();
    return false;
  }
  NavigationCleanup cleanup;
  ez::drive_stop(&chassis);
  if (ez::autonomous_cancelled() || !sensors_ready() ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading) ||
      speed < 1 || speed > 127 || !chassis.pid_drive_toggle_get()) {
    record("COMBINED","CANCEL_SENSOR_ARGUMENT_OR_OUTPUT_ENABLE"); return false;
  }
  chassis.interfered = false;
  chassis.pid_print_toggle(false);
  if (!ez::geometry_turn_deadline_arm(&chassis,sensors_ready)) {
    record("NAV","DEADLINE_ARM_REJECTED"); return false;
  }
  if (!lift_preload_start(lift_deg,mechanism_config::TARGET_TIMEOUT_MS)) {
    record("UP_START",lift_auton_last_reason()); return false;
  }
  record("UP_START","ACCEPTED");
  ez::wait_service_set(service);
  ez::native_odom_exit_observer_set(observe_exit);
  record("BOOMERANG_START","DISPATCH");
  const bool moved = nav_go_to_pose(x,y,heading,speed,NavEndMode::SETTLE);
  ez::drive_stop(&chassis); // No extra rest dwell; native wait has returned.
  const auto deadline = ez::geometry_turn_deadline_result();
  ez::wait_service_set(nullptr);
  ez::native_odom_exit_observer_set(nullptr);
  const auto native = ez::native_odom_wait_result(&chassis);
  const bool nav_success = moved && !ez::autonomous_cancelled() && !chassis.interfered &&
      deadline == geometry_turn::Result::Returned && sensors_ready() &&
      native.completed && normal_exit(native.xy) && normal_exit(native.angular) && healthy(lift_result);
  navigation_report = {true,nav_success,chassis.interfered,{x,y,heading},nav_pose_get(),deadline,native};
  record("BOOMERANG_FINISH",nav_success ? "NATIVE_COMPLETE_STOPPED" : "FAILED_STOPPED");
  if (!nav_success) {
    if (ez::autonomous_cancelled()) record("UP_ABORT","CANCELLED");
    else record("UP_ABORT","NAVIGATION_FAILURE");
    return false;
  }
  // The original target_started timestamp is unchanged. If navigation finishes first,
  // service only the remaining lift movement time; do not restart or re-command it.
  update_lift("UP");
  while (lift_result == PreloadLiftResult::Moving) {
    pros::delay(mechanism_config::UPDATE_PERIOD_MS);
    update_lift("UP");
  }
  if (lift_result != PreloadLiftResult::Reached) return false;
  cleanup.keep_lift_hold = true; // Caller immediately commands descent on this same task.
  return true;
}

bool preload_lower_for_release(double target_deg, unsigned timeout_ms) {
  // An already accepted target region needs no movement; otherwise require descent.
  const double position = lift_position_deg();
  if (!std::isfinite(position) || !std::isfinite(target_deg) || target_deg - position > mechanism_config::LIFT_TARGET_TOLERANCE_DEG) {
    lift_auton_stop(); record("DOWN_START","INVALID_FEEDBACK_OR_NOT_A_DESCENT"); return false;
  }
  if (!lift_preload_start(target_deg,timeout_ms)) {
    record("DOWN_START",lift_auton_last_reason()); return false;
  }
  record("DOWN_START","ACCEPTED");
  lift_result = PreloadLiftResult::Idle;
  update_lift("DOWN");
  while (lift_result == PreloadLiftResult::Moving) {
    pros::delay(mechanism_config::UPDATE_PERIOD_MS);
    update_lift("DOWN");
  }
  const bool stopped = lift_auton_stop();
  const double final = lift_position_deg();
  const bool success = lift_result == PreloadLiftResult::Reached && stopped &&
      !ez::autonomous_cancelled() && lift_target_ready() && std::isfinite(final) &&
      final >= mechanism_config::LIFT_MIN_DEG && final <= mechanism_config::LIFT_MAX_DEG;
  record("DOWN_FINISH",success ? "TARGET_REACHED_STOPPED_RELEASE_ALLOWED" : "FAILED_NO_RELEASE");
  return success;
}

void preload_log_flush(unsigned autonomous_started_ms) {
  // A disabled callback can terminate an earlier autonomous before its final log flush.
  // Do not attribute that older buffer to a new run that failed before combined motion.
  if (event_count && static_cast<std::int32_t>(events[0].at-autonomous_started_ms) < 0) {
    event_count = dropped = 0; navigation_report = {};
  }
  for (unsigned i=0;i<event_count;++i)
    std::printf("PRELOAD t=%u event=%s result=%s p16=%.3f\n",
                events[i].at,events[i].name,events[i].result,events[i].angle);
  if (navigation_report.available) {
    const auto& r = navigation_report;
    const double xy = std::hypot(r.requested.x-r.final.x,r.requested.y-r.final.y);
    const double heading = std::remainder(r.requested.theta-r.final.theta,360.0);
    std::printf("PRELOAD NAV native_ok=%d wait_completed=%d native_xy=%s native_angle=%s observer_xy=%s observer_angle=%s deadline=%d interfered=%d "
                "requested=(%.3f,%.3f,%.3f) final=(%.3f,%.3f,%.3f) xy_error=%.3f heading_error=%+.3f "
                "xy_diagnostic_ok=%d heading_diagnostic_ok=%d [accuracy is report-only]\n",
                r.native_ok,r.wait.completed,exit_name(r.wait.xy),exit_name(r.wait.angular),
                exit_name(native_xy),exit_name(native_angle),static_cast<int>(r.deadline),r.interfered,
                r.requested.x,r.requested.y,r.requested.theta,r.final.x,r.final.y,r.final.theta,
                xy,heading,std::isfinite(xy) && xy <= preload_config::xy_tolerance_in,
                std::isfinite(heading) && std::fabs(heading) <= preload_config::heading_tolerance_deg);
  }
  std::printf("PRELOAD dropped_events=%u\n",dropped);
  event_count = dropped = 0; navigation_report = {};
}
void preload_auton_finish_log(unsigned began, bool released) {
  const unsigned finished = pros::millis();
  preload_log_flush(began);
  std::printf("PRELOAD start_t=%u finish_t=%u released=%d total_ms=%u\n",
              began, finished, released, finished-began);
}
