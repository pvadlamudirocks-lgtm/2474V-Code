#include "pinnacle_auton.hpp"
#include "subsystems.hpp"
#include "lift_calibration.hpp"
#include "nav_sequence.hpp"
#include "nav_diagnostics.hpp"
#include "preload_motion.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <cmath>
#include <cstdio>

namespace {
bool budget_valid(unsigned ms) { return ms > 0 && ms < 0x80000000u; }
bool angle_valid(double p) {
  return std::isfinite(p) && p >= mechanism_config::LIFT_MIN_DEG &&
      p <= mechanism_config::LIFT_MAX_DEG;
}
bool lift_position_matches(double target) {
  const double actual = lift_position_deg();
  return lift_target_ready() && angle_valid(actual) && angle_valid(target) &&
      std::fabs(target-actual) <= mechanism_config::LIFT_TARGET_TOLERANCE_DEG;
}
bool sensors_ready() { return chassis.odom_enabled() && nav_sensors_ready(); }
bool normal_exit(int result) { return result == ez::SMALL_EXIT || result == ez::BIG_EXIT; }
bool at_pose(double x,double y,double h) {
  const auto p=nav_pose_get();
  return sensors_ready() && std::isfinite(x) && std::isfinite(y) && std::isfinite(h) &&
      std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta) &&
      std::hypot(x-p.x,y-p.y) <= preload_config::xy_tolerance_in &&
      std::fabs(std::remainder(h-p.theta,360.0)) <= preload_config::heading_tolerance_deg;
}
bool no_navigation() {
  return !nav_sequence_active() && !ez::geometry_turn_deadline_active(&chassis) &&
      chassis.drive_mode_get() == ez::DISABLE;
}
bool reject(const char* reason) { auton_safe_abort(reason); return false; }

// Observes the actual normal controller, including its original command deadline.
// No motor output, telemetry printing, allocations or alternative PID here.
struct LiftMonitor {
  LiftAutonTargetStatus initial;
  unsigned first_settled_ms = 0;
  bool saw_settled = false;
  const char* failure = nullptr;
  bool check() {
    const auto s = lift_auton_target_status();
    const unsigned now=pros::millis();
    if (ez::autonomous_cancelled()) failure="CANCELLED_OR_MODE";
    else if (!s.ready || !angle_valid(s.position_deg)) failure="LIFT_FEEDBACK_OR_LIMIT_FAULT";
    else if (!s.active) failure=s.reason;
    else if (s.started_ms != initial.started_ms || s.target_deg != initial.target_deg ||
             s.timeout_ms != initial.timeout_ms) failure="LIFT_TARGET_REPLACED";
    else if ((!s.settled || !saw_settled) && now-initial.started_ms >= initial.timeout_ms) failure="LIFT_TARGET_TIMEOUT";
    if (failure) return false;
    if (s.settled && !saw_settled) { saw_settled=true; first_settled_ms=now; }
    return true;
  }
};
bool wait_lift(LiftMonitor& monitor,unsigned wait_budget) {
  const unsigned began=pros::millis();
  while (true) {
    if (!monitor.check()) return false; // Cancellation/deadline before any next output.
    mechanisms_update();
    if (!monitor.check()) return false;
    if (lift_at_target()) return true;
    if (pros::millis()-began >= wait_budget) { monitor.failure="LIFT_WAIT_TIMEOUT"; return false; }
    pros::delay(mechanism_config::UPDATE_PERIOD_MS);
  }
}
void lift_report(const char* command,unsigned began,double start,double target,unsigned timeout,bool ok) {
  std::printf("PINNACLE command=%s start_t=%u finish_t=%u elapsed_ms=%u start_p16=%.3f target=%.3f final_p16=%.3f timeout_ms=%u success=%d detail=%s\n",
      command,began,pros::millis(),pros::millis()-began,start,target,lift_position_deg(),timeout,ok,lift_auton_last_reason());
}
// Existing bounded rest observation; no navigation dispatch and no new motor owner.
bool verify_stopped_pose(double x,double y,double h) {
  NavSequence rest(false,geometry_turn::maximum_powered_ms);
  const bool finished=rest.finish();
  return finished && nav_sequence_rest_verified() && !ez::autonomous_cancelled() && at_pose(x,y,h);
}
}

void auton_safe_abort(const char* reason) {
  ez::autonomous_emergency_stop();
  nav_abort(reason ? reason : "PINNACLE_ABORT"); // Existing serialized drive/mechanism stops; never opens claw.
  std::printf("PINNACLE ABORT t=%u reason=%s lift_detail=%s [stop requested, physical rest not implied]\n",
      pros::millis(),reason?reason:"PINNACLE_ABORT",lift_auton_last_reason());
}

bool auton_boomerang_with_lift(double x,double y,double h,int speed,double lift_target,
    unsigned lift_timeout,unsigned drive_timeout,PinnacleArrival* arrival) {
  if (arrival) arrival->valid_=false;
  if (!no_navigation()) return reject("NAVIGATION_ALREADY_ACTIVE");
  if (!lift_calibration::hold_validated) return reject("LIFT_HOLD_NOT_PHYSICALLY_QUALIFIED");
  if (ez::autonomous_cancelled() || !sensors_ready() || !chassis.pid_drive_toggle_get())
    return reject("MODE_SENSOR_OR_DRIVE_ENABLE");
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(h) || speed<1 || speed>127 ||
      !angle_valid(lift_target) || !budget_valid(lift_timeout) || !budget_valid(drive_timeout) ||
      drive_timeout>geometry_turn::maximum_powered_ms) return reject("INVALID_ARGUMENT_OR_DEADLINE");
  const unsigned began=pros::millis();
  const double start_lift=lift_position_deg();
  NavSequence sequence(false,drive_timeout); // Installs existing same-caller mechanism service.
  if (!sequence.ok() || !lift_auton_stop() || !lift_auton_target_start(lift_target,lift_timeout)) {
    const char* why=lift_auton_last_reason();
    auton_safe_abort(why); sequence.finish(); return false;
  }
  LiftMonitor monitor{lift_auton_target_status()};
  // Optional failure event inspects after service() has called mechanisms_update().
  // No replacement wait hook, background task, or competing lift controller.
  if (!nav_events({{[&](const NavProgress&) { return !monitor.check(); },
                   [] { return false; },false}}) ||
      !ez::geometry_turn_deadline_arm(&chassis,sensors_ready,speed)) {
    auton_safe_abort("EVENT_OR_WATCHDOG_ARM_FAILED"); sequence.finish(); return false;
  }
  const unsigned nav_start=pros::millis();
  const bool moved=nav_go_to_pose(x,y,h,speed,NavEndMode::SETTLE);
  const unsigned nav_end=pros::millis();
  ez::drive_stop(&chassis); // Stops/disarms native PID BEFORE lift wait or reporting.
  const auto deadline=ez::geometry_turn_deadline_result();
  const auto native=ez::native_odom_wait_result(&chassis);
  bool ok=moved && nav_ok() && !chassis.interfered && !ez::autonomous_cancelled() &&
      nav_end-nav_start<drive_timeout && deadline==geometry_turn::Result::Returned &&
      native.completed && normal_exit(native.xy) && normal_exit(native.angular) && sensors_ready();
  if (ok) ok=wait_lift(monitor,lift_timeout); // Original target start/timeout still enforced.
  if (!ok) auton_safe_abort(monitor.failure?monitor.failure:"NAVIGATION_OR_NATIVE_EXIT_FAILED");
  const bool passed_before_rest=ok;
  const bool settled=ok && lift_at_target();
  // Existing rest service continues holding during its bounded rest check.
  const bool finished=sequence.finish();
  const bool stopped=lift_auton_stop();
  ok=ok && settled && finished && stopped && nav_sequence_rest_verified() &&
      !ez::autonomous_cancelled() && at_pose(x,y,h) && lift_position_matches(lift_target);
  if (!ok && passed_before_rest) auton_safe_abort("COMBINED_ARRIVAL_LIFT_OR_REST_FAILED");
  if (ok && arrival) {
    arrival->x_=x; arrival->y_=y; arrival->heading_=h;
    arrival->owner_=pros::c::task_get_current(); arrival->valid_=true;
  }
  const auto p=nav_pose_get();
  std::printf("PINNACLE BOOM start_t=%u complete_t=%u finish_t=%u drive_timeout_ms=%u native_xy=%d native_angle=%d deadline=%d rest=%d target=(%.3f,%.3f,%.3f) final=(%.3f,%.3f,%.3f) xy_error=%.3f heading_error=%+.3f success=%d\n",
      nav_start,nav_end,pros::millis(),drive_timeout,native.xy,native.angular,static_cast<int>(deadline),nav_sequence_rest_verified(),x,y,h,p.x,p.y,p.theta,std::hypot(x-p.x,y-p.y),std::remainder(h-p.theta,360.0),ok);
  std::printf("PINNACLE LIFT command_t=%u settled_observed=%d first_settled_t=%u time_to_settle_ms=%u [command-to-settle, not measured motor-onset]\n",
      monitor.initial.started_ms,monitor.saw_settled,monitor.first_settled_ms,
      monitor.saw_settled?monitor.first_settled_ms-monitor.initial.started_ms:0);
  lift_report("BOOM_WITH_LIFT",began,start_lift,lift_target,lift_timeout,ok);
  return ok;
}

bool auton_wait_for_lift(double target,unsigned timeout) {
  if (!no_navigation() || !angle_valid(target) || !budget_valid(timeout)) return reject("INVALID_LIFT_WAIT");
  const auto status=lift_auton_target_status();
  if (!status.active || status.target_deg!=target) return reject("NO_MATCHING_NORMAL_LIFT_TARGET");
  const unsigned began=pros::millis();
  LiftMonitor monitor{status};
  // An already-settled live target needs no additional settling delay.
  monitor.saw_settled=status.settled; monitor.first_settled_ms=status.settled?began:0;
  bool ok=wait_lift(monitor,timeout);
  const bool stopped=lift_auton_stop();
  ok=ok && stopped && !ez::autonomous_cancelled() && lift_position_matches(target);
  if (!ok) auton_safe_abort(monitor.failure?monitor.failure:"LIFT_STOP_OR_FINAL_POSITION_FAILED");
  lift_report("WAIT_LIFT",began,status.position_deg,target,timeout,ok);
  return ok;
}

bool auton_lift_to_intake(unsigned timeout) {
  if (!no_navigation() || !budget_valid(timeout)) return reject("INVALID_INTAKE_CONTEXT_OR_TIMEOUT");
  const auto preset=lift_calibration::positions[static_cast<unsigned>(lift_calibration::Position::INTAKE)];
  const unsigned began=pros::millis(); const double start=lift_position_deg();
  const bool ok=preset.configured &&
      lift_move_calibrated_position(static_cast<unsigned>(lift_calibration::Position::INTAKE),timeout,LiftFinish::STOP) &&
      !ez::autonomous_cancelled() && lift_position_matches(preset.degrees);
  if (!ok) auton_safe_abort("INTAKE_POSITION_FAILED");
  lift_report("INTAKE",began,start,preset.degrees,timeout,ok);
  return ok;
}

bool auton_claw_close_with_toggle(unsigned pulse_ms) {
  if (!no_navigation() || !pinnacle_config::claw_toggle_overlap_verified || !budget_valid(pulse_ms) ||
      ez::autonomous_cancelled() || !toggle_connected()) return reject("CLAW_TOGGLE_QUALIFICATION_MODE_OR_CONNECTION");
  // No live lift target is allowed to lose servicing during this blocking pulse.
  if (!lift_auton_stop()) return reject("LIFT_STOP_FAILED_BEFORE_TOGGLE");
  const unsigned began=pros::millis();
  const bool closed=claw_close();
  const bool pulse=closed && pulse_toggle(pulse_ms); // No pneumatic dwell.
  // pulse_toggle always stops; if closing failed, stop without starting the pulse.
  const bool stopped=closed ? pulse : toggle_stop();
  const bool ok=closed && pulse && stopped && !ez::autonomous_cancelled();
  if (!ok) auton_safe_abort("CLAW_OR_TOGGLE_COMMAND_FAILED");
  std::printf("PINNACLE CLAW_TOGGLE start_t=%u finish_t=%u close_write=%d pulse_ms=%u pulse_success_and_stop=%d success=%d [not grip feedback]\n",
      began,pros::millis(),closed,pulse_ms,stopped,ok);
  return ok;
}

bool auton_score_at_goal(PinnacleArrival& arrival,double target,unsigned timeout,
    bool descent_clearance_verified,bool object_secured_verified) {
  const bool valid=arrival.valid_ && arrival.owner_==pros::c::task_get_current();
  arrival.valid_=false; // Single use; a failed attempt cannot later release an object.
  if (!valid || !no_navigation() || chassis.interfered || ez::autonomous_cancelled() || !budget_valid(timeout) ||
      !angle_valid(target) || !descent_clearance_verified || !object_secured_verified ||
      !claw_commanded_closed()) return reject("SCORING_EVIDENCE_OR_ARGUMENT_MISSING");
  if (!verify_stopped_pose(arrival.x_,arrival.y_,arrival.heading_)) return reject("GOAL_POSE_OR_REST_FAILED");
  const double start=lift_position_deg(); const unsigned began=pros::millis();
  if (!angle_valid(start) || target-start>mechanism_config::LIFT_TARGET_TOLERANCE_DEG)
    return reject("INVALID_FEEDBACK_OR_NOT_DESCENT");
  bool ok=lift_auton_move_to_deg(target,timeout,LiftFinish::STOP) && lift_position_matches(target);
  if (ok) ok=verify_stopped_pose(arrival.x_,arrival.y_,arrival.heading_) &&
      lift_position_matches(target) && claw_commanded_closed() && !ez::autonomous_cancelled();
  const bool released=ok && claw_open(); // The only release; never in an abort/cleanup.
  if (!released) auton_safe_abort("SCORE_POSITION_REST_OR_RELEASE_FAILED");
  lift_report("SCORE",began,start,target,timeout,released);
  return released;
}

bool auton_acquire_from_loader(const PinnacleLoaderPlan&) { return reject("LOADER_ROUTE_NOT_QUALIFIED"); }
bool auton_run_pinnacle_route(const PinnacleRoutePlan&) { return reject("PINNACLE_ROUTE_INACTIVE"); }
