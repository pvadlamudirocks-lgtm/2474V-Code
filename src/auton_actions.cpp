#include "auton_actions.hpp"
#include "nav_diagnostics.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <cstdio>

namespace auton_actions {
namespace {
bool speed_ok(int speed) { return speed >= 1 && speed <= 127; }
bool heading_set(std::optional<double> heading) {
  return heading && *heading != ez::ANGLE_NOT_SET;
}
bool point_ok(RoutePoint p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && speed_ok(p.speed) &&
         (!p.heading || std::isfinite(*p.heading));
}
bool direction_ok(nav_path_direction direction) {
  return direction == nav_path_direction::forward || direction == nav_path_direction::reverse;
}
const char* label(const char* value) { return value ? value : "unnamed"; }
ActionResult reject(Status status, Context c) {
  std::printf("AUTON %s / %s: %s (no action dispatched)\n",
              label(c.route), label(c.step), status_name(status));
  return {status};
}
bool cancelled() {
  return pros::competition::is_disabled() || !pros::competition::is_autonomous();
}
struct Record {
  ez::pose start, target, final;
  unsigned elapsed;
  const char* step;
  const char* phase;
  bool interfered;
};
void print_record(const Record& r, Context c) {
  if (c.telemetry)
    nav_print_action_record(label(c.route), label(r.step), r.start, r.target,
                            r.final, r.elapsed, r.phase, r.interfered);
}
// ----- Shared action result handling -----
// Capture before/after poses and stop on a rejected or interrupted movement.
template <typename Call>
ActionResult movement(ez::pose target, Context c, Call call) {
  if (cancelled()) return reject(Status::Cancelled, c);
  const auto start = nav_pose_get();
  const auto began = pros::millis();
  const bool accepted = call();
  const unsigned elapsed = pros::millis() - began;
  const auto final = nav_pose_get();
  Status status = Status::ReturnedUnverified;
  if (cancelled()) status = Status::Cancelled;
  else if (!accepted || chassis.interfered || !std::isfinite(final.x) ||
           !std::isfinite(final.y) || !std::isfinite(final.theta)) status = Status::Failed;
  if (status != Status::ReturnedUnverified) {
    nav_cancel_sequence(); // Existing safe stop; no substitute controller or retry.
    stop_all_mechanisms();
    std::printf("AUTON %s / %s: %s\n", label(c.route), label(c.step), status_name(status));
  }
  print_record({start, target, final, elapsed, c.step, status_name(status), chassis.interfered}, c);
  return {status, elapsed};
}
ActionResult callback(const MechanismAction& action, bool enabled, Context c) {
  if (!enabled) return reject(Status::Disabled, c);
  if (!action) return reject(Status::InvalidRequest, c);
  if (cancelled()) return reject(Status::Cancelled, c);
  const auto began = pros::millis();
  const bool accepted = action(); // Caller supplies a short existing subsystem API call.
  const auto status = cancelled() ? Status::Cancelled :
      accepted ? Status::ReturnedUnverified : Status::Failed;
  if (status == Status::Failed || status == Status::Cancelled) {
    nav_cancel_sequence();
    stop_all_mechanisms();
  }
  const unsigned elapsed = pros::millis() - began;
  if (c.telemetry || status != Status::ReturnedUnverified)
    std::printf("AUTON %s / %s: %s, %u ms\n", label(c.route), label(c.step), status_name(status), elapsed);
  return {status, elapsed};
}
}
const char* status_name(Status s) {
  switch (s) {
    case Status::ReturnedUnverified: return "RETURNED; ARRIVAL/COMPLETION UNVERIFIED";
    case Status::SequenceComplete: return "SEQUENCE FINISHED (existing acceptance checks)";
    case Status::Disabled: return "FEATURE DISABLED";
    case Status::InvalidRequest: return "INVALID REQUEST";
    case Status::Failed: return "FAILED";
    case Status::Cancelled: return "CANCELLED";
    case Status::Unconfigured: return "NOT CONFIGURED";
  }
  return "UNKNOWN";
}
ActionResult drive_to_point(FieldPoint p, int speed, Context c) {
  if (!point_ok({p.x,p.y,speed})) return reject(Status::InvalidRequest,c);
  return movement({p.x,p.y,ez::ANGLE_NOT_SET},c,[=] { nav_go_to(p.x,p.y,speed); return true; });
}
ActionResult drive_to_point_reverse(FieldPoint p, int speed, Context c) {
  if (!point_ok({p.x,p.y,speed})) return reject(Status::InvalidRequest,c);
  return movement({p.x,p.y,ez::ANGLE_NOT_SET},c,[=] { nav_go_to_reverse(p.x,p.y,speed); return true; });
}
ActionResult drive_to_pose(RoutePoint p, Context c) {
  if (!point_ok(p) || !heading_set(p.heading)) return reject(Status::InvalidRequest,c);
  return movement({p.x,p.y,*p.heading},c,[=] { nav_go_to_pose(p.x,p.y,*p.heading,p.speed); return true; });
}
ActionResult turn_to_heading(double heading, int speed, Context c) {
  if (!std::isfinite(heading) || heading == ez::ANGLE_NOT_SET || !speed_ok(speed))
    return reject(Status::InvalidRequest,c);
  const auto p = nav_pose_get();
  return movement({p.x,p.y,heading},c,[=] { nav_turn_to(heading,speed); return true; });
}
ActionResult approach_goal(FieldPoint p, int speed, Context c) {
  return drive_to_point(p,speed,c); // No wall approach, correction or mechanism side effects.
}
ActionResult follow_path(const std::vector<nav_waypoint>& points, int speed,
                         std::optional<double> heading, nav_path_direction direction, Context c) {
  if (!ENABLE_ADVANCED_PATHS) return reject(Status::Disabled,c);
  if (points.empty() || !speed_ok(speed) || !direction_ok(direction) ||
      (heading && !std::isfinite(*heading))) return reject(Status::InvalidRequest,c);
  for (const auto& p : points)
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return reject(Status::InvalidRequest,c);
  return movement({points.back().x,points.back().y,heading_set(heading) ? *heading : ez::ANGLE_NOT_SET},c,[&] {
    return heading_set(heading) ? nav_follow_path_pose(points,*heading,speed,direction)
                                : nav_follow_path(points,speed,direction);
  });
}
// ----- Multi-step actions -----
// Check the whole request before moving; NavSequence owns the actual handoffs.
ActionResult run_sequence(const std::vector<SequenceStep>& steps, Context c) {
  if (!ENABLE_ADVANCED_PATHS) return reject(Status::Disabled,c);
  if (steps.empty() || steps.back().finish != NavEndMode::SETTLE)
    return reject(Status::InvalidRequest,c);
  bool previous_chain = false;
  for (const auto& s : steps) {
    const bool hooks = s.hooks.before_move || s.hooks.arrival || !s.hooks.during_move.empty();
    if (hooks && !ENABLE_MECHANISM_ACTIONS) return reject(Status::Disabled,c);
    if (!point_ok(s.target) || !direction_ok(s.direction) ||
        (s.finish != NavEndMode::SETTLE && s.finish != NavEndMode::CHAIN) ||
        (s.finish == NavEndMode::CHAIN && (heading_set(s.target.heading) || s.hooks.arrival)) ||
        (previous_chain && (s.hooks.before_move || !s.hooks.during_move.empty())))
      return reject(Status::InvalidRequest,c);
    for (const auto& e : s.hooks.during_move)
      if (!e.when || !e.action) return reject(Status::InvalidRequest,c);
    previous_chain = s.finish == NavEndMode::CHAIN;
  }
  if (cancelled()) return reject(Status::Cancelled,c);
  std::vector<Record> records;
  if (c.telemetry) records.reserve(steps.size()); // Allocate before starting any movement.
  const auto began = pros::millis();
  NavSequence sequence; // Existing ownership, mechanism servicing, deadlines and cleanup.
  bool ok = sequence.ok();
  for (const auto& s : steps) {
    if (!ok) break;
    if (cancelled()) { nav_abort("route cancelled"); ok = false; break; }
    if (s.hooks.before_move && !s.hooks.before_move()) {
      nav_abort("before-move action rejected"); ok = false; break;
    }
    if (cancelled()) { nav_abort("route cancelled"); ok = false; break; }
    if (!s.hooks.during_move.empty() && !nav_events(s.hooks.during_move)) { ok = false; break; }
    const auto start = nav_pose_get();
    const auto move_began = pros::millis();
    const auto& p = s.target;
    const bool reverse = s.direction == nav_path_direction::reverse;
    if (heading_set(p.heading))
      ok = reverse ? nav_go_to_pose_reverse(p.x,p.y,*p.heading,p.speed,s.finish)
                   : nav_go_to_pose(p.x,p.y,*p.heading,p.speed,s.finish);
    else
      ok = reverse ? nav_go_to_reverse(p.x,p.y,p.speed,s.finish)
                   : nav_go_to(p.x,p.y,p.speed,s.finish);
    const unsigned elapsed = pros::millis() - move_began;
    if (c.telemetry) records.push_back({start,{p.x,p.y,heading_set(p.heading) ? *p.heading : ez::ANGLE_NOT_SET},
        nav_pose_get(),elapsed,s.name,!ok ? "FAILED" : s.finish == NavEndMode::CHAIN ? "CHAIN RELEASE" : "SETTLE RETURN",chassis.interfered});
    if (cancelled()) { nav_abort("route cancelled"); ok = false; break; }
    if (ok && s.hooks.arrival && !s.hooks.arrival()) {
      nav_abort("arrival action rejected"); ok = false;
    }
  }
  const bool finished = sequence.finish(); // Always finish, even after failure.
  const unsigned elapsed = pros::millis() - began;
  const auto status = cancelled() ? Status::Cancelled : ok && finished ? Status::SequenceComplete : Status::Failed;
  for (const auto& record : records) print_record(record,c);
  if (c.telemetry || status != Status::SequenceComplete)
    std::printf("AUTON %s / %s: %s, %u ms; nav fault: %s\n",
                label(c.route),label(c.step),status_name(status),elapsed,nav_fault());
  return {status,elapsed};
}
ActionResult mechanism_action(const MechanismAction& action, Context c) {
  return callback(action,ENABLE_MECHANISM_ACTIONS,c);
}
ActionResult correct_pose_from_wall(const MechanismAction& correction, Context c) {
  return callback(correction,ENABLE_DISTANCE_CORRECTION,c);
}
ActionResult collect_object(int power, Context c) {
  if (!speed_ok(power)) return reject(Status::InvalidRequest,c);
  return mechanism_action([=] { return pickup_capture(power); },c);
}
ActionResult score_object(int power, Context c) {
  if (!speed_ok(power)) return reject(Status::InvalidRequest,c);
  return mechanism_action([=] { return claw_score(power); },c);
}
namespace {
// Wait for a calibrated lift move to succeed before releasing the object.
ActionResult score_preset(double target, unsigned timeout, int power, Context c) {
  if (!mechanism_config::LIFT_SCORING_POSITIONS_CALIBRATED)
    return reject(Status::Unconfigured,c);
  if (!timeout || !speed_ok(power)) return reject(Status::InvalidRequest,c);
  return mechanism_action([=] {
    return lift_auton_move_to_deg(target,timeout) && claw_score(power);
  },c);
}
}
ActionResult score_high(unsigned timeout, int power, Context c) {
  return score_preset(mechanism_config::LIFT_HIGH_DEG,timeout,power,c);
}
ActionResult score_low(unsigned timeout, int power, Context c) {
  return score_preset(mechanism_config::LIFT_LOW_DEG,timeout,power,c);
}
ActionResult collect_object(Context c) {
  if (!speed_ok(mechanism_config::FLEX_INTAKE_POWER)) return reject(Status::Unconfigured,c);
  return collect_object(mechanism_config::FLEX_INTAKE_POWER,c);
}
ActionResult score_object(Context c) {
  if (!speed_ok(mechanism_config::FLEX_OUTTAKE_POWER)) return reject(Status::Unconfigured,c);
  return score_object(mechanism_config::FLEX_OUTTAKE_POWER,c);
}
ActionResult score_high(Context c) { return reject(Status::Unconfigured,c); }
ActionResult score_low(Context c) { return reject(Status::Unconfigured,c); }
ActionResult cancel_route() {
  nav_cancel_sequence();
  stop_all_mechanisms();
  return {Status::Cancelled};
}
std::optional<RoutePoint> mirror_x(RoutePoint p, double axis_x) {
  if (!point_ok(p) || !std::isfinite(axis_x)) return std::nullopt;
  p.x = mirror_x(FieldPoint{p.x,p.y},axis_x).x;
  if (!std::isfinite(p.x)) return std::nullopt;
  if (heading_set(p.heading)) {
    double h = std::fmod(*p.heading,360.0);
    if (h < 0) h += 360.0;
    p.heading = mirror_normalized_heading(h);
  }
  return p;
}
// Compile-time checks exercise cardinal headings, a nonzero axis and involution.
static_assert(mirror_x(FieldPoint{3,7},10).x == 17);
static_assert(mirror_x(mirror_x(FieldPoint{3,7},10),10).x == 3);
static_assert(mirror_x(FieldPoint{3,7},10).y == 7);
static_assert(mirror_normalized_heading(0) == 0);
static_assert(mirror_normalized_heading(90) == 270);
static_assert(mirror_normalized_heading(180) == 180);
static_assert(mirror_normalized_heading(270) == 90);
void run_helper_math_test() {
  const auto a = mirror_x(RoutePoint{3,7,40,450.0},10);
  const auto b = a ? mirror_x(*a,10) : std::nullopt;
  const auto xy = mirror_x(RoutePoint{3,7,40},10);
  const auto sentinel = mirror_x(RoutePoint{3,7,40,ez::ANGLE_NOT_SET},10);
  const bool ok = a && b && a->x == 17 && a->heading == 270 && b->x == 3 && b->heading == 90 &&
      xy && !xy->heading && sentinel && sentinel->heading == ez::ANGLE_NOT_SET &&
      !mirror_x(RoutePoint{NAN,7,40},10) && !mirror_x(RoutePoint{3,7,40},INFINITY);
  std::printf("MIRROR MATH: %s (mathematics only; no robot test)\n",ok ? "PASS" : "FAIL");
}
} // namespace auton_actions




