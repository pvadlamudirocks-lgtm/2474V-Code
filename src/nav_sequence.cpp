#include "nav_sequence.hpp"
#include "subsystems.hpp"
#include "nav_diagnostics.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

namespace {
// ----- Sequence ownership and saved motion state -----
// One autonomous task owns a sequence; a pending CHAIN still has motion to finish.
bool active = false, waiting = false, pending = false, busy = false, tracing = false;
bool print_before = false, encoder_motion = false, turn_motion = false;
const char* fault = nullptr;
unsigned timeout_ms = 8000, started = 0, sequence_start = 0, movement = 0;
ez::pose target{}, previous_pose{};
double traveled = 0, left_target = 0, right_target = 0, turn_target = 0;
std::vector<NavEvent> queued, events;
std::vector<bool> fired;
std::atomic<int> command_left{0}, command_right{0};
std::atomic<unsigned> command_ms{0};
std::atomic<int> peak_left_command{0}, peak_right_command{0};
std::atomic<unsigned> command_samples{0};
int motion_requested_speed = 0;
bool motion_slew_requested = false;
enum Flags : unsigned { START=1, RELEASE=2, SETTLED=4, ZERO=8, REVERSAL=16,
  HEADING=32, FAILURE=64, ACTION=128, FINISH=256 };
struct Row { unsigned ms, motion, flags, command_time; ez::pose pose;
  double error, heading_error, left_rpm, right_rpm; int left, right;
  int requested_speed = 0, peak_left = 0, peak_right = 0;
  unsigned commands = 0;
  bool slew_requested = false, slew_active = false;
  double slew_limit = 0, xy_output = 0, angular_output = 0, control_heading_error = 0; };
void capture_controller(Row& row);
constexpr unsigned CAPACITY = 3000;
Row rows[CAPACITY];
unsigned row_count = 0, dropped = 0, zero_events = 0;
bool was_zero = true, rest_verified = false;
int left_sign = 0, right_sign = 0;

bool rest(const std::vector<pros::Motor>& motors) {
  if (motors.empty()) return false;
  for (const auto& m : motors) {
    const double v = m.get_actual_velocity();
    if (!std::isfinite(v) || std::fabs(v) >= 3) return false;
  }
  return true;
}
double rpm(const std::vector<pros::Motor>& motors) {
  if (motors.empty()) return NAN;
  double peak = 0;
  for (const auto& m : motors) {
    const double value = m.get_actual_velocity();
    if (!std::isfinite(value)) return NAN;
    peak = std::max(peak, std::fabs(value));
  }
  return peak;
}
double heading_error() {
  return target.theta == ez::ANGLE_NOT_SET ? NAN :
      std::remainder(target.theta - nav_theta_get(), 360.0);
}
double position_error() {
  if (turn_motion) return 0;
  if (encoder_motion) return std::max(std::fabs(left_target - chassis.drive_sensor_left()),
                                    std::fabs(right_target - chassis.drive_sensor_right()));
  const auto p = nav_pose_get();
  return std::hypot(target.x - p.x, target.y - p.y);
}
// Save diagnostic rows in memory so serial printing does not delay a handoff.
void sample(unsigned flags = 0) {
  if (!tracing) return;
  const int l = command_left.load(), r = command_right.load();
  const int ls = l > 1 ? 1 : l < -1 ? -1 : 0;
  const int rs = r > 1 ? 1 : r < -1 ? -1 : 0;
  if ((ls && left_sign && ls != left_sign) || (rs && right_sign && rs != right_sign)) flags |= REVERSAL;
  if (ls) left_sign = ls;
  if (rs) right_sign = rs;
  const bool stopped = rest(chassis.left_motors) && rest(chassis.right_motors);
  if (stopped && !was_zero) { flags |= ZERO; ++zero_events; }
  was_zero = stopped;
  const double h = heading_error();
  if ((flags & START) && std::isfinite(h) && std::fabs(h) > 30) flags |= HEADING;
  if (fault || chassis.interfered) flags |= FAILURE;
  Row row{pros::millis() - sequence_start, movement, flags, command_ms.load(),
          nav_pose_get(), position_error(), h, rpm(chassis.left_motors), rpm(chassis.right_motors), l, r};
  capture_controller(row);
  row.requested_speed = motion_requested_speed;
  row.slew_requested = motion_slew_requested;
  row.peak_left = peak_left_command.load(); row.peak_right = peak_right_command.load();
  row.commands = command_samples.load();
  if (row_count < CAPACITY) rows[row_count++] = row; else ++dropped;
}
void observe_command(ez::Drive* drive, int l, int r) {
  if (drive != &chassis) return;
  command_left.store(l); command_right.store(r); command_ms.store(pros::millis());
  // Observe each native command, including peaks between wait-service samples.
  int old = peak_left_command.load();
  while (std::abs(l) > old && !peak_left_command.compare_exchange_weak(old, std::abs(l))) {}
  old = peak_right_command.load();
  while (std::abs(r) > old && !peak_right_command.compare_exchange_weak(old, std::abs(r))) {}
  command_samples.fetch_add(1);
}
}
namespace ez {
struct NavAccess {
  static int waypoint() {
    if (chassis.mode != PURE_PURSUIT) return -1;
    int result = -1;
    for (unsigned i = 1; i < chassis.injected_pp_index.size(); ++i)
      if (chassis.pp_index >= chassis.injected_pp_index[i]) result = i - 1;
    return result;
  }
  static void capture_controller(Row& row) {
    row.slew_active = chassis.slew_left.enabled() || chassis.slew_right.enabled();
    row.slew_limit = std::max(chassis.slew_left.output(), chassis.slew_right.output());
    row.xy_output = chassis.xyPID.output;
    row.angular_output = chassis.current_a_odomPID.output;
    row.control_heading_error = chassis.current_a_odomPID.error;
  }
  static void save_targets() {
    motion_requested_speed = chassis.pid_speed_max_get();
    motion_slew_requested = chassis.current_slew_on;
    left_target = chassis.leftPID.target_get(); right_target = chassis.rightPID.target_get();
    turn_target = chassis.turnPID.target_get();
    // Avoid allocation during native quick-chain's appended extension.
    if (chassis.mode == PURE_PURSUIT) chassis.pp_movements.reserve(chassis.pp_movements.size() + 1);
  }
  static void restore_target() {
    if (encoder_motion) {
      chassis.leftPID.target_set(left_target); chassis.rightPID.target_set(right_target);
    } else if (turn_motion) chassis.turnPID.target_set(turn_target);
    else if (chassis.mode == PURE_PURSUIT && !chassis.pp_movements.empty()) {
      // Keep storage/index valid: replace extension, never erase under the controller.
      chassis.pp_movements.back().target = nav_game_to_internal_pose(target);
      if (chassis.pp_index == static_cast<int>(chassis.pp_movements.size()) - 1)
        chassis.raw_pid_odom_ptp_set(chassis.pp_movements.back(), chassis.current_slew_on);
    }
  }
};
}

namespace {
void capture_controller(Row& row) { ez::NavAccess::capture_controller(row); }
}

bool nav_ok() { return active && !fault && !chassis.interfered; }
bool nav_sequence_active() { return active; }
const char* nav_fault() { return fault ? fault : chassis.interfered ? "EZ interference" : "none"; }
void nav_abort(const char* reason) {
  if (!fault) fault = reason;
  pending = false;
  ez::navigation_lock(); chassis.drive_set(0, 0); ez::navigation_unlock();
  stop_all_mechanisms();
  sample(FAILURE);
}
// Called by the existing exit observer, never reevaluates a PID or changes tuning.
void nav_observe_exit(ez::exit_output result) {
  if (!active || !waiting) return;
  if (result == ez::mA_EXIT || result == ez::VELOCITY_EXIT || result == ez::ERROR_NO_CONSTANTS) {
    chassis.interfered = true;
    if (!fault) fault = "native exit failure";
  }
}
namespace {
bool service(ez::Drive* drive) {
  // EZ calls this in the waiting task, not a new mechanism task.
  // Check safety, update mechanisms and fire each requested event at most once.
  if (drive != &chassis || !active || !waiting) return true;
  if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
    nav_abort("competition mode changed"); return false;
  }
  if (!nav_sensors_ready()) { nav_abort("navigation sensor fault"); return false; }
  if (!nav_ok()) { nav_abort("EZ interference"); return false; }
  if (pros::millis() - started >= timeout_ms) { nav_abort("motion timeout"); return false; }
  const auto p = nav_pose_get();
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) {
    nav_abort("invalid odometry"); return false;
  }
  traveled += std::hypot(p.x - previous_pose.x, p.y - previous_pose.y);
  previous_pose = p;
  const auto lift_before = lift_mode();
  const auto arm_before = arm_mode();
  mechanisms_update();
  if ((lift_before != LiftMode::Disabled && lift_before != LiftMode::Fault && lift_mode() == LiftMode::Fault) ||
      (arm_before != ArmMode::Disabled && arm_before != ArmMode::Fault && arm_mode() == ArmMode::Fault)) {
    nav_abort("mechanism fault during navigation"); return false;
  }
  NavProgress progress{p, traveled, ez::NavAccess::waypoint(), pros::millis() - started};
  for (unsigned i = 0; i < events.size(); ++i) {
    if (!fired[i] && events[i].when(progress)) {
      fired[i] = true;
      if (!events[i].action()) { nav_abort("event action rejected"); return false; }
      sample(ACTION);
    }
  }
  sample();
  return nav_ok();
}
bool verify_rest() {
  unsigned stable = 0;
  const auto began = pros::millis();
  while (pros::millis() - began < 2000) {
    if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) return false;
    mechanisms_update();
    sample();
    const bool stopped = rest(chassis.left_motors) && rest(chassis.right_motors);
    stable = stopped ? stable + 10 : 0;
    if (stable >= 100) return true;
    pros::delay(10);
  }
  return false;
}
bool accurate() {
  const double error = position_error(), h = heading_error();
  return std::isfinite(error) && error <= 1.0 &&
      (target.theta == ez::ANGLE_NOT_SET || (std::isfinite(h) && std::fabs(h) <= 3.0));
}
}
// ----- Sequence lifetime -----
// Starting takes control of the wait hooks; finishing releases them and stops outputs.
NavSequence::NavSequence(bool trace, unsigned deadline) {
  if (active) { nav_abort("nested sequence"); return; }
  owner_ = true; active = true; fault = nullptr;
  pending = busy = waiting = false; rest_verified = false;
  queued.clear(); events.clear(); fired.clear();
  ez::navigation_lock(); chassis.drive_set(0, 0); ez::navigation_unlock();
  stop_all_mechanisms();
  chassis.interfered = false; // Explicit new sequence only; never per movement.
  print_before = chassis.pid_print_toggle_get(); chassis.pid_print_toggle(false);
  tracing = trace; row_count = dropped = zero_events = movement = 0;
  was_zero = true; left_sign = right_sign = 0;
  timeout_ms = deadline ? deadline : 8000;
  sequence_start = pros::millis();
  ez::wait_service_set(service); ez::command_observer_set(observe_command);
}
NavSequence::~NavSequence() { if (owner_) finish(); }
bool NavSequence::ok() const { return owner_ && nav_ok(); }
bool NavSequence::finish() {
  // A route may end after CHAIN. Restore its real endpoint before the final wait.
  if (!owner_) return false;
  if (pending && nav_ok()) {
    ez::navigation_lock(); ez::NavAccess::restore_target(); ez::navigation_unlock();
    pending = false; waiting = true; started = pros::millis();
    events.clear(); fired.clear();
    chassis.pid_wait(); waiting = false;
    if (!nav_ok() || !accurate()) nav_abort("final settle inaccurate");
  }
  if (!queued.empty()) nav_abort("unconsumed events");
  if (!fault && !verify_rest()) nav_abort("rest timeout");
  ez::navigation_lock(); chassis.drive_set(0, 0); ez::navigation_unlock();
  // Drain a possible in-flight native tick, then reassert the boundary stop.
  pros::delay(20); ez::navigation_lock(); chassis.drive_set(0, 0); ez::navigation_unlock();
  command_left.store(0); command_right.store(0);
  rest_verified = verify_rest();
  if (!rest_verified) nav_abort("final rest not verified");
  sample(FINISH);
  stop_all_mechanisms();
  const bool result = !fault && !chassis.interfered;
  ez::wait_service_set(nullptr); ez::command_observer_set(nullptr);
  chassis.pid_print_toggle(print_before);
  active = false; owner_ = false; waiting = busy = pending = false;
  return result;
}
bool nav_events(std::vector<NavEvent> next) {
  if (!nav_ok() || busy || pending || !queued.empty()) { nav_abort("events must be armed before motion, outside a chain handoff"); return false; }
  for (const auto& e : next) if (!e.when || !e.action) { nav_abort("invalid event"); return false; }
  queued = std::move(next); return true;
}
void nav_trace_marker() { sample(ACTION); }
// ----- Movement handoff -----
// CHAIN releases early for the next command; SETTLE also checks error and rest.
bool nav_detail::move(const std::function<void()>& dispatch, NavEndMode end,
                      ez::pose requested, double, bool encoder, bool turn) {
  if (!nav_ok()) return false;
  if (busy) { nav_abort("recursive navigation from event"); return false; }
  busy = true; pending = false; encoder_motion = encoder; turn_motion = turn;
  target = requested; previous_pose = nav_pose_get(); traveled = 0;
  events = std::move(queued); queued.clear(); fired.assign(events.size(), false);
  started = pros::millis(); ++movement;
  ez::navigation_lock();
  peak_left_command.store(0); peak_right_command.store(0); command_samples.store(0);
  command_left.store(0); command_right.store(0); command_ms.store(0);
  dispatch(); ez::NavAccess::save_targets();
  ez::navigation_unlock(); sample(START); // START proves dispatch() actually ran.
  waiting = true;
  if (end == NavEndMode::CHAIN) chassis.pid_wait_quick_chain(); else chassis.pid_wait();
  waiting = false;
  // Native CHAIN release is an intermediate handoff, not final pose acceptance.
  if (nav_ok() && end == NavEndMode::SETTLE && !accurate()) nav_abort("settle outside 1in or 3deg acceptance");
  for (unsigned i = 0; i < events.size(); ++i)
    if (events[i].required && !fired[i]) nav_abort("required movement event not reached");
  if (nav_ok() && end == NavEndMode::SETTLE && !verify_rest()) nav_abort("settle rest timeout");
  if (nav_ok() && end == NavEndMode::SETTLE && !accurate()) nav_abort("pose drift during settle");
  if (!nav_ok()) nav_abort("EZ interference");
  pending = nav_ok() && end == NavEndMode::CHAIN;
  sample(!nav_ok() ? FAILURE : pending ? RELEASE : SETTLED);
  busy = false;
  return nav_ok();
}
// ----- Post-run report -----
// Print the saved timeline only after the sequence has finished and rest was verified.
void nav_trace_dump() {
  if (active || !tracing) return;
  if (!rest_verified) {
    std::printf("CHAIN TRACE WITHHELD: rest not verified after stop; fault=%s\n", nav_fault());
    std::fflush(stdout);
    return;
  }
  std::printf("\n===== BUFFERED MOVEMENT SUMMARY =====\n");
  for (unsigned id = 1; id <= movement; ++id) {
    int peak_l = 0, peak_r = 0, requested = 0;
    double rpm_l = NAN, rpm_r = NAN, slew_min = INFINITY, slew_max = -INFINITY;
    bool slew_requested = false, slew_active = false, failure_reported = false;
    unsigned commands = 0;
    for (unsigned i = 0; i < row_count; ++i) {
      const auto& s = rows[i];
      if (s.motion != id) continue;
      requested = s.requested_speed;
      peak_l = std::max(peak_l, s.peak_left); peak_r = std::max(peak_r, s.peak_right);
      commands = std::max(commands, s.commands);
      if (std::isfinite(s.left_rpm)) rpm_l = std::isfinite(rpm_l) ? std::max(rpm_l,s.left_rpm) : s.left_rpm;
      if (std::isfinite(s.right_rpm)) rpm_r = std::isfinite(rpm_r) ? std::max(rpm_r,s.right_rpm) : s.right_rpm;
      slew_requested |= s.slew_requested; slew_active |= s.slew_active;
      slew_min = std::min(slew_min,s.slew_limit); slew_max = std::max(slew_max,s.slew_limit);
      if ((s.flags & FAILURE) && !failure_reported) {
        std::printf("MOVE %u FAILED at %u ms | error: %.3f in | fault=%s\n",id,s.ms,s.error,nav_fault());
        failure_reported = true;
      }
      if (s.flags & START) std::printf("MOVE %u START (dispatched) at %u ms\n",id,s.ms);
      if (s.flags & RELEASE) std::printf("MOVE %u CHAIN RELEASE at %u ms | ERROR AT RELEASE: %.3f in\n",id,s.ms,s.error);
      if ((s.flags & SETTLED) && !(s.flags & FAILURE)) std::printf("MOVE %u SETTLED at %u ms | error: %.3f in\n",id,s.ms,s.error);
    }
    std::printf("MOVE %u requested_speed=%d | max_abs_command L=%d R=%d | command_samples=%u\n"
                "max_sampled_abs_motor_RPM L=%.2f R=%.2f | slew_requested=%d slew_active_seen=%d | sampled_slew_limit=%.2f..%.2f\n"
                "command_near_127 (>=125): L=%s R=%s\n",
                id,requested,peak_l,peak_r,commands,rpm_l,rpm_r,slew_requested,slew_active,slew_min,slew_max,
                commands ? (peak_l >= 125 ? "YES" : "NO") : "NO DATA",
                commands ? (peak_r >= 125 ? "YES" : "NO") : "NO DATA");
  }
  std::printf("RPM is peak absolute individual-motor telemetry sampled during waits, not wheel/output-shaft RPM.\n"
              "Controller columns below are sampled reads; command peaks observe every native write.\n");
  std::printf("\n===== MOTION CHAIN TRACE BEGIN =====\n"
              "flags: START/NEXT_MOTION_START=1 CHAIN_RELEASE=2 SETTLED=4 ZERO_SPEED_EVENT=8 "
              "COMMAND_SIGN_REVERSAL=16 LARGE_HEADING_CORRECTION=32 INTERFERENCE/FAULT=64 EVENT=128 FINISH=256\n"
              "ms,motion,flags,command_timestamp,pub_x,pub_y,pub_h,position_error,heading_error,L_cmd,R_cmd,L_max_abs_motor_rpm,R_max_abs_motor_rpm,slew_limit,slew_active,xy_raw,angular_raw,control_heading_error\n");
  for (unsigned i = 0; i < row_count; ++i) {
    const auto& s = rows[i];
    std::printf("%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.3f\n",
      s.ms,s.motion,s.flags,s.command_time,s.pose.x,s.pose.y,s.pose.theta,s.error,s.heading_error,
      s.left,s.right,s.left_rpm,s.right_rpm,s.slew_limit,s.slew_active,s.xy_output,s.angular_output,s.control_heading_error);
    if (i % 10 == 0) { std::fflush(stdout); pros::delay(10); }
  }
  const auto p = nav_pose_get();
  std::printf("SUMMARY total_ms=%u final=%.3f,%.3f,%.3f zero_events=%u fault=%s interfered=%d dropped=%u\n",
    row_count ? rows[row_count-1].ms : 0,p.x,p.y,p.theta,zero_events,nav_fault(),chassis.interfered,dropped);
  for (unsigned i=0; i<row_count; ++i) if (rows[i].flags & (RELEASE | SETTLED)) {
    double minimum = INFINITY;
    for (unsigned j=0; j<row_count; ++j)
      if (std::abs(static_cast<int>(rows[j].ms)-static_cast<int>(rows[i].ms)) <= 100)
        minimum = std::min(minimum, std::max(std::fabs(rows[j].left_rpm),std::fabs(rows[j].right_rpm)));
    std::printf("TRANSITION motion=%u release_ms=%u min_max_abs_wheel_rpm_within_100ms=%.2f\n",rows[i].motion,rows[i].ms,minimum);
  }
  std::printf("===== MOTION CHAIN TRACE END =====\n"); std::fflush(stdout);
}

void nav_cancel_sequence() {
  // PROS competition lifecycle has stopped the old autonomous task before reuse.
  ez::wait_service_set(nullptr); ez::command_observer_set(nullptr);
  ez::navigation_lock(); chassis.drive_set(0,0); ez::navigation_unlock();
  stop_all_mechanisms();
  if (active) { fault = "competition mode changed"; chassis.pid_print_toggle(print_before); }
  active = waiting = busy = pending = false;
  queued.clear(); events.clear(); fired.clear();
}

