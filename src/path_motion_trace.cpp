#include "path_motion_trace.hpp"
#include "EZ-Template/path_trace_hook.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

namespace {
constexpr unsigned CAPACITY = 800;  // 16 s at 20 ms; >=8 s with every-tick events.
constexpr unsigned SAMPLE_MS = 20;
constexpr double COMMAND_EPS = 1.0;  // Ignore sub-one-unit sign chatter.
enum Event : unsigned {
  WAYPOINT = 1, BOOMERANG = 2, TRANSLATION = 4, LEFT = 8, RIGHT = 16,
  XY_ERROR = 32, FINAL_PLANE = 64, CARROT_LOCK = 128, RAW_XY = 256,
  BIAS_NEGATIVE = 512
};
struct Sample {
  unsigned ms = 0, events = 0;
  int index = -1, last = -1, previous_index = -1, mode = 0, direction = 0;
  ez::pose pose{}, active{}, final{}, control{};
  double xy_error = 0, xy_raw = 0, xy_derivative = 0;
  double angular_error = 0, angular_raw = 0;
  ez::PathControlFrame frame{};
  double left_rpm = 0, right_rpm = 0, distance = 0, remaining = NAN;
  int left_cmd = 0, right_cmd = 0;
  bool boom = false, locked = false, bias = false, interfered = false;
  ez::exit_output xy_exit = ez::RUNNING, angular_exit = ez::RUNNING;
};
struct Exit {
  ez::exit_output reason = ez::RUNNING;
  unsigned ms = 0;
  int index = -1;
};
struct Summary {
  double peak = 0, min_translation = INFINITY, overshoot = 0, max_heading = 0;
  unsigned reversals = 0, near_reversals = 0, left_reversals = 0, right_reversals = 0;
  unsigned boom_ms = 0, transitions = 0, ticks = 0, max_gap = 0;
  double boom_distance = NAN;
  bool saw_boom = false;
};
Sample samples[CAPACITY];
unsigned head = 0, row_count = 0, overwritten = 0, began = 0, last_row_ms = 0;
unsigned wait_ms = 0, stopped_ms = 0, rest_ms = 0;
Sample previous;
bool have_previous = false, armed = false, ready = false, previous_print = false;
std::atomic<bool> capturing{false};
std::atomic<unsigned> skipped{0};
pros::Mutex trace_mutex;
Exit xy_exit, angular_exit;
Summary summary;
ez::pose requested{}, wait_pose{}, rest_pose{};
double segment_x = 0, segment_y = 0, reference_heading = NAN;
bool segment_valid = false, wait_seen = false, wait_interfered = false;
int translation_sign = 0, left_sign = 0, right_sign = 0, xy_sign = 0;
int plane_sign = 0, raw_sign = 0;

double heading_error(double target, double actual) {
  return std::remainder(target - actual, 360.0);
}
constexpr bool changed_sign(double value, int& previous_sign, double epsilon) {
  const int now = value > epsilon ? 1 : value < -epsilon ? -1 : 0;
  const bool changed = now && previous_sign && now != previous_sign;
  if (now) previous_sign = now;  // Remember polarity across zero samples.
  return changed;
}
// Compile-time checks protect the key diagnosis from zero-crossing chatter and
// the common mistake of counting initial negative EZ rev output as a reversal.
static_assert([] {
  int sign = 0;
  return !changed_sign(-80, sign, COMMAND_EPS) &&
         !changed_sign(0, sign, COMMAND_EPS) &&
         !changed_sign(0.5, sign, COMMAND_EPS) &&
         changed_sign(20, sign, COMMAND_EPS) &&
         !changed_sign(30, sign, COMMAND_EPS) &&
         changed_sign(-20, sign, COMMAND_EPS);
}());
static_assert([] {
  int sign = 0;
  return !changed_sign(0, sign, COMMAND_EPS) &&
         !changed_sign(20, sign, COMMAND_EPS) &&
         !changed_sign(1, sign, COMMAND_EPS) &&
         !changed_sign(0, sign, COMMAND_EPS) &&
         changed_sign(-2, sign, COMMAND_EPS);
}());
double average_rpm(const std::vector<pros::Motor>& motors) {
  if (motors.empty()) return NAN;
  double sum = 0;
  for (const auto& motor : motors) {
    const double rpm = motor.get_actual_velocity();
    if (!std::isfinite(rpm)) return NAN;
    sum += rpm;  // PROS configured reversal is already included.
  }
  return sum / motors.size();
}
bool motors_at_rest(const std::vector<pros::Motor>& motors) {
  if (motors.empty()) return false;
  for (const auto& motor : motors) {
    const double rpm = motor.get_actual_velocity();
    if (!std::isfinite(rpm) || std::fabs(rpm) >= 3) return false;
  }
  return true;  // Check every motor: opposite wheel speeds cannot cancel out.
}
void append(const Sample& sample) {
  samples[head] = sample;
  head = (head + 1) % CAPACITY;
  if (row_count < CAPACITY) ++row_count;
  else ++overwritten;
}
const char* reason(ez::exit_output value) {
  switch (value) {
    case ez::RUNNING: return "RUNNING/not observed";
    case ez::SMALL_EXIT: return "SMALL_EXIT";
    case ez::BIG_EXIT: return "BIG_EXIT";
    case ez::VELOCITY_EXIT: return "VELOCITY_EXIT";
    case ez::mA_EXIT: return "mA_EXIT";
    case ez::ERROR_NO_CONSTANTS: return "ERROR_NO_CONSTANTS";
  }
  return "UNKNOWN";
}
double printable_heading(ez::pose pose) {
  return pose.theta == ez::ANGLE_NOT_SET ? NAN : pose.theta;
}
}  // namespace

namespace ez {
struct PathMotionTrace {
  static void capture(Drive* drive, const PathControlFrame& frame) {
    if (drive != &chassis || !capturing.load()) return;
    // Never block the control task behind a diagnostic reader/exit observer.
    if (!trace_mutex.take(0)) { ++skipped; return; }
    if (!capturing.load()) { trace_mutex.give(); return; }
    Sample s;
    s.ms = pros::millis() - began;
    s.mode = drive->mode;
    s.index = drive->pp_index;
    s.last = static_cast<int>(drive->pp_movements.size()) - 1;
    s.previous_index = have_previous ? previous.index : 0;
    s.pose = drive->odom_current;
    if (s.index < 0 || s.index > s.last) { trace_mutex.give(); return; }
    s.active = drive->pp_movements[s.index].target;
    s.final = drive->pp_movements.back().target;
    s.control = drive->odom_target;  // Actual XY target, including moving carrot.
    s.direction = drive->current_drive_direction;
    s.boom = s.active.theta != ANGLE_NOT_SET;
    s.distance = std::hypot(s.final.x - s.pose.x, s.final.y - s.pose.y);
    s.locked = s.boom && std::hypot(s.active.x - s.pose.x, s.active.y - s.pose.y)
                                < drive->LOOK_AHEAD / 2.0;
    if (segment_valid)
      s.remaining = (requested.x - s.pose.x) * segment_x +
                    (requested.y - s.pose.y) * segment_y;
    s.xy_error = drive->xyPID.error;
    s.xy_raw = drive->xyPID.output;
    s.xy_derivative = drive->xyPID.derivative;
    s.angular_error = drive->current_a_odomPID.error;
    s.angular_raw = drive->current_a_odomPID.output;
    s.frame = frame;
    s.bias = drive->is_odom_turn_bias_enabled;
    s.interfered = drive->interfered;
    s.xy_exit = xy_exit.reason;
    s.angular_exit = angular_exit.reason;
    // The native private_drive_set takes int, then sends int*12000/127 mV.
    // Capture its actual arguments, not voltage telemetry or recomputed mixing.
    s.left_cmd = static_cast<int>(frame.left);
    s.right_cmd = static_cast<int>(frame.right);
    const double translation = (s.left_cmd + s.right_cmd) / 2.0;
    if (s.index != s.previous_index) { s.events |= WAYPOINT; ++summary.transitions; }
    if (s.boom && (!have_previous || !previous.boom)) s.events |= BOOMERANG;
    if (s.locked && (!have_previous || !previous.locked)) s.events |= CARROT_LOCK;
    if (s.bias && frame.bias_scale < 0 &&
        (!have_previous || !previous.bias || previous.frame.bias_scale >= 0))
      s.events |= BIAS_NEGATIVE;
    if (changed_sign(translation, translation_sign, COMMAND_EPS)) {
      s.events |= TRANSLATION; ++summary.reversals;
      if (s.distance <= 8) ++summary.near_reversals;
    }
    if (changed_sign(s.left_cmd, left_sign, COMMAND_EPS)) {
      s.events |= LEFT; ++summary.left_reversals;
    }
    if (changed_sign(s.right_cmd, right_sign, COMMAND_EPS)) {
      s.events |= RIGHT; ++summary.right_reversals;
    }
    if (changed_sign(s.xy_error, xy_sign, 0.01)) s.events |= XY_ERROR;
    if (changed_sign(s.xy_raw, raw_sign, COMMAND_EPS)) s.events |= RAW_XY;
    if (segment_valid && s.index == s.last &&
        changed_sign(s.remaining, plane_sign, 0.01)) s.events |= FINAL_PLANE;
    summary.peak = std::max(summary.peak,
                           std::max(std::fabs(frame.left), std::fabs(frame.right)));
    if (s.distance <= 8) {
      summary.min_translation = std::min(summary.min_translation,
                                          translation * frame.direction_sign);
      const auto game = nav_internal_to_game_pose(s.pose);
      if (std::isfinite(reference_heading))
        summary.max_heading = std::max(summary.max_heading,
                         std::fabs(heading_error(reference_heading, game.theta)));
    }
    if (segment_valid && s.index == s.last)
      summary.overshoot = std::max(summary.overshoot, -s.remaining);
    if (s.boom && !summary.saw_boom) {
      summary.saw_boom = true;
      summary.boom_ms = s.ms;
      summary.boom_distance = s.distance;
    }
    ++summary.ticks;
    if (have_previous) summary.max_gap = std::max(summary.max_gap, s.ms - previous.ms);
    // Events are checked on every controller tick (~10 ms), but motor telemetry
    // and routine rows are collected only every ~20 ms (or on an event).
    if (!have_previous || s.ms - last_row_ms >= SAMPLE_MS || s.events) {
      s.left_rpm = average_rpm(drive->left_motors);
      s.right_rpm = average_rpm(drive->right_motors);
      append(s);
      last_row_ms = s.ms;
    }
    previous = s;
    have_previous = true;
    trace_mutex.give();
  }
};
}  // namespace ez

void path_motion_trace_arm() {
  if (capturing.load() || armed) return;
  previous_print = chassis.pid_print_toggle_get();
  chassis.pid_print_toggle(false);
  armed = true;
  ready = false;
  ez::path_trace_hook_set(ez::PathMotionTrace::capture);
}

void path_motion_trace_begin(const std::vector<ez::odom>& movements) {
  if (!armed || movements.empty()) return;
  trace_mutex.take();
  head = row_count = overwritten = last_row_ms = 0;
  skipped.store(0);
  xy_exit = {}; angular_exit = {}; summary = {};
  have_previous = wait_seen = false;
  translation_sign = left_sign = right_sign = xy_sign = plane_sign = raw_sign = 0;
  requested = nav_internal_to_game_pose(movements.back().target);
  const auto start = nav_pose_get();
  ez::pose segment_start = start;
  for (int i = static_cast<int>(movements.size()) - 2; i >= 0; --i) {
    const auto candidate = movements[i].target;
    if (std::hypot(requested.x - candidate.x, requested.y - candidate.y) > 1e-6) {
      segment_start = candidate;
      break;
    }
  }
  const double length = std::hypot(requested.x - segment_start.x,
                                 requested.y - segment_start.y);
  segment_valid = length > 1e-6;
  segment_x = segment_valid ? (requested.x - segment_start.x) / length : 0;
  segment_y = segment_valid ? (requested.y - segment_start.y) / length : 0;
  reference_heading = requested.theta;
  if (requested.theta == ez::ANGLE_NOT_SET) {
    reference_heading = segment_valid ? std::atan2(segment_x, segment_y) * 180 / M_PI : NAN;
    if (movements.back().drive_direction == ez::rev) reference_heading += 180;
  }
  began = pros::millis();
  armed = false;
  ready = true;
  capturing.store(true);
  trace_mutex.give();
}

void path_motion_trace_exit(ez::PID* pid, ez::exit_output result) {
  if (!capturing.load() || result == ez::RUNNING) return;
  if (pid != &chassis.xyPID && pid != &chassis.current_a_odomPID) return;
  trace_mutex.take();  // Wait task only; controller callback never blocks.
  Exit& event = pid == &chassis.xyPID ? xy_exit : angular_exit;
  if (event.reason == ez::RUNNING) {
    event.reason = result;
    event.ms = pros::millis() - began;
    event.index = have_previous ? previous.index : -1;
  }
  trace_mutex.give();
}

void path_motion_trace_wait_returned() {
  if (!capturing.load()) return;
  trace_mutex.take();
  wait_ms = pros::millis() - began;
  wait_pose = nav_pose_get();
  wait_interfered = chassis.interfered;
  wait_seen = true;
  trace_mutex.give();
}

namespace {
void print_events(const Sample& s) {
  const auto t = static_cast<unsigned long>(s.ms);
  if (s.events & WAYPOINT)
    std::printf("--- %lu ms WAYPOINT ADVANCE: %d -> %d ---\n", t, s.previous_index, s.index);
  if (s.events & BOOMERANG)
    std::printf("--- %lu ms BOOMERANG/HEADING CONTROL ACTIVE; final distance=%.3f ---\n", t, s.distance);
  if (s.events & CARROT_LOCK)
    std::printf("--- %lu ms BOOMERANG CARROT LOCKED TO TARGET (<lookahead/2) ---\n", t);
  if (s.events & TRANSLATION)
    std::printf("--- %lu ms TRANSLATION COMMAND SIGN CHANGED %s ---\n", t,
                s.left_cmd + s.right_cmd < 0 ? "+ -> - (EZ)" : "- -> + (EZ)");
  if (s.events & LEFT) std::printf("--- %lu ms LEFT COMMAND SIGN REVERSAL ---\n", t);
  if (s.events & RIGHT) std::printf("--- %lu ms RIGHT COMMAND SIGN REVERSAL ---\n", t);
  if (s.events & XY_ERROR)
    std::printf("--- %lu ms XY PID ERROR SIGN CHANGE (may be target change) ---\n", t);
  if (s.events & RAW_XY) std::printf("--- %lu ms RAW XY PID OUTPUT SIGN CHANGE ---\n", t);
  if (s.events & FINAL_PLANE)
    std::printf("--- %lu ms CROSSED FINAL SEGMENT TARGET PLANE ---\n", t);
  if (s.events & BIAS_NEGATIVE)
    std::printf("--- %lu ms TURN BIAS SCALE NEGATIVE ---\n", t);
}
void dump() {
  std::printf("\n===== PURE PURSUIT BUFFERED TRACE BEGIN =====\n"
              "ASCII; ms relative to path dispatch; inches/degrees/RPM; nan heading=XY-only.\n"
              "Commands are native integer EZ motor requests; +EZ is FRONT-first.\n"
              "travel_cmd=(L+R)/2*dir_sign: positive=requested travel, negative=reverse.\n"
              "control_x/y=actual PTP/carrot target; final_plane_remaining<0=past plane.\n"
              "mode 5=POINT_TO_POINT,6=PURE_PURSUIT; dir 0=fwd,1=rev.\n"
              "Samples=%u overwritten=%u skipped_lock=%u max_control_gap_ms=%u\n",
              row_count, overwritten, skipped.load(), summary.max_gap);
  std::printf("ms,pub_x,pub_y,pub_h,ez_x,ez_y,ez_h,wp,last,active_x,active_y,active_ez_h,"
              "final_x,final_y,final_ez_h,dist_active,dist_final,final_h_error,mode,dir,boom,"
              "control_x,control_y,signed_target,past_flipped,xy_error,xy_raw,xy_derivative,"
              "angle_error,angle_raw,bias_enabled,bias_scale,slew_limit,xy_scaled,angle_scaled,"
              "L_cmd,R_cmd,ez_translation,travel_cmd,L_rpm,R_rpm,final_plane_remaining,"
              "output_enabled,interfered,xy_exit,angle_exit,event_bits\n");
  bool xy_printed = false, angle_printed = false;
  for (unsigned n = 0; n < row_count; ++n) {
    const Sample& s = samples[(head + CAPACITY - row_count + n) % CAPACITY];
    if (!xy_printed && xy_exit.reason != ez::RUNNING && xy_exit.ms <= s.ms) {
      std::printf("--- %u ms XY EXIT CONDITION: %s (wp=%d) ---\n", xy_exit.ms, reason(xy_exit.reason), xy_exit.index);
      xy_printed = true;
    }
    if (!angle_printed && angular_exit.reason != ez::RUNNING && angular_exit.ms <= s.ms) {
      std::printf("--- %u ms ANGULAR EXIT CONDITION: %s (wp=%d) ---\n", angular_exit.ms, reason(angular_exit.reason), angular_exit.index);
      angle_printed = true;
    }
    print_events(s);
    const auto game = nav_internal_to_game_pose(s.pose);
    const double translation = (s.left_cmd + s.right_cmd) / 2.0;
    std::printf("%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,"
                "%.3f,%.3f,%.4f,%d,%.4f,%.3f,%.5f,%.3f,%.3f,%d,%.4f,%.3f,%.3f,%.3f,"
                "%d,%d,%.3f,%.3f,%.2f,%.2f,%.4f,%d,%d,%d,%d,%u\n",
                s.ms, game.x, game.y, game.theta, s.pose.x, s.pose.y, s.pose.theta,
                s.index, s.last, s.active.x, s.active.y, printable_heading(s.active),
                s.final.x, s.final.y, printable_heading(s.final),
                std::hypot(s.active.x - s.pose.x, s.active.y - s.pose.y), s.distance,
                std::isfinite(reference_heading) ? heading_error(reference_heading, game.theta) : NAN,
                s.mode, s.direction, s.boom, s.control.x, s.control.y,
                s.frame.signed_target, s.frame.flipped, s.xy_error, s.xy_raw, s.xy_derivative,
                s.angular_error, s.angular_raw, s.bias, s.frame.bias_scale,
                s.frame.slew_limit, s.frame.xy_scaled, s.frame.angular_scaled,
                s.left_cmd, s.right_cmd, translation, translation * s.frame.direction_sign,
                s.left_rpm, s.right_rpm, s.remaining, s.frame.output_enabled, s.interfered,
                static_cast<int>(s.xy_exit), static_cast<int>(s.angular_exit), s.events);
    std::fflush(stdout);
    pros::delay(20);  // Pace terminal output ONLY after measured rest.
  }
  if (!xy_printed && xy_exit.reason != ez::RUNNING)
    std::printf("--- %u ms XY EXIT CONDITION: %s ---\n", xy_exit.ms, reason(xy_exit.reason));
  if (!angle_printed && angular_exit.reason != ez::RUNNING)
    std::printf("--- %u ms ANGULAR EXIT CONDITION: %s ---\n", angular_exit.ms, reason(angular_exit.reason));
  std::printf("--- %u ms FINAL WAIT RETURNED ---\n--- %u ms HARNESS STOP ---\n"
              "--- %u ms MEASURED REST CONFIRMED ---\n", wait_ms, stopped_ms, rest_ms);
  std::printf("\n===== PURE PURSUIT MOTION SUMMARY =====\n"
              "Requested final PUBLIC: X=%.3f Y=%.3f H=%.3f (nan=not requested)\n"
              "Actual at wait return: X=%.3f Y=%.3f H=%.3f\n"
              "Actual after stop/rest: X=%.3f Y=%.3f H=%.3f\n"
              "Position error at wait/rest=%.4f / %.4f in\n"
              "Heading reference=%.3f (%s); error at wait/rest=%+.3f / %+.3f deg\n"
              "Peak wheel speed command=%.3f\n"
              "Minimum travel-normalized translation within final 8 inches=%.3f\n"
              "Translation reversed near endpoint: %s\n"
              "Translation sign reversals total=%u near endpoint=%u\n"
              "Left/right motor command reversal occurred: %s (L=%u R=%u)\n"
              "Maximum final-segment plane overshoot while final target active=%.4f in\n"
              "Maximum heading-reference error within final 8 inches=%.3f deg\n"
              "Waypoint transitions=%u (times in event markers above)\n"
              "Boomerang active=%s first_ms=%u distance_to_final=%.3f\n"
              "Exit reason XY=%s at %u ms; ANGULAR=%s at %u ms\n"
              "Interference at wait return=%s\n"
              "Movement duration to wait=%u ms; controller ticks observed=%u\n"
              "Sign events use >1 command-unit deadband and remember sign across zero.\n"
              "Overwritten rows=%u; skipped captures=%u; whole-run summary retains observed ticks.\n"
              "===== PURE PURSUIT BUFFERED TRACE END =====\n",
              requested.x, requested.y, printable_heading(requested),
              wait_pose.x, wait_pose.y, wait_pose.theta, rest_pose.x, rest_pose.y, rest_pose.theta,
              std::hypot(requested.x - wait_pose.x, requested.y - wait_pose.y),
              std::hypot(requested.x - rest_pose.x, requested.y - rest_pose.y), reference_heading,
              requested.theta == ez::ANGLE_NOT_SET ? "natural final segment; not enforced" : "requested",
              heading_error(reference_heading, wait_pose.theta), heading_error(reference_heading, rest_pose.theta),
              summary.peak, std::isfinite(summary.min_translation) ? summary.min_translation : NAN,
              summary.min_translation < -COMMAND_EPS ? "YES" : "NO", summary.reversals, summary.near_reversals,
              summary.left_reversals || summary.right_reversals ? "YES" : "NO",
              summary.left_reversals, summary.right_reversals,
              segment_valid ? summary.overshoot : NAN, summary.max_heading, summary.transitions,
              summary.saw_boom ? "YES" : "NO", summary.boom_ms, summary.boom_distance,
              reason(xy_exit.reason), xy_exit.ms, reason(angular_exit.reason), angular_exit.ms,
              wait_interfered ? "YES" : "NO", wait_ms, summary.ticks, overwritten, skipped.load());
  std::fflush(stdout);
}
}  // namespace

bool path_motion_trace_stop_and_dump() {
  if (!ready || !wait_seen) return false;
  // Explicit diagnostic harness boundary only; the navigation API never stops PID.
  chassis.drive_set(0, 0);
  stopped_ms = pros::millis() - began;
  capturing.store(false);
  ez::path_trace_hook_set(nullptr);
  trace_mutex.take();  // Drain any in-flight callback before reading its buffer.
  trace_mutex.give();
  // Let an in-flight native tick finish, then assert the harness stop once more.
  pros::delay(20);
  chassis.drive_set(0, 0);
  unsigned stable_ms = 0;
  const auto coast_start = pros::millis();
  while (pros::millis() - coast_start < 3000) {
    pros::delay(20);
    stable_ms = motors_at_rest(chassis.left_motors) &&
                motors_at_rest(chassis.right_motors) ? stable_ms + 20 : 0;
    if (stable_ms >= 300) break;
  }
  if (stable_ms < 300) return false;  // Retain buffer; caller may retry after rest.
  rest_ms = pros::millis() - began;
  rest_pose = nav_pose_get();
  dump();
  ready = false;
  chassis.pid_print_toggle(previous_print);
  return true;
}
