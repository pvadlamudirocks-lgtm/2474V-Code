#include "motion_exit_trace.hpp"
#include "autons.hpp"
#include <cstdio>
#include <cmath>
#include <utility>

namespace {
struct Snapshot {
  ez::pose internal{0, 0, 0};
  ez::pose active{0, 0};
  int index = -1, last_index = -1;
  double xy_error = 0, angle_error = 0;
  std::uint32_t elapsed = 0;
};
struct ExitEvent {
  bool seen = false;
  ez::exit_output reason = ez::RUNNING;
  int order = 0;
  Snapshot state;
};
bool armed = false, recording = false, previous_print = false;
bool interfered_before = false, interfered_after = false;
std::uint32_t start_ms = 0;
int event_order = 0;
ez::pose target{0, 0};
ExitEvent xy_event, angle_event;
Snapshot final_state;
double final_imu = 0;
char pages[3][8][96] = {};
int page = 0;
bool pages_ready = false;

pros::Mutex page_mutex;

const char* reason_name(ez::exit_output reason) {
  switch (reason) {
    case ez::RUNNING: return "RUNNING";
    case ez::SMALL_EXIT: return "SMALL";
    case ez::BIG_EXIT: return "BIG";
    case ez::VELOCITY_EXIT: return "VELOCITY";
    case ez::mA_EXIT: return "OVER-CURRENT";
    case ez::ERROR_NO_CONSTANTS: return "NO-CONSTANTS";
  }
  return "UNKNOWN";
}
void draw_page() {  // Caller holds page_mutex.
  if (!pages_ready || !odom_diagnostic_screen_is_active()) return;
  for (int row = 0; row < 8; ++row) pros::lcd::set_text(row, pages[page][row]);
}
}  // namespace

namespace ez {
struct MotionExitTrace {
  static void controller_snapshot(std::uint32_t began) {
    // Freeze read-only values before any terminal I/O. No PID compute/iterate calls.
    const auto captured = pros::millis();
    const auto pose = chassis.odom_current;
    const auto game = nav_internal_to_game_pose(pose);
    const auto imu = chassis.drive_imu_get();
    const auto mode = chassis.mode;
    const int index = chassis.pp_index;
    const int count = static_cast<int>(chassis.pp_movements.size());
    const bool valid = index >= 0 && index < count;
    const auto active = valid ? chassis.pp_movements[index].target : ez::pose{NAN, NAN};
    const auto final = count > 0 ? chassis.pp_movements.back().target : ez::pose{NAN, NAN};
    const double xy_error = chassis.xyPID.error;
    const double angular_error = chassis.current_a_odomPID.error;
    const double heading_target = pose.theta + angular_error;
    const double xy_raw = chassis.xyPID.output;
    const double angle_raw = chassis.current_a_odomPID.output;
    const double max_slew = std::fmax(chassis.slew_left.output(), chassis.slew_right.output());
    const double bias_amount = chassis.odom_turn_bias_amount;
    const bool bias_enabled = chassis.is_odom_turn_bias_enabled;
    const bool output_enabled = chassis.drive_toggle;
    const auto voltage_left = chassis.left_motors[0].get_voltage();
    const auto voltage_right = chassis.right_motors[0].get_voltage();
    const auto task_state = chassis.ez_auto.get_state();

    // Read-only reconstruction of v3.2.2 ptp_task() local output math.
    const double bias_scale = 1.0 - (1.0 - std::cos(ez::util::to_rad(angular_error))) / bias_amount;
    double xy = ez::util::clamp(xy_raw, max_slew);
    if (bias_enabled) xy *= bias_scale;
    double angular = angle_raw;
    double faster = std::fmax(std::fabs(xy), std::fabs(angular));
    if (faster > max_slew) { xy *= max_slew / faster; angular *= max_slew / faster; }
    double left = xy + angular, right = xy - angular;
    faster = std::fmax(std::fabs(left), std::fabs(right));
    if (faster > max_slew) { left *= max_slew / faster; right *= max_slew / faster; }

    std::printf("\n==============================\nONE-SHOT CONTROLLER SNAPSHOT\n"
                "elapsed_ms = %lu\nPUBLIC X/Y/H = %.3f %.3f %.3f\n"
                "EZ X/Y/H = %.3f %.3f %.3f\nIMU H = %.3f\n"
                "mode = %d (0=DISABLE, 5=PTP, 6=PP)\n"
                "pp_index = %d\nwaypoint count = %d\n"
                "active waypoint = (%.3f, %.3f)\nfinal waypoint = (%.3f, %.3f)\n"
                "distance to active = %.3f\nXY PID error = %.5f\nangular PID error = %.5f\n"
                "heading target (EZ H + angular error) = %.3f\n"
                "turn-bias scale = %.5f (enabled = %s)\n"
                "reconstructed PTP left/right = %.5f %.5f\n"
                "motor voltage readback left/right (mV) = %ld %ld\n"
                "PID drive output enabled = %s\ncontroller active (mode != DISABLE) = %s\n"
                "EZ task state = %d (0=running,1=ready,2=blocked,3=suspended,4=deleted)\n"
                "==============================\n",
                static_cast<unsigned long>(captured - began), game.x, game.y, game.theta,
                pose.x, pose.y, pose.theta, imu, static_cast<int>(mode), index, count,
                active.x, active.y, final.x, final.y, std::hypot(active.x - pose.x, active.y - pose.y),
                xy_error, angular_error, heading_target, bias_scale, bias_enabled ? "true" : "false",
                left, right, static_cast<long>(voltage_left), static_cast<long>(voltage_right),
                output_enabled ? "true" : "false", mode != DISABLE ? "true" : "false",
                static_cast<int>(task_state));
    std::fflush(stdout);
  }
  // Match PID::exit_condition(vector<Motor>) current protection, but do not
  // evaluate intermediate-point settling/velocity exits (which reset all timers).
  static exit_output current_only(PID& pid, Drive& drive) {
    if (pid.exit.mA_timeout == 0) return RUNNING;
    pid.is_mA = drive.left_motors[0].is_over_current() ||
                drive.right_motors[0].is_over_current();
    if (pid.is_mA) {
      pid.l += util::DELAY_TIME;
      if (pid.l > pid.exit.mA_timeout) {
        pid.timers_reset();
        return mA_EXIT;
      }
    } else {
      pid.l = 0;
    }
    return RUNNING;
  }
  static void settling_timers_reset(PID& pid) {
    // Keep l/is_mA: current protection must span the final-waypoint boundary.
    pid.i = pid.j = pid.k = pid.m = 0;
  }
  static bool wait_for_final_waypoint(Drive& drive) {
    if (drive.mode != PURE_PURSUIT || drive.pp_movements.empty()) return true;
    while (drive.pp_index != static_cast<int>(drive.pp_movements.size()) - 1) {
      pros::delay(util::DELAY_TIME);
      const auto xy = current_only(drive.xyPID, drive);
      const auto angle = current_only(drive.current_a_odomPID, drive);
      if (&drive == &chassis) {
        observe(&drive.xyPID, xy);
        observe(&drive.current_a_odomPID, angle);
      }
      if (xy == mA_EXIT || angle == mA_EXIT) {
        drive.interfered = true;
        return false;
      }
    }
    settling_timers_reset(drive.xyPID);
    settling_timers_reset(drive.current_a_odomPID);
    return true;
  }
  static Snapshot capture(Drive& drive) {
    Snapshot s;
    s.elapsed = pros::millis() - start_ms;
    s.internal = drive.odom_current;
    s.index = drive.pp_index;  // Sample once; path storage is stable during wait.
    s.last_index = static_cast<int>(drive.pp_movements.size()) - 1;
    if (s.index >= 0 && s.index <= s.last_index) s.active = drive.pp_movements[s.index].target;
    s.xy_error = drive.xyPID.error;
    s.angle_error = drive.current_a_odomPID.error;
    return s;
  }
  static void observe(PID* pid, exit_output result) {
    if (!recording || result == RUNNING) return;
    ExitEvent* event = pid == &chassis.xyPID ? &xy_event :
                       pid == &chassis.current_a_odomPID ? &angle_event : nullptr;
    if (event == nullptr || event->seen) return;
    event->state = capture(chassis);
    event->reason = result;
    event->order = ++event_order;
    event->seen = true;
  }
};
}  // namespace ez

void motion_controller_snapshot_start() {
  const auto began = pros::millis();
  pros::Task snapshot_task([began]() {
    const auto elapsed = pros::millis() - began;
    if (elapsed < 1500) pros::delay(1500 - elapsed);
    ez::MotionExitTrace::controller_snapshot(began);
  }, "One-shot motion snapshot");
}

// GNU ld --wrap observes the exact overload used by native Drive::pid_wait().
// Forward exactly once, preserve its result, and never call exit_condition again.
extern "C" ez::exit_output __real__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
    ez::PID*, std::vector<pros::Motor>, bool);
extern "C" ez::exit_output __wrap__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
    ez::PID* pid, std::vector<pros::Motor> motors, bool print) {
  if (pid == &chassis.turnPID && chassis.drive_mode_get() == ez::TURN) {
    // Linear acceleration is not a stall measurement for an in-place turn.
    // Scope the native toggle to this evaluation; other modes retain their setting.
    const bool secondary_enabled = pid->velocity_sensor_secondary_toggle_get();
    pid->velocity_sensor_secondary_toggle_set(false);
    const auto result = pid->exit_condition_turn_trace(std::move(motors), print);
    pid->velocity_sensor_secondary_toggle_set(secondary_enabled);
    turn_velocity_trace_record_result(result);  // Observe only; return the same native result.
    return result;
  }
  const auto result = __real__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
      pid, std::move(motors), print);
  ez::MotionExitTrace::observe(pid, result);
  return result;
}

void motion_exit_trace_arm() {
  page_mutex.take(); pages_ready = false; page_mutex.give();
  xy_event = {}; angle_event = {}; event_order = 0;
  armed = true;
  recording = false;
  previous_print = chassis.pid_print_toggle_get();
  chassis.pid_print_toggle(false);  // No per-waypoint terminal output during motion.
  odom_trace_screen_show("12 IN +X / SPEED 30 TEST");
  pros::lcd::set_text(4, "Final result after pid_wait");
}
void motion_exit_trace_begin(const ez::odom& movement) {
  if (!armed) return;
  target = movement.target;
  interfered_before = chassis.interfered;
  start_ms = pros::millis();
  recording = true;
}
void motion_exit_trace_finish() {
  if (!recording) return;
  final_state = ez::MotionExitTrace::capture(chassis);
  final_imu = chassis.drive_imu_get();
  interfered_after = chassis.interfered;
  recording = false;
  armed = false;
  // Preserve the wait-return snapshot, then stop residual PID output for this test.
  chassis.drive_set(0, 0);
  chassis.pid_print_toggle(previous_print);
}
void motion_exit_trace_show() {
  // Wait out any in-flight normal LCD draw, then disable that writer until driver mode.
  odom_display_freeze();
  page_mutex.take();
  const auto game = nav_internal_to_game_pose(final_state.internal);
  std::snprintf(pages[0][0], 96, "EXIT TRACE  INTERFERED: %s", interfered_after ? "YES" : "NO");
  std::snprintf(pages[0][1], 96, "FINAL X:%.2f Y:%.2f H:%.2f", game.x, game.y, game.theta);
  std::snprintf(pages[0][2], 96, "WP:%d/%d ACTIVE:%.2f,%.2f", final_state.index, final_state.last_index, final_state.active.x, final_state.active.y);
  std::snprintf(pages[0][3], 96, "FINAL TARGET: %.2f,%.2f", target.x, target.y);
  const ExitEvent* events[] = {&xy_event, &angle_event};
  const char* names[] = {"XY", "ANG"};
  for (int n = 0; n < 2; ++n) {
    const auto& event = *events[n];
    const int row = 4 + 2 * n;
    std::snprintf(pages[0][row], 96, "%s EXIT: %s", names[n], event.seen ? reason_name(event.reason) : "NONE");
    if (event.seen)
      std::snprintf(pages[0][row + 1], 96, "TIME:%lu ms ORDER:%d", static_cast<unsigned long>(event.state.elapsed), event.order);
    else
      std::snprintf(pages[0][row + 1], 96, "NO EXIT EVENT CAPTURED");
  }
  page = 0; pages_ready = true; draw_page();
  page_mutex.give();

  // Called once by autonomous(), after pid_wait() and the existing drive stop.
  std::printf(
      "\n==============================\n"
      "12-INCH +X MOTION RESULT\n"
      "==============================\n"
      "FINAL PUBLIC:\nX = %.3f\nY = %.3f\nH = %.3f\n\n"
      "WAYPOINT:\ncurrent index = %d\ntotal/final index = %d\n"
      "active target = (%.3f, %.3f)\nfinal target = (%.3f, %.3f)\n\n",
      game.x, game.y, game.theta, final_state.index, final_state.last_index,
      final_state.active.x, final_state.active.y, target.x, target.y);
  const char* terminal_names[] = {"XY", "ANGULAR"};
  for (int n = 0; n < 2; ++n) {
    const auto& event = *events[n];
    std::printf("FIRST %s EXIT:\nreason = %s\n", terminal_names[n],
                event.seen ? reason_name(event.reason) : "NO EXIT EVENT CAPTURED");
    if (event.seen)
      std::printf("time_ms = %lu\norder = %d\n\n",
                  static_cast<unsigned long>(event.state.elapsed), event.order);
    else
      std::printf("time_ms = N/A\norder = N/A\n\n");
  }
  std::printf("INTERFERED = %s\n==============================\n",
              interfered_after ? "true" : "false");
  std::fflush(stdout);
}
// Only the original object's entry-point symbol is renamed by objcopy.
// Execute its unchanged native wait instructions, with exit calls observed above.
extern "C" void ez_trace_native_pid_wait(ez::Drive*);
extern "C" void __wrap__ZN2ez5Drive8pid_waitEv(ez::Drive* drive) {
  if (!ez::MotionExitTrace::wait_for_final_waypoint(*drive)) return;
  ez_trace_native_pid_wait(drive);
}
