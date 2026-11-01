#include "EZ-Template/navigation_hooks.hpp"
#include "navigation.hpp"
void nav_observe_exit(ez::exit_output result);
#include "motion_exit_trace.hpp"
#include "path_motion_trace.hpp"
#include "autons.hpp"
#include "full_system_diag.hpp"
#include <cstdio>
#include <cmath>
#include <utility>
#include <atomic>

namespace {
// One blocking autonomous caller; capture only, never print inside native waits.
struct DriveExitSample {
  ez::exit_output reason=ez::RUNNING;
  unsigned ms=0;
  double error=NAN, derivative=NAN, secondary=NAN, rpm[2]={NAN,NAN};
};
DriveExitSample drive_exits[2];
unsigned drive_wait_started=0;
bool drive_exit_recording=false; // Only the blocking caller uses exit capture.
struct MotorRow { double position=NAN, rpm=NAN; int mv=0, ma=0; };
struct DriveRow {
  unsigned ms=0;
  int left=0,right=0;
  double heading=NAN,left_pid=NAN,right_pid=NAN,heading_pid=NAN;
  MotorRow motor[4];
};
DriveRow drive_rows[90]; // At most one sample/100 ms; no serial output while moving.
std::atomic<unsigned> drive_row_count{0};
std::atomic<bool> drive_sampling{false};
void capture_drive_output(ez::Drive* drive,int left,int right) {
  if(drive!=&chassis || !drive_sampling.load(std::memory_order_acquire)) return;
  const unsigned count=drive_row_count.load(std::memory_order_relaxed);
  const unsigned ms=pros::millis()-drive_wait_started;
  if(count>=90 || (count && ms-drive_rows[count-1].ms<100)) return;
  DriveRow row;
  row.ms=ms; row.left=left; row.right=right; row.heading=drive->drive_imu_get();
  row.left_pid=drive->leftPID.output; row.right_pid=drive->rightPID.output;
  row.heading_pid=drive->headingPID.output;
  for(unsigned side=0;side<2;++side) {
    auto& motors=side ? drive->right_motors : drive->left_motors;
    for(unsigned i=0;i<2 && i<motors.size();++i) {
      auto& out=row.motor[side*2+i];
      out.position=motors[i].get_position(); out.rpm=motors[i].get_actual_velocity();
      out.mv=motors[i].get_voltage(); out.ma=motors[i].get_current_draw();
    }
  }
  drive_rows[count]=row;
  drive_row_count.store(count+1,std::memory_order_release);
}
void capture_drive_exit(ez::Drive* drive,bool left,int result) {
  if(drive!=&chassis) return;
  auto& out=drive_exits[left ? 0 : 1];
  const auto& pid=left ? drive->leftPID : drive->rightPID;
  out.reason=static_cast<ez::exit_output>(result);
  out.ms=pros::millis()-drive_wait_started;
  out.error=pid.error; out.derivative=pid.derivative;
  out.secondary=(left ? drive->leftPID : drive->rightPID).velocity_sensor_secondary_get();
  auto& motors=left ? drive->left_motors : drive->right_motors;
  for(unsigned i=0;i<motors.size() && i<2;++i) out.rpm[i]=motors[i].get_actual_velocity();
}
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
bool pose_recording = false;
ez::exit_output pose_xy_exit = ez::RUNNING, pose_angle_exit = ez::RUNNING;
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
      pros::delay(util::DELAY_TIME); if (!ez::wait_service_tick(&drive)) return false;
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
    path_motion_trace_exit(pid, result);  // Observe the same result, never reevaluate.
    if (pose_recording && result != RUNNING) {
      if (pid == &chassis.xyPID && pose_xy_exit == RUNNING) pose_xy_exit = result;
      if (pid == &chassis.current_a_odomPID && pose_angle_exit == RUNNING) pose_angle_exit = result;
    }
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

void pose_exit_trace_begin() {
  pose_xy_exit = pose_angle_exit = ez::RUNNING;
  pose_recording = true;
}

std::pair<ez::exit_output, ez::exit_output> pose_exit_trace_finish() {
  pose_recording = false;
  return {pose_xy_exit, pose_angle_exit};
}

// GNU ld --wrap observes the exact overload used by native Drive::pid_wait().
// Forward exactly once, preserve its result, and never call exit_condition again.
extern "C" ez::exit_output __real__ZN2ez3PID14exit_conditionEN4pros2v55MotorEb(ez::PID*,pros::Motor,bool);
extern "C" ez::exit_output __wrap__ZN2ez3PID14exit_conditionEN4pros2v55MotorEb(
    ez::PID* pid,pros::Motor motor,bool print) {
  const auto result=__real__ZN2ez3PID14exit_conditionEN4pros2v55MotorEb(pid,motor,print);
  full_system_diag_exit(pid,result);
  if(drive_exit_recording && result!=ez::RUNNING) {
    if(pid==&chassis.leftPID) capture_drive_exit(&chassis,true,result);
    else if(pid==&chassis.rightPID) capture_drive_exit(&chassis,false,result);
  }
  return result;
}
extern "C" ez::exit_output __real__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
    ez::PID*, std::vector<pros::Motor>, bool);
extern "C" ez::exit_output __wrap__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
    ez::PID* pid, std::vector<pros::Motor> motors, bool print) {
  if (pid == &chassis.turnPID && chassis.drive_mode_get() == ez::TURN) {
    // Linear acceleration is not a stall measurement for an in-place turn.
    // Scope the native toggle to this evaluation; other modes retain their setting.
    const bool secondary_enabled = pid->velocity_sensor_secondary_toggle_get();
    const int velocity_exit_time = pid->exit.velocity_exit_time;
    pid->velocity_sensor_secondary_toggle_set(false);
    // Skip both velocity branches only for this TURN evaluation. Discarding a
    // VELOCITY_EXIT afterward would already have reset angular/current timers.
    pid->exit.velocity_exit_time = 0;
    const auto result = pid->exit_condition_turn_trace(std::move(motors), print);
    full_system_diag_exit(pid,result);
    pid->exit.velocity_exit_time = velocity_exit_time;
    pid->velocity_sensor_secondary_toggle_set(secondary_enabled);
    nav_observe_exit(result);
    turn_velocity_trace_record_result(result);  // Observe only; return the same native result.
    return result;
  }
  const auto result = __real__ZN2ez3PID14exit_conditionESt6vectorIN4pros2v55MotorESaIS4_EEb(
      pid, std::move(motors), print);
  full_system_diag_exit(pid,result);
  nav_observe_exit(result);
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
  const bool diagnose=drive==&chassis && drive->drive_mode_get()==ez::DRIVE && !full_system_diag_recording();
  const bool interference_at_entry=drive->interfered;
  const unsigned began=pros::millis();
  double initial[4]={NAN,NAN,NAN,NAN};
  double start_left=NAN,start_right=NAN,target_left=NAN,target_right=NAN,start_imu=NAN;
  if(diagnose) {
    drive_exits[0]={}; drive_exits[1]={}; drive_wait_started=began;
    start_left=drive->drive_sensor_left(); start_right=drive->drive_sensor_right();
    target_left=drive->leftPID.target_get(); target_right=drive->rightPID.target_get();
    start_imu=drive->drive_imu_get();
    for(unsigned i=0;i<2;++i) {
      if(i<drive->left_motors.size()) initial[i]=drive->left_motors[i].get_position();
      if(i<drive->right_motors.size()) initial[i+2]=drive->right_motors[i].get_position();
    }
    drive_exit_recording=true;
    drive_row_count.store(0);
    drive_sampling.store(true,std::memory_order_release);
    ez::drive_telemetry_observer_set(capture_drive_output);
  }
  const bool printing=drive->pid_print_toggle_get();
  drive->pid_print_toggle(false); // Stop before potentially blocking diagnostics.
  if (ez::MotionExitTrace::wait_for_final_waypoint(*drive)) ez_trace_native_pid_wait(drive);
  const unsigned wait_ms=pros::millis()-began;
  if(diagnose) {
    drive_exit_recording=false;
    drive_sampling.store(false,std::memory_order_release);
    ez::drive_telemetry_observer_set(nullptr);
  }
  const bool stopped=ez::movement_stop();
  if (!stopped) drive->interfered = true;
  if(drive->interfered) ez::movement_cancel("native movement failure");
  drive->pid_print_toggle(printing);
  if(diagnose) {
    const double final_left=drive->drive_sensor_left(),final_right=drive->drive_sensor_right();
    const auto pose=drive->odom_pose_get();
    std::printf("DRIVE_DIAG wait_ms=%u through_stop_ms=%u stop_ack=%d fault=%s interfered=%d interfered_at_entry=%d\n"
                "DRIVE_DIAG encoder_in L start=%.4f target=%.4f final=%.4f delta=%.4f error=%.4f | R start=%.4f target=%.4f final=%.4f delta=%.4f error=%.4f\n"
                "DRIVE_DIAG imu_start=%.4f imu_final=%.4f pose=(%.4f,%.4f,%.4f)\n",
                wait_ms,pros::millis()-began,stopped,ez::movement_fault(),drive->interfered,interference_at_entry,
                start_left,target_left,final_left,final_left-start_left,target_left-final_left,
                start_right,target_right,final_right,final_right-start_right,target_right-final_right,
                start_imu,drive->drive_imu_get(),pose.x,pose.y,pose.theta);
    for(unsigned side=0;side<2;++side) {
      const auto& e=drive_exits[side];
      std::printf("DRIVE_DIAG %s exit=%s at_ms=%u pid_error=%.4f derivative=%.6f secondary_accel=%.6f rpm_at_exit=(%.2f,%.2f)\n",
                  side ? "R" : "L",reason_name(e.reason),e.ms,e.error,e.derivative,e.secondary,e.rpm[0],e.rpm[1]);
      auto& motors=side ? drive->right_motors : drive->left_motors;
      for(unsigned i=0;i<motors.size() && i<2;++i)
        std::printf("DRIVE_DIAG motor_port=%d position_delta_native_units=%.4f rpm_after_stop=%.2f\n",
                    motors[i].get_port(),motors[i].get_position()-initial[side*2+i],motors[i].get_actual_velocity());
    }
    std::printf("DRIVE_SAMPLE columns: ms,L_cmd,R_cmd,imu,L_PID,R_PID,H_PID; then L0,L1,R0,R1 each: position_native,rpm,voltage_readback_mV,current_mA\n");
    const unsigned count=drive_row_count.load(std::memory_order_acquire);
    for(unsigned i=0;i<count;++i) {
      const auto& row=drive_rows[i];
      std::printf("DRIVE_SAMPLE %u,%d,%d,%.3f,%.3f,%.3f,%.3f",row.ms,row.left,row.right,row.heading,row.left_pid,row.right_pid,row.heading_pid);
      for(const auto& motor:row.motor) std::printf(",%.3f,%.2f,%d,%d",motor.position,motor.rpm,motor.mv,motor.ma);
      std::printf("\n");
    }
  }
}


