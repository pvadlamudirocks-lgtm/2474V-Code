/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "pros/misc.hpp"
#include "EZ-Template/path_trace_hook.hpp"
#include <atomic>
#include "EZ-Template/motion_output_safety.hpp"
#include "EZ-Template/navigation_hooks.hpp"

namespace {
std::atomic<ez::PathTraceHook> path_trace_hook{nullptr};
std::atomic<ez::CommandObserver> command_observer{nullptr};
std::atomic<ez::DriveTelemetryObserver> drive_telemetry_observer{nullptr};
pros::Mutex navigation_mutex; // Short controller/vector mutation serialization only.
std::atomic<bool> emergency_stop{false};
ez::output_safety::Latch output_fault;
ez::output_safety::DriverOutputPermission driver_output_permission;
std::atomic<ez::Drive*> turn_trace_drive{nullptr};
ez::TurnOutputTrace turn_trace;
void trace_zero(ez::Drive* drive, const char* reason) {
  if (turn_trace_drive.load() != drive) return;
  ++turn_trace.zero_count;
  turn_trace.command_left = turn_trace.command_right = 0;
  turn_trace.mode = static_cast<int>(drive->drive_mode_get());
  if (turn_trace.zero_count == 1) turn_trace.zero_reason = reason;
}
std::atomic<ez::Drive*> geometry_drive{nullptr};
geometry_turn::Deadline geometry_deadline; // Access only under navigation_mutex.
ez::TurnSensorCheck geometry_sensors_ok = nullptr; // Same mutex; opt-in only.
int geometry_max_power = 127; // Optional diagnostic cap, under navigation_mutex.
std::atomic<geometry_turn::Result> geometry_result{geometry_turn::Result::Idle};
bool geometry_tick(ez::Drive* drive) {
  if (geometry_drive.load() != drive) return false;
  geometry_deadline.update(pros::millis(), ez::autonomous_cancelled());
  if (geometry_deadline.result == geometry_turn::Result::Running && geometry_sensors_ok) {
    const bool healthy = geometry_sensors_ok();
    // Recheck elapsed time/cancellation after device reads, before any motor output.
    geometry_deadline.update(pros::millis(), ez::autonomous_cancelled(), healthy);
  }
  if (!geometry_deadline.must_stop()) return false;
  drive->drive_set(0,0); // Existing controller owner; no logging or outer wait.
  trace_zero(drive, "DEADLINE_OR_SENSOR_OR_CANCEL");
  geometry_result.store(geometry_deadline.result); // Published after stop command.
  return true; // Stay stopped until explicit cleanup disarms this test.
}
}
void ez::path_trace_hook_set(PathTraceHook hook) { path_trace_hook.store(hook); }
void ez::command_observer_set(CommandObserver observer) { command_observer.store(observer); }
void ez::drive_telemetry_observer_set(DriveTelemetryObserver observer) { drive_telemetry_observer.store(observer); }
void ez::navigation_lock() { navigation_mutex.take(); }
void ez::navigation_unlock() { navigation_mutex.give(); }
void ez::drive_stop(Drive* drive) {
  navigation_lock();
  drive->drive_set(0,0);
  trace_zero(drive, "EXPLICIT_STOP");
  if (turn_trace_drive.load() == drive) turn_trace_drive.store(nullptr);
  if (geometry_drive.load() == drive) {
    geometry_deadline.finish(pros::millis(), autonomous_cancelled());
    geometry_result.store(geometry_deadline.result);
    geometry_drive.store(nullptr);
    geometry_sensors_ok = nullptr;
  }
  navigation_unlock();
}
bool ez::geometry_turn_deadline_arm(Drive* drive, TurnSensorCheck sensors_ok, int max_power) {
  navigation_lock();
  const bool accepted = drive && max_power >= 1 && max_power <= 127 &&
                        !geometry_drive.load() && !autonomous_cancelled() &&
                        drive->drive_mode_get() == DISABLE;
  if (accepted) {
    geometry_sensors_ok = sensors_ok;
    geometry_max_power = max_power;
    geometry_deadline.arm(pros::millis(), geometry_turn::maximum_powered_ms);
    geometry_result.store(geometry_turn::Result::Running);
    geometry_drive.store(drive);
  }
  navigation_unlock();
  return accepted;
}
bool ez::geometry_turn_deadline_active(Drive* drive) { return geometry_drive.load() == drive; }
geometry_turn::Result ez::geometry_turn_deadline_result() { return geometry_result.load(); }
bool ez::geometry_turn_deadline_stopped(Drive* drive) {
  const auto result = geometry_result.load();
  return geometry_drive.load() == drive &&
         (result == geometry_turn::Result::Timeout || result == geometry_turn::Result::Interrupted || result == geometry_turn::Result::SensorFault);
}
void ez::motion_output_fault(output_safety::Fault reason) {
  // Also covers manually dispatched native tests that did not change competition flags.
  driver_output_permission.revoke();
  output_fault.set(reason); emergency_stop.store(true);
}
ez::output_safety::Fault ez::motion_output_fault_get() { return output_fault.get(); }
bool ez::driver_output_authorized() { return driver_output_permission.get(); }
void ez::driver_control_ready() {
  driver_output_permission.driver_ready(pros::competition::is_autonomous(), pros::competition::is_disabled());
}
void ez::autonomous_begin() {
  driver_output_permission.revoke();
  output_fault.reset(); emergency_stop.store(false);
}
void ez::autonomous_emergency_stop() { emergency_stop.store(true); }
bool ez::autonomous_cancelled() {
  const bool autonomous=pros::competition::is_autonomous();
  if(autonomous && pros::Controller(pros::E_CONTROLLER_MASTER).get_digital(pros::E_CONTROLLER_DIGITAL_B)==1)
    emergency_stop.store(true);
  return emergency_stop.load() || pros::competition::is_disabled() || !autonomous;
}
using namespace ez;

void Drive::ez_auto_task() {
  while (true) {
    // Basic competition/E-stop handling runs in this existing motor owner.
    // Explicit turn diagnostics alone can arm a deadline; unarmed behavior is unchanged.
    navigation_lock();
    if (!geometry_tick(this) && (drive_mode_get()!=DISABLE || pros::competition::is_autonomous()) && autonomous_cancelled()) { drive_set(0,0); trace_zero(this, "AUTONOMOUS_CANCELLED"); }
    navigation_unlock();
    ez_tracking_task();
    navigation_lock();
    if (geometry_tick(this)) {}
    else if((drive_mode_get()!=DISABLE || pros::competition::is_autonomous()) && autonomous_cancelled()) { drive_set(0,0); trace_zero(this, "AUTONOMOUS_CANCELLED"); }
    else switch (drive_mode_get()) {
      case DRIVE: drive_pid_task(); break;
      case TURN ... TURN_TO_POINT: turn_pid_task(); break;
      case SWING: swing_pid_task(); break;
      case POINT_TO_POINT: ptp_task(); break;
      case PURE_PURSUIT: pp_task(); break;
      case DISABLE: break;
      default: break;
    }
    // Catch a mode change or B press that occurred during a native calculation.
    if (!geometry_tick(this) && (drive_mode_get()!=DISABLE || pros::competition::is_autonomous()) && autonomous_cancelled()) { drive_set(0,0); trace_zero(this, "AUTONOMOUS_CANCELLED"); }
    navigation_unlock();
    util::AUTON_RAN = drive_mode_get() != DISABLE ? true : false;
    pros::delay(ez::util::DELAY_TIME);
  }
}
// Drive PID task
void Drive::drive_pid_task() {
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  // Compute PID
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  headingPID.compute(drive_imu_get());

  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }

  // Left and Right outputs
  double l_drive_out = leftPID.output;
  double r_drive_out = rightPID.output;

  // Scale leftPID and rightPID to slew (if slew is disabled, it returns max_speed)
  double max_slew_out = fmax(slew_left.output(), slew_right.output());
  double faster_side = fmax(fabs(l_drive_out), fabs(r_drive_out));
  if (faster_side > max_slew_out) {
    l_drive_out *= (max_slew_out / faster_side);
    r_drive_out *= (max_slew_out / faster_side);
  }

  // Toggle heading
  double imu_out = heading_on ? headingPID.output : 0;

  // Combine heading and drive
  double l_out = l_drive_out + imu_out;
  double r_out = r_drive_out - imu_out;

  // Vector scaling when combining drive and imu
  max_slew_out = fmax(slew_left.output(), slew_right.output());
  faster_side = fmax(fabs(l_out), fabs(r_out));
  if (faster_side > max_slew_out) {
    l_out *= (max_slew_out / faster_side);
    r_out *= (max_slew_out / faster_side);
  }

  // Validate before implicit floating-to-integer conversion.
  if(!std::isfinite(l_out)||!std::isfinite(r_out) || std::abs(l_out)>std::numeric_limits<int>::max() || std::abs(r_out)>std::numeric_limits<int>::max()) {
    motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return;
  }
  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
  if (drive_toggle) if (auto observer = command_observer.load()) observer(this, l_out, r_out);
  if (drive_toggle) if (auto observer = drive_telemetry_observer.load()) observer(this, l_out, r_out);
}

// Turn PID task
void Drive::turn_pid_task() {
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  // Compute PID if it's a normal turn
  if (mode == TURN) {
    turnPID.compute(drive_imu_get());
  }
  // Compute PID if we're turning to point
  else {
    double a_target = util::absolute_angle_to_point(point_to_face[!ptf1_running], odom_pose_get());  // Calculate the point for angle to face
    a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
    double error = a_target - odom_theta_get();
    turnPID.compute_error(error, odom_theta_get());
  }

  // Compute slew
  slew_turn.iterate(drive_imu_get());
  if (motion_output_fault_get()!=output_safety::Fault::None || !std::isfinite(turnPID.output)) { motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return; }

  // Clip gyroPID to max speed
  double gyro_out = util::clamp(turnPID.output, slew_turn.output(), -slew_turn.output());
  const double trace_after_slew = gyro_out;

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (turnPID.constants.ki != 0 && (fabs(turnPID.target_get()) > turnPID.constants.start_i && fabs(turnPID.error) < turnPID.constants.start_i)) {
    if (pid_turn_min_get() != 0)
      gyro_out = util::clamp(gyro_out, pid_turn_min_get(), -pid_turn_min_get());
  }

  const double trace_before_cap = gyro_out;
  // An explicitly armed diagnostic cap also bounds slew/minimum-turn output.
  if (geometry_drive.load() == this)
    gyro_out = util::clamp(gyro_out, geometry_max_power, -geometry_max_power);
  // Set motors
  if (drive_toggle)
    private_drive_set(gyro_out, -gyro_out);
  if (turn_trace_drive.load() == this) {
    turn_trace.sampled_ms = pros::millis();
    turn_trace.pid = turnPID.output;
    turn_trace.slew_limit = slew_turn.output();
    turn_trace.after_slew = trace_after_slew;
    turn_trace.before_cap = trace_before_cap;
    turn_trace.turn_left = static_cast<int>(gyro_out);
    turn_trace.turn_right = static_cast<int>(-gyro_out);
    turn_trace.drive_enabled = drive_toggle;
    turn_trace.mode = static_cast<int>(drive_mode_get());
    if (drive_toggle) {
      turn_trace.command_left = turn_trace.turn_left;
      turn_trace.command_right = turn_trace.turn_right;
    }
  }
  if (drive_toggle) if (auto observer = command_observer.load()) observer(this, gyro_out, -gyro_out);
}

// Swing PID task
void Drive::swing_pid_task() {
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  // Compute PID
  swingPID.compute(drive_imu_get());
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());

  // Compute slew
  double current = slew_swing_using_angle ? drive_imu_get() : (current_swing == LEFT_SWING ? drive_sensor_left() : drive_sensor_right());
  slew_swing.iterate(current);
  if (motion_output_fault_get()!=output_safety::Fault::None || !std::isfinite(swingPID.output)) { motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return; }

  // Clip swingPID to max speed
  double swing_out = util::clamp(swingPID.output, slew_swing.output(), -slew_swing.output());

  // Clip the speed of the turn when the robot is within StartI, only do this when target is larger then StartI
  if (swingPID.constants.ki != 0 && (fabs(swingPID.target_get()) > swingPID.constants.start_i && fabs(swingPID.error) < swingPID.constants.start_i)) {
    if (pid_swing_min_get() != 0)
      swing_out = util::clamp(swing_out, pid_swing_min_get(), -pid_swing_min_get());
  }

  // Set the motors powers, and decide what to do with the "still" side of the drive
  double opposite_output = 0;
  double scale = swing_out / max_speed;
  if (drive_toggle) {
    // Check if left or right swing, then set motors accordingly
    if (current_swing == LEFT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? rightPID.output : (swing_opposite_speed * scale);
      if(!std::isfinite(swing_out)||!std::isfinite(opposite_output) || std::abs(swing_out)>std::numeric_limits<int>::max() || std::abs(opposite_output)>std::numeric_limits<int>::max()) {
        motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return;
      }
      private_drive_set(swing_out, opposite_output);
  if (drive_toggle) if (auto observer = command_observer.load()) observer(this, swing_out, opposite_output);
    } else if (current_swing == RIGHT_SWING) {
      opposite_output = swing_opposite_speed == 0 ? leftPID.output : -(swing_opposite_speed * scale);
      if(!std::isfinite(swing_out)||!std::isfinite(opposite_output) || std::abs(swing_out)>std::numeric_limits<int>::max() || std::abs(opposite_output)>std::numeric_limits<int>::max()) {
        motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return;
      }
      private_drive_set(opposite_output, -swing_out);
  if (drive_toggle) if (auto observer = command_observer.load()) observer(this, opposite_output, -swing_out);
    }
  }
}

// Odom To Point Task
void Drive::ptp_task() {
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  // Compute slew
  slew_left.iterate(drive_sensor_left());
  slew_right.iterate(drive_sensor_right());
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  double max_slew_out = fmax(slew_left.output(), slew_right.output());

  const auto pose_now=odom_pose_get();
  if(point_to_face.size()<2 || !std::isfinite(pose_now.x) || !std::isfinite(pose_now.y) ||
     !std::isfinite(pose_now.theta) || !std::isfinite(odom_target.x) || !std::isfinite(odom_target.y) ||
     !std::isfinite(xy_delta_fake) || !std::isfinite(new_current_fake) || !std::isfinite(past_target) ||
     !std::isfinite(odom_imu_start) || !output_safety::pose_valid(point_to_face[0]) || !output_safety::pose_valid(point_to_face[1])) { motion_output_fault(output_safety::Fault::State); drive_set(0,0); return; }
  // Decide if we've past the target or not
  double temp_target = is_past_target(odom_target, odom_pose_get());        // Use this instead of distance formula to fix impossible movements
  int dir = (current_drive_direction == REV ? -1 : 1);                      // If we're going backwards, add a -1
  int flipped = util::sgn(temp_target) != util::sgn(past_target) ? -1 : 1;  // Check if we've flipped directions to what we started

  // Compute xy PID
  new_current_fake += xy_delta_fake * ((dir * flipped));  // Create a "current sensor value" for the PID to calculate off of
  xyPID.compute_error(fabs(temp_target) * dir * flipped, new_current_fake);

  // Compute angle
  pose ptf = point_to_face[!ptf1_running];
  double a_target = util::absolute_angle_to_point(ptf, odom_pose_get());  // Calculate the point for angle to face
  a_target = new_turn_target_compute(a_target, odom_imu_start, current_angle_behavior);
  double wrapped_a_target = a_target - odom_theta_get();
  current_a_odomPID.compute_error(wrapped_a_target, odom_theta_get());
  // printf("shortest_a_target: %.2f      error: %.2f\n", a_target, wrapped_a_target);

  const bool bias_enabled=odom_turn_bias_enabled();
  const double scale=1.0-((1.0-cos(util::to_rad(current_a_odomPID.error)))/odom_turn_bias_amount);
  const auto mixed=output_safety::mix(xyPID.output,current_a_odomPID.output,scale,bias_enabled,
                                     slew_left.output(),slew_right.output(),max_speed);
  if(!mixed.valid || !std::isfinite(temp_target) || !std::isfinite(past_target) ||
     !std::isfinite(new_current_fake) || !std::isfinite(a_target) || !std::isfinite(wrapped_a_target)) {
    motion_output_fault(output_safety::Fault::Output); drive_set(0,0); return;
  }
  max_slew_out=mixed.limit;
  const double xy_out=mixed.xy, a_out=mixed.angular, l_out=mixed.left, r_out=mixed.right;

  // printf("lr out (%.2f, %.2f)   xy/a(%.2f, %.2f)   lr slew (%.2f, %.2f)\n", l_out, r_out, xy_out, a_out, slew_left.output(), slew_right.output());
  // printf("max_slew_out %.2f      headingerr: %.2f\n", max_slew_out, aPID.error);
  // printf("lr(%.2f, %.2f)   xy_raw: %.2f   xy_out: %.2f   heading_out: %.2f      max_slew_out: %.2f\n", l_out, r_out, xyPID.output, xy_out, current_a_odomPID.output, max_slew_out);
  // printf("xy(%.2f, %.2f, %.2f)   xyPID: %.2f   aPID: %.2f     dir: %i   sgn: %i   past_target: %i    is_past_target: %i   is_past_using_xy: %i      fake_xy(%.2f, %.2f, %.2f)\n", odom_x_get(), odom_y_get(), odom_theta_get(), xyPID.target_get(), current_a_odomPID.target_get(), dir, flipped, past_target, (int)is_past_target(odom_target, odom_pose_get()), is_past_target_using_xy, fake_x, fake_y, util::to_deg(fake_angle));
  // printf("xy(%.2f, %.2f, %.2f)   xyPID: %.2f   aPID: %.2f   ptf:(%.2f, %.2f)   xy/a(%.2f, %.2f)   lr(%.2f, %.2f)   fake xy: %.2f\n", odom_x_get(), odom_y_get(), odom_theta_get(), xyPID.error, current_a_odomPID.error, ptf.x, ptf.y, xy_out, a_out, l_out, r_out, new_current_fake);

  // Set motors
  if (drive_toggle)
    private_drive_set(l_out, r_out);
  if (drive_toggle) if (auto observer = command_observer.load()) observer(this, l_out, r_out);

  // Read-only diagnostic callback. The native controller calculations and motor
  // write above are unchanged. A null callback has no capture overhead.
  if (auto hook = path_trace_hook.load()) {
    hook(this, {temp_target, max_slew_out, scale, xy_out, a_out, l_out, r_out,
                dir, flipped, drive_toggle, a_target, odom_target.x, odom_target.y,
                odom_target.theta, bias_enabled, slew_left.enabled(), slew_right.enabled()});
  }

  // This is for wait_until
  leftPID.compute(drive_sensor_left());
  rightPID.compute(drive_sensor_right());
}

void Drive::boomerang_task() {
  if (pp_index<0 || static_cast<size_t>(pp_index)>=pp_movements.size()) {
    motion_output_fault(output_safety::Fault::Path); drive_set(0,0); return;
  }
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  int target_index = pp_index;
  pose target = pp_movements[target_index].target;

  const int dir=current_drive_direction==REV?-1:current_drive_direction==FWD?1:0;
  pose temp;
  if(!output_safety::boomerang_geometry(target,odom_pose_get(),odom_boomerang_dlead_get(),
      max_boomerang_distance,odom_look_ahead_get(),dir,
      [](pose a,pose b) { return util::distance_to_point(a,b); },
      [](double distance,pose p) { return util::vector_off_point(distance,p); },temp)) {
    // This is the existing motor owner, already under navigation_mutex.
    // Publish cancellation first so all native pid_wait loops can return.
    motion_output_fault(output_safety::Fault::State);
    drive_set(0,0);
    return;
  }

  if (odom_target.x != temp.x || odom_target.y != temp.y) {
    bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
    raw_pid_odom_ptp_set({temp, pp_movements[target_index].drive_direction, pp_movements[target_index].max_xy_speed}, slew_on);
  }

  // printf("cur(%.2f, %.2f, %.2f)   tar(%.2f, %.2f, %.2f)   h %.2f  \n", odom_x_get(), odom_y_get(), odom_theta_get(), temp.x, temp.y, temp.theta, h);

  ptp_task();
}

void Drive::pp_task() {
  if (pp_index<0 || static_cast<size_t>(pp_index)>=pp_movements.size()) {
    motion_output_fault(output_safety::Fault::Path); drive_set(0,0); return;
  }
  if (motion_output_fault_get()!=output_safety::Fault::None) { drive_set(0,0); return; }
  if (fabs(util::distance_to_point(pp_movements[pp_index].target, odom_pose_get())) < odom_look_ahead_get()) {
    if (pp_index < pp_movements.size() - 1) {
      pp_index = pp_index >= pp_movements.size() - 1 ? pp_index : pp_index + 1;
      bool slew_on = slew_left.enabled() || slew_right.enabled() ? true : false;
      if (!current_slew_on) slew_on = false;
      raw_pid_odom_ptp_set(pp_movements[pp_index], slew_on);
    }
  }

  if (pp_movements[pp_index].target.theta != ANGLE_NOT_SET) {
    boomerang_task();
  } else {
    ptp_task();
  }
}



void ez::turn_output_trace_begin(Drive* drive) {
  navigation_lock();
  turn_trace = TurnOutputTrace{};
  turn_trace_drive.store(drive);
  navigation_unlock();
}
void ez::turn_output_trace_end() {
  navigation_lock();
  turn_trace_drive.store(nullptr);
  navigation_unlock();
}
ez::TurnOutputTrace ez::turn_output_trace_get() {
  navigation_lock();
  const auto copy = turn_trace;
  navigation_unlock();
  return copy;
}
void ez::turn_output_trace_exit(const PID* pid, const char* branch, double secondary, int k, int m, int l) {
  auto* drive = turn_trace_drive.load();
  if (!drive || pid != &drive->turnPID) return;
  navigation_lock();
  if (turn_trace_drive.load() == drive) {
    turn_trace.exit_branch = branch;
    turn_trace.exit_error = pid->error;
    turn_trace.exit_derivative = pid->derivative;
    turn_trace.exit_secondary = secondary;
    turn_trace.exit_k = k;
    turn_trace.exit_m = m;
    turn_trace.exit_l = l;
  }
  navigation_unlock();
}

bool ez::path_trace_hook_claim(PathTraceHook hook) {
  PathTraceHook empty=nullptr;
  return path_trace_hook.compare_exchange_strong(empty,hook);
}
void ez::path_trace_hook_release(PathTraceHook hook) {
  path_trace_hook.compare_exchange_strong(hook,nullptr);
}
bool ez::autonomous_cancelled_snapshot() {
  return emergency_stop.load() || pros::competition::is_disabled() ||
      !pros::competition::is_autonomous();
}
