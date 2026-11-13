#include "dev_tests.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "EZ-Template/path_trace_hook.hpp"
#include <atomic>
#include "EZ-Template/navigation_hooks.hpp"
#include <cmath>
#include <cstdio>

namespace {
constexpr unsigned SAMPLE_MS = 25, CAPACITY = 220;
constexpr double TARGET_X = 6, TARGET_Y = -12;
struct Commands {
  int left = 0, right = 0, left_sign = 0, right_sign = 0;
  unsigned ms = 0, updates = 0, left_reversals = 0, right_reversals = 0;
};
// Accessed under the existing navigation mutex: observer runs in the motor owner.
Commands commands;
struct MotorSample { double rpm = NAN; int mv = PROS_ERR, ma = PROS_ERR; };
struct Row {
  unsigned ms = 0;
  ez::pose pose{};
  double imu = NAN;
  Commands command{};
  MotorSample motors[4];
  bool interfered = false, after_stop = false;
};
Row rows[CAPACITY];
unsigned began = 0, row_count = 0, dropped = 0, last_sample = 0;
constexpr unsigned FRAME_CAPACITY = 256;
struct NativeExits {
  int xy = -1, angular = -1; // -1 means not observed, never a fabricated exit.
  unsigned ms = 0, evaluations = 0;
};
struct ControllerRow {
  unsigned ms = 0, events = 0;
  ez::pose pose{};
  ez::PathControlFrame frame{};
  double xy_raw = 0, angular_raw = 0, angular_error = 0;
  bool bias_enabled = false;
  NativeExits exits{};
};
ControllerRow frames[FRAME_CAPACITY], previous_frame;
unsigned frame_count=0, frame_dropped=0, frame_last_ms=0;
unsigned reversal_events=0, with_plane=0, with_flip=0, with_xy_raw=0;
unsigned with_negative_bias=0, with_angular_sign=0, with_angular_change=0;
int last_left=0,last_right=0,last_plane=0,last_xy_raw=0,last_xy_scaled=0,last_angular=0;
bool have_frame=false;
NativeExits native_exits;
std::atomic<bool> capturing{false};
// All state above except capturing is protected by the existing navigation mutex.
// Frame callback is already under that mutex; no hardware reads or terminal IO.
bool sign_change(double value, int& prior, double epsilon) {
  const int sign = value>epsilon ? 1 : value < -epsilon ? -1 : 0;
  const bool changed = sign && prior && sign!=prior;
  if(sign) prior=sign;
  return changed;
}
void observe_native_exits(ez::Drive* drive,int xy,int angular) {
  if(drive!=&chassis || !capturing.load()) return;
  ez::navigation_lock();
  if(capturing.load()) {
    native_exits.xy=xy; native_exits.angular=angular;
    native_exits.ms=pros::millis()-began;
    ++native_exits.evaluations;
  }
  ez::navigation_unlock();
}
void observe_frame(ez::Drive* drive,const ez::PathControlFrame& frame) {
  if(drive!=&chassis || !capturing.load()) return;
  ControllerRow r;
  r.ms=pros::millis()-began;
  r.pose=nav_internal_to_game_pose(drive->odom_pose_get()); // Cached odometry only.
  r.frame=frame;
  r.xy_raw=drive->xyPID.output;
  r.angular_raw=drive->current_a_odomPID.output;
  r.angular_error=drive->current_a_odomPID.error;
  r.bias_enabled=frame.bias_enabled;
  r.exits=native_exits;
  const bool left=sign_change(static_cast<int>(frame.left),last_left,1);
  const bool right=sign_change(static_cast<int>(frame.right),last_right,1);
  const bool plane=sign_change(frame.signed_target,last_plane,0.000001);
  const bool flip=have_frame && frame.flipped!=previous_frame.frame.flipped;
  const bool xy=sign_change(r.xy_raw,last_xy_raw,1);
  const bool scaled=sign_change(frame.xy_scaled,last_xy_scaled,1);
  const bool negative=r.bias_enabled && frame.bias_scale<0;
  const bool angular=sign_change(r.angular_raw,last_angular,1);
  const bool angular_change=have_frame && std::fabs(r.angular_raw-previous_frame.angular_raw)>1;
  r.events=(left?1:0)|(right?2:0)|(plane?4:0)|(flip?8:0)|(xy?16:0)|
           (scaled?32:0)|(negative?64:0)|(angular?128:0)|(angular_change?256:0);
  if(left || right) {
    ++reversal_events;
    with_plane+=plane; with_flip+=flip; with_xy_raw+=xy;
    with_negative_bias+=negative; with_angular_sign+=angular;
    with_angular_change+=angular_change;
  }
  // Keep nominal 25 ms frames plus command/plane/sign events. Summaries still
  // count every controller tick if storage fills. Never allocate or print here.
  if(!have_frame || r.ms-frame_last_ms>=SAMPLE_MS || (r.events & 191u)) {
    frame_last_ms=r.ms;
    if(frame_count<FRAME_CAPACITY) frames[frame_count++]=r;
    else ++frame_dropped;
  }
  previous_frame=r; have_frame=true;
}
const char* exit_name(int value) {
  if(value<0) return "UNOBSERVED";
  switch(static_cast<ez::exit_output>(value)) {
    case ez::RUNNING:return "RUNNING";
    case ez::SMALL_EXIT:return "SMALL";
    case ez::BIG_EXIT:return "BIG";
    case ez::VELOCITY_EXIT:return "VELOCITY";
    case ez::mA_EXIT:return "OVER_CURRENT";
    case ez::ERROR_NO_CONSTANTS:return "NO_CONSTANTS";
  }
  return "UNKNOWN";
}
void dump_controller_frames() {
  std::printf("RR_CAUSES reversal_ticks=%u same_tick_plane_cross=%u flipped_change=%u "
              "raw_xy_sign_change=%u negative_bias_active=%u angular_sign_change=%u "
              "angular_raw_delta_gt1=%u frames=%u dropped_frames=%u\n",
              reversal_events,with_plane,with_flip,with_xy_raw,with_negative_bias,
              with_angular_sign,with_angular_change,frame_count,frame_dropped);
  std::printf("RR_CAUSES counts overlap; same-tick correlation is not proof of causation. "
              "Signs ignore +/-1 output units and remember polarity across zero; plane epsilon=1e-6. "
              "events bits=1:Lrev,2:Rrev,4:plane,8:flipped,16:rawXY,32:scaledXY,64:negativeBias,128:angularSign,256:angularDelta. "
              "Angular target is native controller heading; native target heading sentinel is printed as NaN.\n");
  std::printf("RR_FRAME,ms,x_in,y_in,heading_deg,native_target_x_in,native_target_y_in,native_target_heading_deg,"
              "native_angular_target_deg,signed_plane,flipped,direction,angular_error_deg,"
              "bias_enabled,bias_multiplier,xy_raw,xy_scaled,angular_raw,angular_scaled,"
              "slew_limit,left_requested,right_requested,output_enabled,events,exit_sample_ms,"
              "exit_evaluations,native_xy,native_angular\n");
  for(unsigned i=0;i<frame_count;++i) {
    const auto& r=frames[i]; const auto& f=r.frame;
    std::printf("RR_FRAME,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%d,%d,%.4f,%d,"
                "%.5f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%u,%u,%u,%s,%s\n",
                r.ms,r.pose.x,r.pose.y,r.pose.theta,f.target_x,f.target_y,
                f.target_heading==ez::ANGLE_NOT_SET ? NAN : f.target_heading,f.angular_target,f.signed_target,
                f.flipped,f.direction_sign,r.angular_error,r.bias_enabled,f.bias_scale,
                r.xy_raw,f.xy_scaled,r.angular_raw,f.angular_scaled,f.slew_limit,
                f.left,f.right,f.output_enabled,r.events,r.exits.ms,r.exits.evaluations,
                exit_name(r.exits.xy),exit_name(r.exits.angular));
    if(i%4==3) {std::fflush(stdout);pros::delay(10);} // Post-stop serial pacing only.
  }
}
bool healthy() { return chassis.odom_enabled() && nav_sensors_ready(); }

void observe_commands(ez::Drive* drive, int left, int right) {
  if (drive != &chassis) return;
  commands.left = left; commands.right = right;
  commands.ms = pros::millis() - began;
  ++commands.updates;
  const int ls = left > 1 ? 1 : left < -1 ? -1 : 0;
  const int rs = right > 1 ? 1 : right < -1 ? -1 : 0;
  if (ls && commands.left_sign && ls != commands.left_sign) ++commands.left_reversals;
  if (rs && commands.right_sign && rs != commands.right_sign) ++commands.right_reversals;
  if (ls) commands.left_sign = ls;
  if (rs) commands.right_sign = rs;
}
void sample(bool after_stop = false) {
  if (row_count == CAPACITY) { ++dropped; return; }
  Row row;
  row.ms = pros::millis() - began;
  row.after_stop = after_stop;
  ez::navigation_lock();
  row.command = commands; // No sensor reads or printing while holding this lock.
  ez::navigation_unlock();
  row.pose = nav_pose_get();
  row.imu = chassis.drive_imu_get();
  row.interfered = chassis.interfered;
  for (unsigned side=0; side<2; ++side) {
    auto& motors = side ? chassis.right_motors : chassis.left_motors;
    for (unsigned i=0; i<2 && i<motors.size(); ++i) {
      auto& out = row.motors[side*2+i];
      out.rpm = motors[i].get_actual_velocity();
      out.mv = motors[i].get_voltage();
      out.ma = motors[i].get_current_draw();
    }
  }
  rows[row_count++] = row;
}
bool service(ez::Drive*) {
  const unsigned now = pros::millis();
  if (now-last_sample >= SAMPLE_MS) {
    last_sample = now;
    sample();
  }
  return true; // Existing cancellation/deadline checks run before this callback.
}
}

void run_reverse_right_diag() {
  ez::drive_stop(&chassis);
  if (ez::autonomous_cancelled() || !healthy() ||
      chassis.left_motors.size()!=2 || chassis.right_motors.size()!=2) {
    std::printf("REVERSE_RIGHT_DIAG NOT STARTED: cancelled, sensor fault, or motor count mismatch; STOP_COMMANDED=YES\n");
    return;
  }
  nav_set_start(0,0,0); // Same starting reference as the existing development tests.
  if (!healthy() || !nav_validate_pose(0,0,0,0.5,0.5) ||
      std::fabs(std::remainder(chassis.drive_imu_get(),360.0))>0.5) {
    std::printf("REVERSE_RIGHT_DIAG NOT STARTED: start readback failed; STOP_COMMANDED=YES\n");
    return;
  }

  const bool printing = chassis.pid_print_toggle_get();
  chassis.pid_print_toggle(false);
  row_count = dropped = 0;
  frame_count=frame_dropped=frame_last_ms=0;
  reversal_events=with_plane=with_flip=with_xy_raw=0;
  with_negative_bias=with_angular_sign=with_angular_change=0;
  last_left=last_right=last_plane=last_xy_raw=last_xy_scaled=last_angular=0;
  have_frame=false;
  native_exits=NativeExits{};
  began = last_sample = pros::millis();
  ez::navigation_lock();
  commands = Commands{};
  ez::navigation_unlock();

  if (!ez::geometry_turn_deadline_arm(&chassis, healthy)) {
    ez::drive_stop(&chassis);

    chassis.pid_print_toggle(printing);
    std::printf("REVERSE_RIGHT_DIAG NOT STARTED: deadline arm rejected; STOP_COMMANDED=YES\n");
    return;
  }
  // Reuse the existing 5000 ms motor-owner deadline. Its stop is mode-independent;
  // the optional turn-only power clamp does not alter this PTP movement.
  capturing.store(true);
  ez::native_odom_exit_observer_set(observe_native_exits);
  ez::path_trace_hook_set(observe_frame);
  ez::command_observer_set(observe_commands);
  ez::wait_service_set(service);
  nav_go_to_reverse(TARGET_X,TARGET_Y,40);
  ez::drive_stop(&chassis); // First operation after motion returns: disables PID.
  const unsigned stopped_ms = pros::millis()-began;
  const auto deadline = ez::geometry_turn_deadline_result();
  ez::wait_service_set(nullptr);
  ez::command_observer_set(nullptr);
  reverse_right_diag_cancel_capture();
  const auto exits = native_exits;
  sample(true); // Final measured row is after zero; requested values remain last PTP commands.
  chassis.pid_print_toggle(printing);

  const char* status = deadline == geometry_turn::Result::Timeout ? "TIMEOUT" :
      deadline == geometry_turn::Result::Interrupted || ez::autonomous_cancelled() ? "CANCELLED" :
      deadline == geometry_turn::Result::SensorFault ? "SENSOR_FAULT" :
      chassis.interfered ? "INTERFERED" : "WAIT_RETURNED";
  std::printf("REVERSE_RIGHT_DIAG target=(6,-12) in request_speed=40/127 sample_period_ms=25 "
              "cutoff_ms=5000 stopped_ms=%u status=%s STOP_COMMANDED=YES rows=%u dropped=%u\n",
              stopped_ms,status,row_count,dropped);
  std::printf("REVERSE_RIGHT_DIAG native_xy_exit=%s native_angular_exit=%s "
              "exit_evaluations=%u last_exit_ms=%u xy_completed=%d angular_completed=%d interfered=%d\n",
              exit_name(exits.xy),exit_name(exits.angular),exits.evaluations,exits.ms,
              exits.xy>=0 && exits.xy!=static_cast<int>(ez::RUNNING),
              exits.angular>=0 && exits.angular!=static_cast<int>(ez::RUNNING),chassis.interfered);
  dump_controller_frames();
  unsigned index=0;
  for (const auto* motors : {&chassis.left_motors,&chassis.right_motors})
    for (const auto& motor : *motors)
      std::printf("RR_MOTOR_MAP,index=%u,side=%s,port=%d,reversed=%d\n",
                  index++,motors==&chassis.left_motors ? "L" : "R",motor.get_port(),motor.is_reversed());
  std::printf("RR_UNITS: pose/errors=in; heading=deg; requested commands=-127..127; "
              "voltage=PROS readback mV, not inferred command; velocity=PROS rpm; current=mA. "
              "PROS movement telemetry uses configured motor reversals; current is magnitude. "
              "NaN/PROS_ERR values indicate unavailable data. Motor reads are sequential, not simultaneous.\n");
  std::printf("RR_NOTE: requested_L/R are last post-scaling native motor-write arguments, not RPM-derived. "
              "cmd_ms/updates identify stale samples; final row retains last PTP request although stop has commanded zero. "
              "Reversal counters observe every controller command, ignore +/-1 and remember polarity across zero. "
              "WAIT_RETURNED does not prove endpoint accuracy.\n");
  std::printf("RR_CSV,elapsed_ms,public_x_in,public_y_in,public_heading_deg,imu_heading_deg,"
              "error_x_in,error_y_in,cmd_ms,command_updates,requested_L_127,requested_R_127,"
              "L_reversals,R_reversals,L0_voltage_mV,L0_velocity_rpm,L0_current_mA,"
              "L1_voltage_mV,L1_velocity_rpm,L1_current_mA,R0_voltage_mV,R0_velocity_rpm,R0_current_mA,"
              "R1_voltage_mV,R1_velocity_rpm,R1_current_mA,interfered,after_stop\n");
  for (unsigned i=0;i<row_count;++i) {
    const auto& r=rows[i];
    std::printf("RR_CSV,%u,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u,%d,%d,%u,%u",
                r.ms,r.pose.x,r.pose.y,r.pose.theta,r.imu,TARGET_X-r.pose.x,TARGET_Y-r.pose.y,
                r.command.ms,r.command.updates,r.command.left,r.command.right,
                r.command.left_reversals,r.command.right_reversals);
    for (const auto& m:r.motors) std::printf(",%d,%.3f,%d",m.mv,m.rpm,m.ma);
    std::printf(",%d,%d\n",r.interfered,r.after_stop);
    if(i%4==3) {std::fflush(stdout);pros::delay(10);} // Post-stop pacing only.
  }
  std::fflush(stdout);
}

void reverse_right_diag_cancel_capture() {
  // Also called by the existing mode-transition cleanup if PROS kills the caller.
  ez::navigation_lock();
  if(capturing.exchange(false)) {
    ez::path_trace_hook_set(nullptr);
    ez::native_odom_exit_observer_set(nullptr);
  }
  ez::navigation_unlock();
}
