#include "boomerang_motion_trace.hpp"
#include "boomerang_trace_data.hpp"
#include "boomerang_report_writer.hpp"
#include "boomerang_trace_options.hpp"
#include "nav_diagnostics.hpp"
#include "EZ-Template/path_trace_hook.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cerrno>

namespace {
using namespace boomerang_trace;
struct Control {
  unsigned at=0;
  ez::pose pose{NAN,NAN,NAN};
  double xy_error=NAN,angular_error=NAN,xy_output=NAN,angular_output=NAN;
  ez::PID::Constants angular_constants{};
  ez::PathControlFrame frame{};
  int mode=-1;
  bool seen=false,interfered=false;
};
struct Motor { double rpm=NAN,mv=NAN,ma=NAN; };
struct Row {
  unsigned ms=0,dt=0,read_ms=0,valid_mask=0;
  int phase=0,imu_errno=0;
  Control control;
  double imu=NAN,rotation=NAN,battery=NAN;
  bool imu_installed=false,imu_calibrating=false,cancelled=false;
  int imu_status=-1;
  NavTrackerSample tracker[2]{};
  Motor motor[4];
};
struct Config {
  ez::PID::Constants pid[8]{};
  ez::slew::Constants slew[5]{};
  double lookahead=NAN,lead=NAN,boom_distance=NAN,bias=NAN,angle_tolerance=NAN;
  int turn_behavior=-1,odom_behavior=-1,bias_behavior=-1;
  bool forward_slew=false,reverse_slew=false,turn_slew=false;
  double tracker_diameter[2]{NAN,NAN},tracker_offset[2]{NAN,NAN};
  int ports[4]{},imu_port=0;
  bool reversed[4]{},active_seen=false;
  ez::PID::Constants active_xy{},active_angular{};
  ez::slew::Constants active_slew[2]{};
};
struct Stage { unsigned ms,id; int status; };
Row rows[capacity];
Window window;
Control latest;
Config config;
Stage stages[16];
unsigned stage_count=0,stage_dropped=0;
pros::Mutex& mutex(){static pros::Mutex m;return m;}
std::atomic<bool> requested{false},sampling{false},worker_running{false},session{false},ready{false},wrapper_active{false};
std::atomic<unsigned> hook_skips{0},sampler_skips{0},pending_end{0};
std::atomic<unsigned> dispatch_at{unknown},wait_at{unknown},stop_request_at{unknown},stop_at{unknown};
unsigned began=0,last_ms=0,max_gap=0,max_read=0,missed_slots=0;
std::atomic<bool> output_busy{false},output_abort{false};
bool synthetic=false;
bool truncated=false,cancelled=false,finished=false,post_interrupted=false;
ez::pose target{NAN,NAN,NAN};
int speed=0;
ez::NativeOdomWaitResult exits;
Rest rest;

void snapshot_config() { // Before dispatch; supported public readbacks, no setters.
  ez::PID* p[]={&chassis.xyPID,&chassis.odom_angularPID,&chassis.boomerangPID,
               &chassis.current_a_odomPID,&chassis.turnPID,&chassis.headingPID,&chassis.leftPID,&chassis.rightPID};
  for(unsigned i=0;i<8;++i)config.pid[i]=p[i]->constants_get();
  ez::slew* s[]={&chassis.slew_left,&chassis.slew_right,&chassis.slew_forward,
                &chassis.slew_backward,&chassis.slew_turn};
  for(unsigned i=0;i<5;++i)config.slew[i]=s[i]->constants_get();
  config.lookahead=chassis.odom_look_ahead_get();
  config.lead=chassis.odom_boomerang_dlead_get();
  config.boom_distance=chassis.odom_boomerang_distance_get();
  config.bias=chassis.odom_turn_bias_get();
  config.angle_tolerance=chassis.pid_angle_behavior_tolerance_get();
  config.turn_behavior=chassis.pid_turn_behavior_get();
  config.odom_behavior=chassis.pid_odom_behavior_get();
  config.bias_behavior=chassis.pid_angle_behavior_bias_get();
  config.forward_slew=chassis.slew_drive_forward_get();
  config.reverse_slew=chassis.slew_drive_backward_get();
  config.turn_slew=chassis.slew_turn_get();
  config.imu_port=chassis.imu.get_port();
  for(unsigned i=0;i<2;++i) {
    auto* t=nav_diagnostic_tracker(i);
    config.tracker_diameter[i]=t?t->wheel_diameter_get():NAN;
    config.tracker_offset[i]=t?t->distance_to_center_get():NAN;
  }
  for(unsigned side=0;side<2;++side) {
    const auto& motors=side?chassis.right_motors:chassis.left_motors;
    for(unsigned i=0;i<2;++i) {
      const unsigned n=side*2+i;
      config.ports[n]=i<motors.size()?motors[i].get_port():0;
      config.reversed[n]=i<motors.size()?motors[i].is_reversed():false;
    }
  }
}
void observe(ez::Drive* drive,const ez::PathControlFrame& frame) {
  if(drive!=&chassis || !sampling.load() || dispatch_at.load()==unknown || stop_at.load()!=unknown)return;
  if(!mutex().take(0)){++hook_skips;return;}
  if(sampling.load() && stop_at.load()==unknown) {
    if(!config.active_seen) {
      config.active_seen=true;
      config.active_xy=drive->xyPID.constants;
      config.active_angular=drive->current_a_odomPID.constants;
      config.active_slew[0]=drive->slew_left.constants;
      config.active_slew[1]=drive->slew_right.constants;
    }
    latest.at=pros::millis()-began;
    latest.pose=nav_internal_to_game_pose(drive->odom_pose_get());
    latest.xy_error=valid(drive->xyPID.error);
    latest.angular_error=valid(drive->current_a_odomPID.error);
    latest.xy_output=valid(drive->xyPID.output);
    latest.angular_output=valid(drive->current_a_odomPID.output);
    latest.angular_constants=drive->current_a_odomPID.constants;
    latest.frame=frame; latest.mode=drive->drive_mode_get();
    latest.interfered=drive->interfered;latest.seen=true;
  }
  mutex().give(); // No hardware reads, formatting, allocation, or waits.
}
void hardware(Row& r) {
  const unsigned started=pros::millis();
  r.imu_installed=chassis.imu.is_installed();
  errno=0;
  r.imu_status=static_cast<int>(chassis.imu.get_status());
  r.imu_calibrating=chassis.imu.is_calibrating();
  r.imu=valid(chassis.imu.get_heading());
  r.rotation=valid(chassis.imu.get_rotation());
  r.imu_errno=errno;
  r.battery=valid(pros::battery::get_voltage());
  r.cancelled=ez::autonomous_cancelled_snapshot();
  if(std::isfinite(valid(r.control.pose.x))&&std::isfinite(valid(r.control.pose.y))&&
      std::isfinite(valid(r.control.pose.theta)))r.valid_mask|=1u;
  if(r.imu_installed&&!r.imu_calibrating&&r.imu_errno==0&&
      r.imu_status!=static_cast<int>(pros::ImuStatus::error)&&
      std::isfinite(r.imu)&&std::isfinite(r.rotation))r.valid_mask|=2u;
  for(unsigned i=0;i<2;++i) {
    r.tracker[i]=nav_read_tracker(i);
    if(r.tracker[i].valid&&r.tracker[i].error==0)r.valid_mask|=4u<<i;
  }
  if(std::isfinite(r.battery))r.valid_mask|=16u;
  for(unsigned side=0;side<2;++side) {
    const auto& motors=side?chassis.right_motors:chassis.left_motors;
    for(unsigned i=0;i<2&&i<motors.size();++i) {
      const unsigned n=2*side+i;auto& m=r.motor[n];
      m.rpm=valid(motors[i].get_actual_velocity());
      m.mv=valid(motors[i].get_voltage());m.ma=valid(motors[i].get_current_draw());
      if(std::isfinite(m.rpm)&&std::isfinite(m.mv)&&std::isfinite(m.ma))r.valid_mask|=32u<<n;
    }
  }
  r.read_ms=pros::millis()-started;
}
void append(Row r) {
  if(window.count) {
    r.dt=r.ms-last_ms;max_gap=std::max(max_gap,r.dt);
    if(r.dt>=2*sample_ms)missed_slots+=r.dt/sample_ms-1;
  }
  last_ms=r.ms;max_read=std::max(max_read,r.read_ms);
  if(window.append())rows[window.count-1]=r;else truncated=true;
}
bool all_rest(const Row& r) {
  for(const auto& m:r.motor)if(!std::isfinite(m.rpm)||std::fabs(m.rpm)>=3)return false;
  return true;
}
void freeze() { // Caller owns mutex.
  sampling.store(false);window.stop();ready.store(window.ready);session.store(false);
}
void finalize_pending() { // Caller owns mutex; no sensor/output operations.
  const unsigned flags=pending_end.exchange(0);
  if(!flags)return;
  cancelled=cancelled || (flags&EndCancelled);
  truncated=truncated || (flags&(EndTimeout|EndContended));
  post_interrupted=post_interrupted || (flags&EndInterrupted);
  sampling.store(false);window.finalize(flags);
  ready.store(window.ready);session.store(false);
}
bool report_allowed() {
  if(output_abort.load())return false;
  return chassis.drive_mode_get()==ez::DISABLE &&
      (pros::competition::is_disabled() || pros::competition::is_autonomous());
}
void sampler() {
  while(sampling.load()) {
    const unsigned tick=pros::millis(),ms=tick-began,stopped=stop_at.load();
    if(ez::autonomous_cancelled_snapshot() || (stopped==unknown && ms>=capture_ms) ||
       (stopped!=unknown && (ms-stopped>=tail_ms || chassis.drive_mode_get()!=ez::DISABLE))) {
      unsigned flags=0;
      if(ez::autonomous_cancelled_snapshot())flags|=EndCancelled;
      if(stopped==unknown && ms>=capture_ms)flags|=EndTimeout;
      if(stopped!=unknown && chassis.drive_mode_get()!=ez::DISABLE)flags|=EndInterrupted;
      // Normal end of the finite post-stop window also finalizes the capture.
      sampling.store(false);
      if(flags)pending_end.fetch_or(flags);
      if(mutex().take(0)) {
        if(flags)finalize_pending();else freeze();
        mutex().give();
      } else {++sampler_skips;pending_end.fetch_or(EndContended);}
      break;
    }
    Row r;
    if(mutex().take(0)) {
      r.ms=pros::millis()-began;
      r.phase=stopped!=unknown?2:dispatch_at.load()==unknown?0:1;
      r.control=latest;
      if(r.phase!=1) {
        r.control={};r.control.pose=nav_pose_get();
        r.control.mode=chassis.drive_mode_get();r.control.interfered=chassis.interfered;
      }
      mutex().give();
      hardware(r);
      if(mutex().take(0)) {
        // Drop any read spanning cancellation, stop, or a subsequent movement.
        if(sampling.load() && !r.cancelled && !ez::autonomous_cancelled_snapshot() && stopped==stop_at.load() &&
           (r.phase!=2 || chassis.drive_mode_get()==ez::DISABLE)) {
          append(r);
          if(r.phase==2) {
            rest.sample(r.ms+r.read_ms,all_rest(r)); // Last device read has completed.
            if(rest.observed!=unknown)freeze();
          }
        } else ++sampler_skips;
        mutex().give();
      } else ++sampler_skips;
    } else ++sampler_skips;
    const unsigned cost=pros::millis()-tick;
    if(sampling.load())pros::delay(cost<sample_ms?sample_ms-cost:1);
  }
  ez::path_trace_hook_release(observe);
  worker_running.store(false);
}
const char* exit_name(int x) {
  switch(x) {
    case ez::SMALL_EXIT:return "SMALL";case ez::BIG_EXIT:return "BIG";
    case ez::VELOCITY_EXIT:return "VELOCITY";case ez::mA_EXIT:return "OVERCURRENT";
    case ez::RUNNING:return "RUNNING";case ez::ERROR_NO_CONSTANTS:return "NO_CONSTANTS";
    default:return "UNOBSERVED";
  }
}
double command(const Row& r,bool right) {
  const double v=right?r.control.frame.right:r.control.frame.left;
  return r.phase==1&&r.control.seen&&r.control.frame.output_enabled&&std::isfinite(v)&&
      std::fabs(v)<=2147483647.0?static_cast<int>(v):NAN;
}
void peak(double& p,double v) {if(std::isfinite(v))p=std::isfinite(p)?std::max(p,std::fabs(v)):std::fabs(v);}
int time_value(unsigned t){return t==unknown?-1:static_cast<int>(t);}
ReportWriter report;
bool report_sink(const char* line,unsigned size) {
  // No recorder/navigation lock held. Limit each serial burst and yield.
  // fwrite/fflush success is local acceptance, NOT host receipt.
  for(unsigned offset=0;offset<size;) {
    if(!report_allowed())return false;
    const unsigned n=std::min(64u,size-offset);
    if(std::fwrite(line+offset,1,n,stdout)!=n||std::fflush(stdout)!=0)return false;
    offset+=n;pros::delay(20);  }
  return true;
}
}
bool boomerang_motion_trace_arm(unsigned id) {
  if(output_busy.load())return false;
  if(!mutex().take(0))return false;
  if(!worker_running.load())finalize_pending();
  const bool ok=!worker_running.load()&&window.arm(id);
  if(ok)requested.store(true);
  mutex().give();return ok;
}
void boomerang_motion_trace_disarm(){boomerang_motion_trace_cancel();}
void boomerang_motion_trace_begin(ez::pose pose,int requested_speed) {
  if(output_busy.load()||!requested.exchange(false))return;
  if(!mutex().take(0))return;
  if(worker_running.load()||!window.armed||window.ready){mutex().give();return;}
  if(!ez::path_trace_hook_claim(observe)){window.armed=false;mutex().give();return;}
  window.begin();session.store(true);wrapper_active.store(true);target=pose;speed=requested_speed;
  synthetic=false;latest={};exits={};rest={};config={};
  last_ms=max_gap=max_read=missed_slots=stage_count=stage_dropped=0;
  hook_skips.store(0);sampler_skips.store(0);pending_end.store(0);
  truncated=cancelled=finished=post_interrupted=false;
  dispatch_at.store(unknown);wait_at.store(unknown);stop_request_at.store(unknown);stop_at.store(unknown);
  began=pros::millis();snapshot_config();
  Row first;first.ms=pros::millis()-began;first.control.pose=nav_pose_get();
  first.control.mode=chassis.drive_mode_get();first.control.interfered=chassis.interfered;
  hardware(first);append(first);
  sampling.store(true);worker_running.store(true);
  mutex().give();
  pros::Task task(sampler,TASK_PRIORITY_MIN,TASK_STACK_DEPTH_DEFAULT,"Boom recorder");
  if(task.get_state()==pros::E_TASK_STATE_INVALID) {
    sampling.store(false);worker_running.store(false);ez::path_trace_hook_release(observe);
    if(mutex().take(0)){truncated=true;freeze();mutex().give();}
  }
}
void boomerang_motion_trace_event(BoomerangTraceEvent event) {
  if(!wrapper_active.load())return;
  const unsigned t=pros::millis()-began;
  switch(event) {
    case BoomerangTraceEvent::Dispatch:dispatch_at.store(t);break;
    case BoomerangTraceEvent::WaitReturned:wait_at.store(t);break;
    case BoomerangTraceEvent::StopRequested:stop_request_at.store(t);break;
    case BoomerangTraceEvent::StopReturned:stop_at.store(t);break;
  }
}
void boomerang_motion_trace_finish() {
  if(!wrapper_active.exchange(false))return;
  ez::path_trace_hook_release(observe);
  exits=ez::native_odom_wait_result(&chassis); // Atomic native readback on the sole route caller.
  finished=true; // Same sampler observes post-stop rest; no wait or sensor IO here.
}
void boomerang_motion_trace_cancel() {
  if(output_busy.load()){output_abort.store(true);return;}
  if(!session.load()&&!requested.load()&&!wrapper_active.load())return;
  requested.store(false);sampling.store(false);wrapper_active.store(false);ez::path_trace_hook_release(observe);
  pending_end.fetch_or(EndCancelled);
  if(!mutex().take(0))return; // Deferred finalization; never block mode cleanup.
  finalize_pending();
  if(window.active){cancelled=true;freeze();}else {window.armed=false;if(window.ready)cancelled=true;}
  mutex().give();
}
bool boomerang_motion_trace_stage(unsigned id,int status) {
  if(!sampling.load()||!mutex().take(0))return false;
  const bool ok=sampling.load()&&stage_count<16;
  if(ok)stages[stage_count++]={pros::millis()-began,id,status};
  else ++stage_dropped;
  mutex().give();return ok;
}
bool boomerang_motion_trace_report() {
  if(!session.load()&&!ready.load())return false;
  const unsigned start=pros::millis();
  while(worker_running.load()&&pros::millis()-start<1500) {
    if(!report_allowed())return false;
    pros::delay(10); // Explicit report request AFTER motion, never in nav wrapper.
  }
  return boomerang_motion_trace_export(true,BOOM_TRACE_EXPORT_SD);
}
static bool render_report(ReportWriter::Sink sink) {
  const unsigned count=window.count;
  report.reset(sink);
  double rpm=NAN,current=NAN,heading=NAN,difference=NAN,step=NAN;
  Signal cmds[2],velocities[4],steering;
  unsigned saturation=0,phase_count[3]={}; TimestampOrder ordering;
  unsigned invalid_rows=0,stale_rows=0,dt_count=0,dt_total=0,dt_min=unknown,dt_max=0;
  bool interference=false;
  for(unsigned i=0;i<count;++i) {
    const auto& r=rows[i];interference=interference||r.control.interfered;
    invalid_rows+=r.valid_mask!=511u;
    if(r.phase>=0&&r.phase<3)++phase_count[r.phase];
    ordering.add(r.ms,i);
    peak(heading,heading_error(target.theta,r.control.pose.theta));
    if(r.phase==1) {
      for(unsigned side=0;side<2;++side)cmds[side].add(command(r,side),1);
      peak(difference,command(r,false)-command(r,true));
      steering.add(command(r,false)-command(r,true),1);
      if(std::isfinite(command(r,false))&&std::isfinite(command(r,true))&&
         std::max(std::fabs(command(r,false)),std::fabs(command(r,true)))>=speed)++saturation;
      if(!r.control.seen||r.ms-r.control.at>2*sample_ms)++stale_rows;
      if(i&&rows[i-1].phase==1) {
        ++dt_count;dt_total+=r.dt;dt_min=std::min(dt_min,r.dt);dt_max=std::max(dt_max,r.dt);
      }
    }
    for(unsigned m=0;m<4;++m) {
      peak(rpm,r.motor[m].rpm);peak(current,r.motor[m].ma);
      if(r.phase==1)velocities[m].add(r.motor[m].rpm,5);
    }
  }
  for(const auto& c:cmds)if(c.comparisons)peak(step,c.peak_step);
  const auto start=count?rows[0].control.pose:ez::pose{NAN,NAN,NAN};
  const auto final=count?rows[count-1].control.pose:ez::pose{NAN,NAN,NAN};
  report.print("\nBOOM_TRACE_BEGIN id=%u schema=4 build=%s_%s EZ_declared=3.2.2 PROS_declared=4.1.1\n"
    "target=(%.3f,%.3f,%.3f) speed=%d start=(%.3f,%.3f,%.3f) final_or_last=(%.3f,%.3f,%.3f)\n"
    "xy_error_in=%.3f final_heading_error_deg=%.3f observed_duration_ms=%u\n"
    "TIMES relative_to_capture_ms dispatch=%d wait_return=%d stop_request=%d stop_return=%d "
    "rest_window_start=%d rest_confirmed=%d native_completed=%d xy_exit=%s angular_exit=%s\n"
    "finished=%d cancelled=%d interfered=%d post_observation_interrupted=%d "
    "peak_abs_rpm=%.3f peak_current_mA=%.0f max_abs_heading_error_deg=%.3f "
    "max_abs_LminusR_127=%.0f max_command_step_127=%.0f command_reversals_L_R=%u,%u\n"
    "rows=%u capacity=%u dropped=%u truncated=%d invalid_sensor_rows=%u stale_control_rows=%u "
    "nominal_ms=%u effective_min_mean_max_ms=%d,%.2f,%u max_gap_ms=%u "
    "estimated_missed_slots=%u sampler_lock_skips=%u hook_lock_skips=%u max_device_read_ms=%u\n",
    window.id,__DATE__,__TIME__,target.x,target.y,target.theta,speed,start.x,start.y,start.theta,
    final.x,final.y,final.theta,distance(final.x,final.y,target.x,target.y),
    heading_error(target.theta,final.theta),count?rows[count-1].ms:0,
    time_value(dispatch_at.load()),time_value(wait_at.load()),time_value(stop_request_at.load()),
    time_value(stop_at.load()),time_value(rest.since),time_value(rest.observed),exits.completed,
    exit_name(exits.completed?exits.xy:-1),exit_name(exits.completed?exits.angular:-1),
    finished,cancelled,interference,post_interrupted,rpm,current,heading,difference,step,
    cmds[0].reversals,cmds[1].reversals,count,capacity,window.dropped,truncated,
    invalid_rows,stale_rows,sample_ms,time_value(dt_min),dt_count?double(dt_total)/dt_count:NAN,
    dt_max,max_gap,missed_slots,sampler_skips.load(),hook_skips.load(),max_read);
  report.print("BOOM_HEALTH capture_start_uptime_ms=%u trial_key=%u:%u mode=BOOMERANG direction=FWD worker_running=%d sampling=%d capture_deadline_ms=%u tail_limit_ms=%u\n",
    began,window.id,began,worker_running.load(),sampling.load(),capture_ms,tail_ms);
  report.print("BOOM_SOURCE synthetic=%d (synthetic configuration and values are fixtures, not live readbacks)\n",synthetic);
  report.print("BOOM_QUALITY sampled_at_requested_speed_rows=%u steering_delta_direction_changes=%u\n",
    saturation,steering.oscillations);
  report.print("BOOM_UNAVAILABLE chassis_acceleration wheel_slip actual_chassis_rest host_receipt_ack per_read_motor_errno native_exit_instant_ms\n");
  report.print("BOOM_CONFIG first_captured_control_tick_available=%d\n",config.active_seen);
  if(config.active_seen) {
    const ez::PID::Constants p[]={config.active_xy,config.active_angular};
    for(unsigned i=0;i<2;++i)
      report.print("BOOM_CONFIG active_%s kp=%.6f ki=%.6f kd=%.6f start_i=%.6f\n",
        i?"angular":"xy",p[i].kp,p[i].ki,p[i].kd,p[i].start_i);
    for(unsigned i=0;i<2;++i)
      report.print("BOOM_CONFIG active_slew_%s min_127=%.3f distance_native=%.6f\n",
        i?"R":"L",config.active_slew[i].min_speed,config.active_slew[i].distance_to_travel);
  }
  report.print("BOOM_TIMING wait_duration_ms=%d (wait return may be cancellation, not success)\n",
    wait_at.load()!=unknown&&dispatch_at.load()!=unknown?int(wait_at.load()-dispatch_at.load()):-1);
  const char* names[]={"xy","odom_angular","boomerang","active_angular_PRE_dispatch","turn","heading","drive_left","drive_right"};
  for(unsigned i=0;i<8;++i) {
    const auto& p=config.pid[i];
    report.print("BOOM_CONFIG PID %s live_pre_dispatch kp=%.6f ki=%.6f kd=%.6f start_i=%.6f\n",
                names[i],p.kp,p.ki,p.kd,p.start_i);
  }
  const char* sn[]={"left","right","forward","reverse","turn"};
  for(unsigned i=0;i<5;++i)
    report.print("BOOM_CONFIG SLEW %s live_pre_dispatch min_127=%.3f distance_native=%.6f\n",
                sn[i],config.slew[i].min_speed,config.slew[i].distance_to_travel);
  report.print("BOOM_CONFIG live_pre_dispatch lookahead_in=%.6f dlead=%.6f boom_distance_in=%.6f turn_bias=%.6f "
    "angle_tolerance_deg=%.6f turn_behavior_enum=%d odom_behavior_enum=%d bias_behavior_enum=%d "
    "global_slew_forward_reverse_turn=%d,%d,%d IMU_port=%d\n",
    config.lookahead,config.lead,config.boom_distance,config.bias,config.angle_tolerance,
    config.turn_behavior,config.odom_behavior,config.bias_behavior,
    config.forward_slew,config.reverse_slew,config.turn_slew,config.imu_port);
  for(unsigned i=0;i<2;++i)report.print("BOOM_CONFIG tracker=%u diameter_in=%.6f offset_in=%.6f\n",
    i,config.tracker_diameter[i],config.tracker_offset[i]);
  for(unsigned i=0;i<4;++i)
    report.print("BOOM_MOTOR index=%u side=%s port=%d reversed=%d rpm_direction_changes=%u "
      "rpm_sign_reversals=%u peak_rpm_step=%.3f\n",i,i<2?"L":"R",config.ports[i],config.reversed[i],
      velocities[i].oscillations,velocities[i].reversals,velocities[i].comparisons?velocities[i].peak_step:NAN);
  for(unsigned i=0;i<stage_count;++i)report.print("BOOM_STAGE ms=%u id=%u caller_status=%d\n",
      stages[i].ms,stages[i].id,stages[i].status);
  report.print("BOOM_NOTES stage_dropped=%u time=-1/unobserved nan=invalid; phase=0:pre,1:motion,2:post. "
    "cmd=native PTP requested arguments (-127..127), NOT measured voltage/RPM. "
    "Slew limit is not output. Motor RPM/voltage honor PROS reversal. "
    "IMU heading and odometry heading may use different zero references. "
    "valid_mask bits=pose,IMU,P5,P7,battery,M0,M1,M2,M3. "
    "Rest=all4_abs_rpm<3 for100ms with gaps<=90ms; not proof of chassis rest. "
    "RPM oscillations=count of delta-direction changes exceeding5rpm; command reversals ignore +/-1. "
    "All counts are sampled, not every controller tick; correlations do not prove causation. "
    "Physical measurement accuracy unverified.\n",stage_dropped);
  report.print("BOOM_CSV,row_index,ms,dt_ms,phase,read_ms,control_age_ms,valid_mask,x_in,y_in,h_deg,"
    "imu_heading_deg,imu_rotation_deg,imu_installed,imu_calibrating,imu_status,imu_errno,"
    "target_x_in,target_y_in,target_h_deg,remaining_in,heading_error_deg,L_cmd_127,R_cmd_127,"
    "xy_error,angular_error,xy_output,angular_output,slew_limit_127,L_slew,R_slew,"
    "active_a_kp,active_a_ki,active_a_kd,active_a_start_i,battery_mV,mode,cancelled,interfered");
  for(unsigned i=0;i<2;++i)report.print(",T%u_port,T%u_installed,T%u_config_ok,T%u_valid,T%u_raw_cd,T%u_in,T%u_errno",i,i,i,i,i,i,i);
  for(unsigned i=0;i<4;++i)report.print(",M%u_rpm,M%u_mV,M%u_mA",i,i,i);
  report.print(",signed_target_native,flipped,bias_enabled,bias_scale,xy_scaled,angular_scaled,angular_target_native_deg,active_target_x_native,active_target_y_native,active_target_h_native_deg,direction_sign,output_enabled\n");
  for(unsigned i=0;i<count;++i) {
    const auto& r=rows[i];const auto& c=r.control;const auto& p=c.angular_constants;
    report.print("BOOM_CSV,%u,%u,%u,%d,%u,%d,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,"
      "%.3f,%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,"
      "%.4f,%.4f,%.4f,%.4f,%.0f,%d,%d,%d",
      i,r.ms,r.dt,r.phase,r.read_ms,c.seen?int(r.ms-c.at):-1,r.valid_mask,
      valid(c.pose.x),valid(c.pose.y),valid(c.pose.theta),r.imu,r.rotation,
      r.imu_installed,r.imu_calibrating,r.imu_status,r.imu_errno,target.x,target.y,target.theta,
      distance(c.pose.x,c.pose.y,target.x,target.y),heading_error(target.theta,c.pose.theta),
      command(r,false),command(r,true),c.xy_error,c.angular_error,c.xy_output,c.angular_output,
      c.seen?valid(c.frame.slew_limit):NAN,c.seen?c.frame.slew_left_enabled:-1,c.seen?c.frame.slew_right_enabled:-1,
      c.seen?p.kp:NAN,c.seen?p.ki:NAN,c.seen?p.kd:NAN,c.seen?p.start_i:NAN,
      r.battery,c.mode,r.cancelled,c.interfered);
    for(const auto& t:r.tracker)report.print(",%d,%d,%d,%d,%.0f,%.4f,%d",t.actual_port,t.installed,
      t.configuration_ok,t.valid,t.raw_cd,t.distance_in,t.error);
    for(const auto& m:r.motor)report.print(",%.2f,%.0f,%.0f",m.rpm,m.mv,m.ma);
    report.print(",%.5f,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d\n",
      c.seen?valid(c.frame.signed_target):NAN,c.seen?c.frame.flipped:-1,
      c.seen?c.frame.bias_enabled:-1,c.seen?valid(c.frame.bias_scale):NAN,
      c.seen?valid(c.frame.xy_scaled):NAN,c.seen?valid(c.frame.angular_scaled):NAN,
      c.seen?valid(c.frame.angular_target):NAN,c.seen?valid(c.frame.target_x):NAN,
      c.seen?valid(c.frame.target_y):NAN,c.seen?valid(c.frame.target_heading):NAN,
      c.seen?c.frame.direction_sign:0,c.seen?c.frame.output_enabled:-1);
  }
  report.print("BOOM_ORDER nondecreasing=%d equal_timestamps=%u backward_timestamps=%u first_backward_row=%d first_equal_row=%d backward_pair_ms=%u,%u\n",
    ordering.ordered(),ordering.equal,ordering.backwards,time_value(ordering.first_bad),time_value(ordering.first_equal),ordering.bad_previous,ordering.bad_current);
  report.print("BOOM_VALIDATION id=%u recorded=%u emitted=%u ordered=%d pre_motion_post=%u,%u,%u csv_fnv1a=%08lx\n",
    window.id,count,report.emitted,ordering.ordered(),phase_count[0],phase_count[1],phase_count[2],
    static_cast<unsigned long>(report.csv_hash));
  report.print("BOOM_TRACE_END id=%u recorded=%u emitted=%u local_write_ok=%d host_delivery=UNVERIFIED\n",
    window.id,count,report.emitted,report.ok);
  return report.ok&&!report.used&&report.emitted==count;
}
namespace {
BoomerangReportStatus output_status;
bool output_terminal=false,output_sd=false;
FILE* sd_file=nullptr;
int sd_error=0;
int io_error(){return errno?errno:EIO;}
bool sd_sink(const char* data,unsigned size) {
  if(!report_allowed()){sd_error=ECANCELED;return false;}
  errno=0;
  if(std::fwrite(data,1,size,sd_file)!=size){sd_error=io_error();return false;}
  pros::delay(1); // Low-priority worker, no recorder/navigation lock.
  return true;
}
void output_worker() {
  // A separate low-priority owner survives competition callback termination.
  // Never remove this task: allow it to close its file on mode-change abort.
  if(output_sd) {
    auto& status=output_status.sd;status.attempted=true;
    errno=0;
    if(!report_allowed())status.error=ECANCELED;
    else if(pros::usd::is_installed()!=1)status.error=ENODEV;
    else {
      std::snprintf(output_status.sd_path,sizeof(output_status.sd_path),
        "/usd/boom-%u-%u.txt",window.id,began);
      // Append preserves any existing file, including across Brain reboots.
      // Do not use fopen("rb") failure as an existence test: PROS maps it to ENFILE.
      errno=0;sd_file=std::fopen(output_status.sd_path,"ab");
      if(!sd_file)status.error=io_error();
      else {
        sd_error=0;
        const bool rendered=render_report(sd_sink);
        status.rows=report.emitted;
        int failure=sd_error?sd_error:rendered?0:EIO;
        errno=0;const int flushed=std::fflush(sd_file);
        if(flushed!=0&&!failure)failure=io_error();
        errno=0;const int closed=std::fclose(sd_file);sd_file=nullptr;
        if(closed!=0&&!failure)failure=io_error();
        status.error=failure;status.success=rendered&&flushed==0&&closed==0;
      }
    }
  }
  // SD failure does not skip terminal reporting and vice versa.
  if(output_terminal) {
    auto& status=output_status.terminal;status.attempted=true;
    errno=0;
    const bool allowed=report_allowed();
    status.success=allowed&&render_report(report_sink);
    status.rows=allowed?report.emitted:0;
    status.error=status.success?0:report_allowed()?io_error():ECANCELED;
  }
  if(output_terminal&&report_allowed())
    std::printf("BOOM_OUTPUT terminal_attempted=%d terminal_ok=%d terminal_errno=%d terminal_rows=%u sd_attempted=%d sd_ok=%d sd_errno=%d sd_rows=%u sd_path=%s retained=1\n",
      output_status.terminal.attempted,output_status.terminal.success,output_status.terminal.error,output_status.terminal.rows,
      output_status.sd.attempted,output_status.sd.success,output_status.sd.error,output_status.sd.rows,
      output_status.sd_path[0]?output_status.sd_path:"NONE");
  // Keep the complete frozen capture for independent retry/discard, even on success.
  output_busy.store(false);
}
}
BoomerangReportStatus boomerang_motion_trace_output_status() {
  if(output_busy.load()) {
    BoomerangReportStatus busy;busy.terminal.error=busy.sd.error=EBUSY;return busy;
  }
  return output_status; // Single diagnostic caller; published after worker completion.
}
bool boomerang_motion_trace_export(bool terminal,bool sd) {
  if(!terminal&&!sd)return false;
  bool expected=false;
  if(!output_busy.compare_exchange_strong(expected,true))return false;
  if(worker_running.load()||wrapper_active.load()||!mutex().take(0)){output_busy.store(false);return false;}
  finalize_pending();
  output_abort.store(false);
  if(!window.ready||sampling.load()||!report_allowed()){mutex().give();output_busy.store(false);return false;}
  output_status={};output_terminal=terminal;output_sd=sd;
  mutex().give();
  pros::Task worker(output_worker,TASK_PRIORITY_MIN,TASK_STACK_DEPTH_DEFAULT,"Boom report IO");
  if(worker.get_state()==pros::E_TASK_STATE_INVALID){output_busy.store(false);return false;}
  // Explicit post-motion caller only. Mode cleanup never joins this worker.
  const unsigned started=pros::millis();
  while(output_busy.load()&&pros::millis()-started<30000) {
    if(!report_allowed()){output_abort.store(true);return false;}
    pros::delay(10);
  }
  if(output_busy.load()){output_abort.store(true);return false;}
  return (!terminal||output_status.terminal.success)&&(!sd||output_status.sd.success);
}
bool boomerang_motion_trace_dump(){return boomerang_motion_trace_export(true,false);}
bool boomerang_motion_trace_discard() {
  if(output_busy.load()||worker_running.load()||sampling.load()||wrapper_active.load()||!mutex().take(0))return false;
  window.ready=false;window.armed=false;ready.store(false);pending_end.store(0);
  output_abort.store(false);mutex().give();return true;
}
void boomerang_motion_trace_synthetic_start() {
  if (!(pros::competition::is_disabled() ||
      pros::competition::is_autonomous()) ||
    chassis.drive_mode_get() != ez::DISABLE ||
    output_busy.load() ||
    worker_running.load() ||
    !mutex().take(0)) {
    std::printf("SYNTHETIC TEST: ENTRY REJECTED\n");
    std::fflush(stdout);
    return;
}
  if(window.active||window.ready){mutex().give();return;}
  window.arm(71001);window.begin();synthetic=true;
  latest={};config={};rest={};exits={};target={19.94,6.93,56.83};speed=100;
  began=pros::millis();last_ms=max_gap=max_read=missed_slots=stage_count=stage_dropped=0;
  hook_skips.store(0);sampler_skips.store(0);pending_end.store(0);
  truncated=cancelled=post_interrupted=false;finished=true;
  dispatch_at.store(0);wait_at.store(1950);stop_request_at.store(1950);stop_at.store(1950);
  config.active_seen=true;
  for(auto& p:config.pid)p={1.234567,0.001234,12.345678,5.0};
  config.lookahead=7;config.lead=0.625;config.boom_distance=16;config.bias=2;
  config.ports[0]=6;config.ports[1]=4;config.ports[2]=3;config.ports[3]=2;
  for(unsigned i=0;i<71;++i) {
    Row r;r.ms=i?30*(i-1):0;r.phase=i==0?0:i>=66?2:1;r.read_ms=1;
    r.control.pose={19.94*i/70.0,6.93*i/70.0,56.83*i/70.0};
    r.control.seen=r.phase==1;r.control.at=r.ms;r.control.mode=r.phase==1?ez::POINT_TO_POINT:ez::DISABLE;
    r.control.xy_error=20-i*0.28;r.control.angular_error=56.83-i*0.8;
    r.control.xy_output=123.456;r.control.angular_output=-12.345;
    r.control.frame.left=80.123;r.control.frame.right=99.876;r.control.frame.output_enabled=true;
    r.control.frame.slew_limit=100;r.control.frame.bias_scale=0.912345;
    r.control.frame.xy_scaled=88.12345;r.control.frame.angular_scaled=-9.12345;
    r.control.frame.target_x=19.94;r.control.frame.target_y=6.93;r.control.frame.target_heading=56.83;
    r.imu=r.control.pose.theta;r.rotation=r.imu;r.imu_installed=true;r.imu_status=0;r.battery=12345;
    for(auto& m:r.motor){m.rpm=r.phase==2?0:345.678;m.mv=9876;m.ma=1234;}
    // Deliberate NaN and invalid tracker fields exercise representative formatting.
    if(i%7==0)r.motor[2].rpm=NAN;
    append(r);
  }
  freeze();mutex().give();
  // Only formatting/IO; never calls snapshot_config, hardware, navigation or mechanism APIs.
  boomerang_motion_trace_export(true,BOOM_TRACE_EXPORT_SD);
}
