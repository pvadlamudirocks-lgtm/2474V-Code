#include "full_system_diag.hpp"
#include "full_system_diag_limits.hpp"
#include "main.h"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "autons.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <atomic>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
using namespace full_diag;
enum Status { PASS, FAIL, INCONCLUSIVE, SKIPPED };
const char* names[]={"PASS","FAIL","INCONCLUSIVE","SKIPPED"};
enum Kind { PULSE, TURN, DRIVE, POINT };
struct Sample {
  unsigned ms=0,read_end_ms=0;
  bool valid=false,imu_valid=false,tracker_valid[2]={},motor_valid[4]={};
  double heading=NAN,rotation=NAN,x=NAN,y=NAN,h=NAN,left=NAN,right=NAN;
  double raw[2]={NAN,NAN},distance[2]={NAN,NAN};
  int error[3]={};
  double pos[4]={},rpm[4]={},temperature[4]={};
  int voltage[4]={},current[4]={};
  int mode=0,lcmd=0,rcmd=0;
  double pid_error[4]={},pid_output[4]={};
};
constexpr unsigned capacity=200;
Sample rows[capacity], baseline, last;
std::atomic<unsigned> row_count{0};
std::atomic<bool> armed{false},done{false};
std::atomic<const char*> failure{nullptr};
Kind kind=DRIVE;
unsigned began=0,mask=0;
double requested=0;
int lcmd=0,rcmd=0;
bool recording_exits=false;
bool hooks_owned=false;
bool stage_seen[5]={};
int exits[5]={}; // left,right,turn,XY,angular; caller task only
unsigned exit_ms[5]={};
double exit_error[5]={};
Status imu_status=SKIPPED,motor_status[4]={SKIPPED,SKIPPED,SKIPPED,SKIPPED};
Status tracker_status[2]={SKIPPED,SKIPPED},turn_status=SKIPPED,drive_status=SKIPPED,point_status=SKIPPED;
const char* first_failure=nullptr;

pros::Motor& motor(unsigned i){return i<2 ? chassis.left_motors[i] : chassis.right_motors[i-2];}
ez::tracking_wheel* tracker(unsigned i){return i ? chassis.odom_tracker_right : chassis.odom_tracker_back;}
bool enabled(){return !pros::competition::is_disabled() && pros::competition::is_autonomous();}
bool button(pros::controller_digital_e_t key){return master.is_connected()==1 && master.get_digital(key)==1;}
bool cancelled(){return !enabled() || button(pros::E_CONTROLLER_DIGITAL_B);}
void note(const char* text){if(!first_failure)first_failure=text;}
bool finite(double v){return std::isfinite(v) && v!=PROS_ERR && v!=PROS_ERR_F;}
Sample read_sample(){
  Sample s; s.ms=pros::millis(); s.valid=chassis.left_motors.size()==2 && chassis.right_motors.size()==2;
  errno=0; s.heading=chassis.imu.get_heading(); s.error[2]=errno;
  s.rotation=chassis.imu.get_rotation();
  s.imu_valid = chassis.imu.is_installed() && chassis.imu.get_status()!=pros::ImuStatus::error &&
    !chassis.imu.is_calibrating() && finite(s.heading) && finite(s.rotation);
  s.valid &= s.imu_valid;
  const auto p=nav_pose_get(); s.x=p.x;s.y=p.y;s.h=p.theta;
  s.valid &= finite(s.x)&&finite(s.y)&&finite(s.h);
  for(unsigned i=0;i<2;++i){
    auto* t=tracker(i); errno=0;
    const auto raw=t ? t->smart_encoder.get_position() : PROS_ERR;
    s.error[i]=t ? errno : ENODEV; s.raw[i]=raw;
    const double tpi=t ? t->ticks_per_inch() : NAN;
    const bool ok=t && t->smart_encoder.is_installed() && raw!=PROS_ERR && finite(tpi) && tpi>0;
    s.tracker_valid[i]=ok;s.valid &= ok; if(ok)s.distance[i]=static_cast<double>(raw)/tpi;
  }
  if(chassis.left_motors.size()==2 && chassis.right_motors.size()==2){
    for(unsigned i=0;i<4;++i){
      auto& m=motor(i);s.pos[i]=m.get_position();s.rpm[i]=m.get_actual_velocity();
      s.voltage[i]=m.get_voltage();s.current[i]=m.get_current_draw();s.temperature[i]=m.get_temperature();
      s.motor_valid[i] = m.is_installed() && finite(s.pos[i]) && finite(s.rpm[i]) &&
        finite(s.temperature[i]) && s.voltage[i]!=PROS_ERR && s.current[i]!=PROS_ERR;
      s.valid &= s.motor_valid[i];
    }
    s.left=chassis.drive_sensor_left();s.right=chassis.drive_sensor_right();
    s.valid &= finite(s.left)&&finite(s.right);
  }
  s.mode=chassis.drive_mode_get();s.lcmd=lcmd;s.rcmd=rcmd;
  ez::PID* pids[]={&chassis.leftPID,&chassis.rightPID,&chassis.turnPID,&chassis.headingPID};
  for(unsigned i=0;i<4;++i){s.pid_error[i]=pids[i]->error;s.pid_output[i]=pids[i]->output;}
  if(kind==POINT){s.pid_error[2]=chassis.xyPID.error;s.pid_output[2]=chassis.xyPID.output;
    s.pid_error[3]=chassis.current_a_odomPID.error;s.pid_output[3]=chassis.current_a_odomPID.output;}
  s.read_end_ms=pros::millis();
  return s;
}
void print_sample(const char* label,const Sample& s){
  std::printf("%s ms=%u valid=%d IMU_heading=%.4f rotation=%.4f errno=%d pose=(%.4f,%.4f,%.4f) encoder_in=(%.4f,%.4f)\n",
    label,s.ms,s.valid,s.heading,s.rotation,s.error[2],s.x,s.y,s.h,s.left,s.right);
  for(unsigned i=0;i<2;++i)std::printf("TRACKER P%d valid=%d raw_cd=%.0f distance_in=%.5f errno=%d\n",
    tracker(i)?tracker(i)->smart_encoder.get_port():0,s.tracker_valid[i],s.raw[i],s.distance[i],s.error[i]);
  if(chassis.left_motors.size()==2 && chassis.right_motors.size()==2)
    for(unsigned i=0;i<4;++i)std::printf("MOTOR P%d valid=%d reversed=%d brake_enum=%d position_native=%.4f rpm=%.3f voltage_readback_mV=%d current_mA=%d temp_C=%.2f\n",
      motor(i).get_port(),s.motor_valid[i],motor(i).is_reversed(),static_cast<int>(motor(i).get_brake_mode()),
      s.pos[i],s.rpm[i],s.voltage[i],s.current[i],s.temperature[i]);
}
void observe_commands(ez::Drive* drive,int left,int right){if(drive==&chassis){lcmd=left;rcmd=right;}}
void stop_in_owner(const char* reason){
  if(reason){const char* empty=nullptr;failure.compare_exchange_strong(empty,reason);ez::movement_cancel(reason);}
  chassis.drive_set(0,0); // Disable native PID before zeroing individual pulse outputs.
  bool accepted=true;
  for(unsigned i=0;i<4;++i)if(motor(i).move_voltage(0)==PROS_ERR)accepted=false;
  if(!accepted){const char* empty=nullptr;failure.compare_exchange_strong(empty,"motor stop write rejected");
    ez::movement_cancel("motor stop write rejected");}
  armed.store(false);done.store(true);
}
bool service(ez::Drive* drive,ez::DiagnosticEvent event){
  if constexpr(!power_allowed(profile,motor_bench_physically_verified,controlled_motion_enabled))return true;
  if(drive!=&chassis || !armed.load(std::memory_order_acquire))return true;
  if(event==ez::DiagnosticEvent::STOP){
    const char* reason=!enabled() ? "disabled/mode changed" :
      std::strcmp(ez::movement_fault(),"none") ? ez::movement_fault() : nullptr;
    stop_in_owner(reason);return false;
  }
  if(cancelled() || !ez::movement_ready()) {stop_in_owner(cancelled()?"operator cancel/disabled":ez::movement_fault());return false;}
  const unsigned now=pros::millis();
  if(expired(now,began,kind==PULSE?pulse_ms:motion_ms)){
    stop_in_owner(kind==PULSE?nullptr:"diagnostic deadline");return false;
  }
  const unsigned count=row_count.load(std::memory_order_relaxed);
  // Sample before output, so a detected bad reading cannot authorize a pulse.
  // Command/PID columns describe the preceding controller iteration.
  if(event==ez::DiagnosticEvent::BEFORE_OUTPUT && (!count || now-last.ms>=20)){
    const auto s=read_sample();last=s;
    if(count<capacity){rows[count]=s;row_count.store(count+1,std::memory_order_release);}
    const double angle=s.rotation-baseline.rotation;
    const double travel=std::max({std::fabs(s.left-baseline.left),std::fabs(s.right-baseline.right),std::hypot(s.x-baseline.x,s.y-baseline.y)});
    if(!s.valid || !nav_sensors_ready()){stop_in_owner("invalid sensor feedback");return false;}
    if(excessive(angle,travel) || ((kind==PULSE || kind==DRIVE || kind==POINT)&&std::fabs(angle)>10)){
      stop_in_owner("excessive rotation/travel");return false;
    }
    if(kind==TURN && wrong_turn(angle,requested)){stop_in_owner("opposite measured turn");return false;}
    if((kind==DRIVE || kind==POINT) && (s.left-baseline.left < -0.25 || s.right-baseline.right < -0.25)){
      stop_in_owner("opposite measured drive");return false;
    }
    if(kind==PULSE)for(unsigned i=0;i<4;++i)if((mask&(1u<<i)) && s.rpm[i]<-3 && s.pos[i]<baseline.pos[i]){
      stop_in_owner("opposite motor encoder response");return false;
    }
  }
  // Reads may span a deadline or competition transition.
  if(cancelled() || expired(pros::millis(),began,kind==PULSE?pulse_ms:motion_ms)){
    stop_in_owner(cancelled()?"operator cancel/disabled":kind==PULSE?nullptr:"diagnostic deadline");return false;
  }
  if(kind==PULSE && event==ez::DiagnosticEvent::BEFORE_OUTPUT){
    // This callback is executing IN the existing drive owner, not another task.
    for(unsigned i=0;i<4;++i)if(motor(i).move_voltage(mask&(1u<<i)?pulse_mv:0)==PROS_ERR){
      stop_in_owner("motor voltage command rejected");return false;
    }
    return false; // Never run PID output during an individual/group pulse.
  }
  return kind!=PULSE;
}
bool countdown(const char* instruction){
  std::printf("OPERATOR: %s; starts in 5 seconds. B/disable cancels.\n",instruction);
  pros::lcd::set_text(0,"FULL SYSTEM DIAG");pros::lcd::set_text(1,instruction);
  const auto start=pros::millis();unsigned shown=0;
  while(!expired(pros::millis(),start,countdown_ms)){
    if(cancelled()){note("cancelled during countdown");return false;}
    const unsigned elapsed=pros::millis()-start;
    if(elapsed>=countdown_ms)break;
    const unsigned remaining=(countdown_ms-elapsed+999)/1000;
    if(remaining!=shown){shown=remaining;pros::lcd::print(2,"START IN %u | B CANCEL",remaining);std::printf("COUNTDOWN %u\n",remaining);}
    pros::delay(10);
  }
  pros::lcd::set_text(2,"GO | B/disable cancels");
  return !cancelled();
}
bool rest(){
  const auto start=pros::millis();unsigned stable=0;
  auto previous=read_sample();
  while(!expired(pros::millis(),start,settle_ms)){
    if(cancelled())return false;
    const auto s=read_sample();bool quiet=s.valid;
    for(double rpm:s.rpm)quiet &= std::fabs(rpm)<3;
    quiet &= previous.valid && std::fabs(s.rotation-previous.rotation)<0.2 &&
      std::hypot(s.x-previous.x,s.y-previous.y)<0.05;
    previous=s;
    if(quiet){if(!stable)stable=pros::millis();if(pros::millis()-stable>=rest_ms)return true;}else stable=0;
    pros::delay(10);
  }
  return false;
}
void dump(){
  std::printf("TRACE mode=%d request=%.3f mask=%u pulse_mV=%d limit_ms=%u | PID columns L,R,TURN(or XY),HEADING(or angular)\n",
    static_cast<int>(kind),requested,mask,pulse_mv,kind==PULSE?pulse_ms:motion_ms);
  std::printf("TRACE columns: ms,read_span_ms,valid,mode,x,y,h,imu_rotation,rawH,rawV,L_in,R_in,Lcmd,Rcmd,PID_errors[4],PID_outputs[4], motors[4] each position,rpm,mV,mA\n");
  const unsigned count=row_count.load(std::memory_order_acquire);
  for(unsigned i=0;i<count;++i){const auto& s=rows[i];
    std::printf("TRACE %u,%u,%d,%d,%.4f,%.4f,%.3f,%.3f,%.0f,%.0f,%.4f,%.4f,%d,%d",s.ms-began,s.read_end_ms-s.ms,s.valid,s.mode,s.x,s.y,s.h,s.rotation,s.raw[0],s.raw[1],s.left,s.right,s.lcmd,s.rcmd);
    for(double v:s.pid_error)std::printf(",%.3f",v);for(double v:s.pid_output)std::printf(",%.3f",v);
    for(unsigned j=0;j<4;++j)std::printf(",%.3f,%.3f,%d,%d",s.pos[j],s.rpm[j],s.voltage[j],s.current[j]);std::printf("\n");
  }
  std::printf("EXITS L=%s R=%s TURN=%s XY=%s ANG=%s (RUNNING=no terminal capture)\n",
    ez::exit_to_string(static_cast<ez::exit_output>(exits[0])).c_str(),ez::exit_to_string(static_cast<ez::exit_output>(exits[1])).c_str(),
    ez::exit_to_string(static_cast<ez::exit_output>(exits[2])).c_str(),ez::exit_to_string(static_cast<ez::exit_output>(exits[3])).c_str(),ez::exit_to_string(static_cast<ez::exit_output>(exits[4])).c_str());
  for(unsigned i=0;i<5;++i)if(exits[i]!=ez::RUNNING)
    std::printf("EXIT index=%u at_ms=%u error=%.5f\n",i,exit_ms[i],exit_error[i]);
}
Status execute(Kind next,unsigned motor_mask,double request){
  if constexpr(!power_allowed(profile,motor_bench_physically_verified,controlled_motion_enabled))return SKIPPED;
  if((profile==Profile::MOTOR_BENCH && next!=PULSE) || (profile==Profile::CONTROLLED_MOTION && next==PULSE))return SKIPPED;
  if(!enabled() || !ez::movement_ready() || !nav_sensors_ready()){note("movement readiness rejected");return FAIL;}
  if(!ez::movement_stop() || !rest()){note("pre-move stop/rest not verified");return FAIL;}
  kind=next;mask=motor_mask;requested=request;baseline=read_sample();last=baseline;
  if(!baseline.valid){note("invalid pre-move sample");return FAIL;}
  const double radians=baseline.h*3.141592653589793/180;
  const double target_x=baseline.x+(next==DRIVE?request*std::sin(radians):0);
  const double target_y=baseline.y+(next==DRIVE?request*std::cos(radians):next==POINT?request:0);
  print_sample("BEFORE MOVEMENT",baseline);
  std::printf("TARGET kind=%d x=%.4f y=%.4f heading=%.4f request=%.3f; limits are diagnostic screening, not competition qualification.\n",
    static_cast<int>(next),target_x,target_y,baseline.h+(next==TURN?request:0),request);
  row_count.store(0);failure.store(nullptr);done.store(false);lcmd=rcmd=0;
  for(unsigned i=0;i<5;++i){exits[i]=ez::RUNNING;exit_ms[i]=0;exit_error[i]=NAN;}
  began=pros::millis();recording_exits=true;armed.store(true,std::memory_order_release);
  if(next==PULSE){
    while(!done.load() && !expired(pros::millis(),began,pulse_ms+100))pros::delay(5);
    if(!done.load()){failure.store("pulse owner did not finish");ez::movement_cancel("pulse owner did not finish");}
  }else if(next==TURN)nav_turn_to(nav_theta_get()+request,speed);
  else if(next==DRIVE)nav_drive_distance(request,speed);
  else nav_go_to(baseline.x,baseline.y+request,speed);
  armed.store(false);recording_exits=false;
  const bool ack=ez::movement_stop();const bool stopped=ack && rest();
  if(!stopped){
    note(failure.load()?failure.load():ack?"post-move rest not verified":"post-move stop unacknowledged");
    std::printf("RESULT FAIL elapsed_ms=%u stop_ack=%d rest_verified=0 fault=%s diagnostic_fault=%s; long output withheld.\n",
      pros::millis()-began,ack,ez::movement_fault(),failure.load()?failure.load():"none");
    return FAIL;
  }
  const auto final=read_sample();
  dump();print_sample("AFTER STOP (physical travel unverified)",final);
  std::printf("RESULT through_stop_ms=%u stop_ack=%d rest_verified=%d interference=%d fault=%s diagnostic_fault=%s\n",
    final.ms-began,ack,stopped,chassis.interfered,ez::movement_fault(),failure.load()?failure.load():"none");
  bool ok=ack && stopped && !failure.load() && ez::movement_ready() && !chassis.interfered && final.valid;
  if(next==PULSE){
    for(unsigned i=0;i<4;++i)if(mask&(1u<<i)){
      bool response=false;for(unsigned j=0;j<row_count.load();++j)response |= rows[j].rpm[i]>3;
      ok &= response && final.pos[i]>baseline.pos[i];
    }
  }else {
    const auto arrived=[](int exit){return exit==ez::SMALL_EXIT || exit==ez::BIG_EXIT;};
    if(next==TURN)ok &= arrived(exits[2]) && std::fabs((final.rotation-baseline.rotation)-request)<=3;
    else {
      ok &= next==DRIVE ? arrived(exits[0])&&arrived(exits[1]) : arrived(exits[3])&&arrived(exits[4]);
      ok &= std::hypot(final.x-target_x,final.y-target_y)<=1 && std::fabs(final.rotation-baseline.rotation)<=3;
      std::printf("ARRIVAL position_error_in=%.4f heading_change_deg=%.4f (limits 1in/3deg)\n",
        std::hypot(final.x-target_x,final.y-target_y),final.rotation-baseline.rotation);
    }
  }
  if(!ok){note(failure.load()?failure.load():"response/arrival/stop acceptance not met");return FAIL;}
  std::printf("PASS: sensor-response/arrival/stop criteria only. Physical direction/travel: INCONCLUSIVE until independently measured.\n");
  return PASS;
}
void summary(){
  const char* stages[]={"SENSOR HEALTH","MANUAL IMU / TRACKING WHEELS","MOTOR RESPONSE","AUTONOMOUS PID","ODOMETRY NAVIGATION"};
  for(unsigned i=0;i<5;++i)if(!stage_seen[i])std::printf("[STAGE %c] %s: SKIPPED\n",'A'+i,stages[i]);
  std::printf("\n========== DIAGNOSTIC SUMMARY ==========\nIMU: %s\n",names[imu_status]);
  if(chassis.left_motors.size()==2 && chassis.right_motors.size()==2)
    for(unsigned i=0;i<4;++i)std::printf("MOTOR P%d: %s\n",std::abs(motor(i).get_port()),names[motor_status[i]]);
  for(unsigned i=0;i<2;++i)std::printf("TRACKER P%d: %s\n",tracker(i)?std::abs(tracker(i)->smart_encoder.get_port()):0,names[tracker_status[i]]);
  std::printf("TURN PID: %s\nDRIVE PID: %s\nODOM NAVIGATION: %s\nFIRST IDENTIFIED FAILURE: %s\n"
    "EVIDENCE: see timestamped snapshots, TRACE and exits above; physical observations are not inferred.\n"
    "POSSIBLE CAUSES: high current/no RPM suggests load or opposed motors; low commands suggest feedback cancellation; wrong signs require physical inspection; early exits are not arrival.\n"
    "RECOMMENDED NEXT INVESTIGATION: inspect the FIRST failed/inconclusive prerequisite; do not tune from final pose alone.\n"
    "PASS applies only to stated checks in this session, not full competition qualification. Geometry remains physically unverified unless independently measured.\n"
    "========== 2474V FULL SYSTEM DIAG END ==========\n",names[turn_status],names[drive_status],names[point_status],first_failure?first_failure:"none established; skipped/inconclusive is not PASS");
}
}

bool full_system_diag_recording(){return recording_exits && armed.load(std::memory_order_acquire);}
void full_system_diag_exit(ez::PID* pid,int reason){
  if(!full_system_diag_recording() || reason==ez::RUNNING)return;
  ez::PID* pids[]={&chassis.leftPID,&chassis.rightPID,&chassis.turnPID,&chassis.xyPID,&chassis.current_a_odomPID};
  for(unsigned i=0;i<5;++i)if(pid==pids[i] && exits[i]==ez::RUNNING){
    exits[i]=reason;exit_ms[i]=pros::millis()-began;exit_error[i]=pid->error;
  }
}
// Passive windows use the same timestamped row format as powered captures.
// Reads within a row are sequential; read_span_ms exposes acquisition skew.
bool capture_window(unsigned duration){
  row_count.store(0);began=pros::millis();
  for(unsigned i=0;i<5;++i){exits[i]=ez::RUNNING;exit_ms[i]=0;exit_error[i]=NAN;}
  bool valid=true;
  while(!expired(pros::millis(),began,duration)){
    if(cancelled()){note("cancelled during passive window");return false;}
    const auto sample=read_sample();const auto count=row_count.load();
    if(count>=capacity){note("passive capture capacity exceeded");return false;}
    rows[count]=sample;row_count.store(count+1);valid &= sample.valid;
    pros::delay(passive_sample_ms);
  }
  return valid;
}
void run_full_system_diag(){
  std::printf("\n========== 2474V FULL SYSTEM DIAG START ==========\n");
  const char* profile_name=profile==Profile::PASSIVE?"PASSIVE":profile==Profile::MOTOR_BENCH?"MOTOR_BENCH":"CONTROLLED_MOTION";
  std::printf("PROFILE=%s bench_verified=%d controlled_enabled=%d\n",profile_name,motor_bench_physically_verified,controlled_motion_enabled);
  first_failure=nullptr;imu_status=turn_status=drive_status=point_status=SKIPPED;
  for(auto& s:motor_status)s=SKIPPED;for(auto& s:tracker_status)s=SKIPPED;
  armed.store(false);recording_exits=false;hooks_owned=false;
  for(auto& seen:stage_seen)seen=false;
  if(nav_sequence_active()){note("active navigation sequence: diagnostic refused");summary();return;}
  struct Cleanup {~Cleanup(){armed.store(false);recording_exits=false;
    if(!ez::movement_stop())note("cleanup stop unacknowledged");
    if constexpr(power_allowed(profile,motor_bench_physically_verified,controlled_motion_enabled))
      if(hooks_owned){ez::diagnostic_service_set(nullptr);ez::command_observer_set(nullptr);}
    summary();}} cleanup;
  if(!ez::movement_stop()){note("stop unacknowledged");return;}
  odom_display_freeze();
  stage_seen[0]=true;
  pros::lcd::set_text(0,profile_name);pros::lcd::set_text(1,"HEALTH: STILL FOR 2s");
  std::printf("[STAGE A] SENSOR HEALTH\nKeep stationary for 2 seconds. IMU P%d installed=%d calibrating=%d status=%d\n",
    chassis.imu.get_port(),chassis.imu.is_installed(),chassis.imu.is_calibrating(),static_cast<int>(chassis.imu.get_status()));
  const auto initial=read_sample();
  bool healthy=capture_window(health_ms) && initial.valid;
  double drift=0;bool imu_valid=initial.imu_valid;
  bool motors_valid[4],trackers_valid[2];
  std::copy(initial.motor_valid,initial.motor_valid+4,motors_valid);
  std::copy(initial.tracker_valid,initial.tracker_valid+2,trackers_valid);
  for(unsigned j=0;j<row_count.load();++j){const auto& s=rows[j];
    imu_valid &= s.imu_valid;drift=std::max(drift,std::fabs(s.rotation-initial.rotation));
    for(unsigned i=0;i<4;++i)motors_valid[i] &= s.motor_valid[i];
    for(unsigned i=0;i<2;++i)trackers_valid[i] &= s.tracker_valid[i];
  }
  print_sample("HEALTH START",initial);print_sample("HEALTH END",read_sample());dump();
  imu_status=imu_valid && drift<=1?INCONCLUSIVE:FAIL;
  for(unsigned i=0;i<4;++i)motor_status[i]=motors_valid[i]?INCONCLUSIVE:FAIL;
  for(unsigned i=0;i<2;++i)tracker_status[i]=trackers_valid[i]?INCONCLUSIVE:FAIL;
  std::printf("STATIONARY SCREEN: %s drift=%.4f deg limit=1 deg; physical stillness unverified\n",healthy && drift<=1?"PASS":"FAIL",drift);
  if(!healthy || drift>1)note("passive health/stationary screening failed");
  if(cancelled())return;

  if constexpr(profile==Profile::PASSIVE){
    stage_seen[1]=true;
    std::printf("[STAGE B] MANUAL IMU / TRACKING WHEELS\nAfter countdown, manually rotate CW approximately90deg during the 8-second window, then hold still. No resets.\n");
    print_sample("PRE-COUNTDOWN BASELINE",read_sample());
    if(!countdown("MANUAL CW90: WAIT FOR GO"))return;
    const auto start=read_sample();
    const bool window_valid=capture_window(manual_ms);
    const auto end=read_sample();
    pros::lcd::set_text(1,"MANUAL WINDOW FINISHED");
    print_sample("MANUAL START",start);print_sample("MANUAL END",end);dump();
    const double angle=end.rotation-start.rotation;
    std::printf("MANUAL validity=%d dIMU=%.4f dX=%.4f dY=%.4f\n",window_valid && start.valid && end.valid,
      angle,end.x-start.x,end.y-start.y);
    for(unsigned i=0;i<2;++i)if(auto* t=tracker(i))std::printf("TRACKER P%d d_raw_cd=%.0f d_in=%.5f expected_rotation_magnitude_in=%.5f offset=%.5f diameter=%.4f ratio=%.4f\n",
      t->smart_encoder.get_port(),end.raw[i]-start.raw[i],end.distance[i]-start.distance[i],
      std::fabs(t->distance_to_center_get()*angle*3.141592653589793/180),t->distance_to_center_get(),t->wheel_diameter_get(),t->ratio_get());
    if(!window_valid || !start.valid || !end.valid)note("manual sample invalid or interrupted");
    if(!start.imu_valid || !end.imu_valid)imu_status=FAIL;
    for(unsigned i=0;i<2;++i)if(!start.tracker_valid[i] || !end.tracker_valid[i])tracker_status[i]=FAIL;
    for(unsigned j=0;j<row_count.load();++j){
      if(!rows[j].imu_valid)imu_status=FAIL;
      for(unsigned i=0;i<2;++i)if(!rows[j].tracker_valid[i])tracker_status[i]=FAIL;
      for(unsigned i=0;i<4;++i)if(!rows[j].motor_valid[i])motor_status[i]=FAIL;
    }
    std::printf("MANUAL 90deg correspondence: INCONCLUSIVE (approximate operator motion, no independent angle measurement). Sensor-reported CW90 +/-5 screening: %s\n",
      window_valid && start.valid && end.valid && std::fabs(angle-90)<=5?"PASS":"INCONCLUSIVE");
    std::printf("Tracker geometry/physical direction remain INCONCLUSIVE. PASSIVE ends here; powered stages SKIPPED.\n");
    return; // Compile-time default can never install the powered hook or dispatch motion.
  }
  if constexpr(!power_allowed(profile,motor_bench_physically_verified,controlled_motion_enabled)){
    std::printf("POWERED PROFILE BLOCKED: requires physical bench verification AND explicit controlled_motion_enabled.\n");return;
  }
  if(!healthy || imu_status==FAIL || !nav_sensors_ready() || !ez::movement_ready()){note("powered prerequisites failed");return;}
  if(!ez::diagnostic_service_set(service)){note("diagnostic owner hook busy");return;}
  hooks_owned=true;ez::command_observer_set(observe_commands);
  if constexpr(profile==Profile::MOTOR_BENCH){
    stage_seen[2]=true;
    std::printf("[STAGE C] MOTOR RESPONSE\nSTAND ONLY, all wheels clear. Six automatic 2V/250ms actions. Each has a 5-second countdown. Physical mounting direction cannot be inferred.\n");
    for(unsigned i=0;i<6;++i){
      const unsigned bits=i<4?1u<<i:i==4?3u:12u;char prompt[64];
      std::snprintf(prompt,sizeof(prompt),"STAND %s%d: 2V/250ms",i<4?"P":"group ",i<4?std::abs(motor(i).get_port()):i-3);
      if(!countdown(prompt))return;
      const auto result=execute(PULSE,bits,0);
      if(i<4)motor_status[i]=result;
      std::printf("MOTOR/GROUP mask=%u: %s (encoder response only)\n",bits,names[result]);
      if(result!=PASS){note("motor/group response failed");return;}
    }
    std::printf("BENCH electrical/encoder tests complete; physical verification remains manual. No flags are set automatically. Floor tests SKIPPED.\n");return;
  }
  stage_seen[3]=true;
  std::printf("[STAGE D] AUTONOMOUS PID\nFLOOR clear space. Separate build enable + prior physical bench verification required.\n");
  for(double angle:{turn_deg,-turn_deg}){
    if(!countdown(angle>0?"FLOOR: CW15 speed30":"FLOOR: CCW15 speed30"))return;
    const auto result=execute(Kind::TURN,0,angle);
    if(result!=PASS){turn_status=result;note("turn test failed");return;}
  }
  turn_status=PASS;
  if(!countdown("FLOOR: FRONT3 speed30"))return;
  drive_status=execute(Kind::DRIVE,0,drive_in);
  if(drive_status!=PASS){note("drive test failed");return;}
  if constexpr(!tracker_geometry_physically_verified){std::printf("[STAGE E] ODOMETRY NAVIGATION: SKIPPED; physical tracker geometry unverified\n");stage_seen[4]=true;return;}
  stage_seen[4]=true;
  std::printf("[STAGE E] ODOMETRY NAVIGATION\n");
  if(!countdown("FLOOR: reset0 then +Y3"))return;
  nav_set_start(0,0,0);
  if(!nav_validate_pose(0,0,0,0.5,0.5)){point_status=FAIL;note("point start failed");return;}
  point_status=execute(POINT,0,drive_in);
}
