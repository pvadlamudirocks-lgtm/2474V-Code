#include "full_system_diag.hpp"
#include "full_system_diag_limits.hpp"
#include "main.h"
#include "navigation.hpp"
#include "autons.hpp"
#include <atomic>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdarg>
namespace {
using namespace full_diag;
enum Status { PASS, FAIL, INCONCLUSIVE, SKIPPED };
const char* names[]={"PASS","FAIL","INCONCLUSIVE","SKIPPED"};
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
Sample rows[capacity];
std::atomic<unsigned> row_count{0};
unsigned began=0;
int lcmd=0,rcmd=0; // Passive capture never requests output.
bool stage_seen[5]={};
Status imu_status=SKIPPED,motor_status[4]={SKIPPED,SKIPPED,SKIPPED,SKIPPED};
Status tracker_status[2]={SKIPPED,SKIPPED},turn_status=SKIPPED,drive_status=SKIPPED,point_status=SKIPPED;
const char* first_failure=nullptr;
Sample manual_start,manual_end;
bool manual_captured=false,manual_valid=false,summary_printed=false;
Status health_status=SKIPPED,stationary_status=SKIPPED,manual_screen=SKIPPED;
double stationary_drift=NAN;
bool output_failed=false;
// Fixed-size lines, no retries. PROS serial writes have no verified hard deadline.
void diag_print(const char* format,...){
  if(output_failed)return;
  char line[512];va_list args;va_start(args,format);
  const int n=std::vsnprintf(line,sizeof(line),format,args);va_end(args);
  if(n<0 || static_cast<unsigned>(n)>=sizeof(line)){output_failed=true;return;}
  if(std::printf("%s",line)<0){output_failed=true;return;}
  pros::delay(20); // Report pacing only; never inside sensor acquisition.
}
pros::Motor& motor(unsigned i){return i<2?chassis.left_motors[i]:chassis.right_motors[i-2];}
ez::tracking_wheel* tracker(unsigned i){return i?chassis.odom_tracker_right:chassis.odom_tracker_back;}
bool cancelled(){return pros::competition::is_disabled() || !pros::competition::is_autonomous() ||
  (master.is_connected()==1 && master.get_digital(pros::E_CONTROLLER_DIGITAL_B)==1);}
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
  s.read_end_ms=pros::millis();
  return s;
}
bool countdown(const char* instruction){
  diag_print("OPERATOR: %s; starts in 5 seconds. B/disable cancels.\n",instruction);
  pros::lcd::set_text(0,"FULL SYSTEM DIAG");pros::lcd::set_text(1,instruction);
  const auto start=pros::millis();unsigned shown=0;
  while(!expired(pros::millis(),start,countdown_ms)){
    if(cancelled()){note("cancelled during countdown");return false;}
    const unsigned elapsed=pros::millis()-start;
    if(elapsed>=countdown_ms)break;
    const unsigned remaining=(countdown_ms-elapsed+999)/1000;
    if(remaining!=shown){shown=remaining;pros::lcd::print(2,"START IN %u | B CANCEL",remaining);diag_print("COUNTDOWN %u\n",remaining);}
    pros::delay(10);
  }
  pros::lcd::set_text(2,"GO | B/disable cancels");
  return !cancelled();
}
void dump(){
  if(!detailed_trace || output_failed || cancelled())return;
  const unsigned count=row_count.load(), selected=std::min(count,trace_max_rows);
  diag_print("OPTIONAL TRACE %u/%u manual samples: ms,span,valid,x,y,h,IMU,rawH,rawV\n",selected,count);
  for(unsigned row=0;row<selected && !output_failed && !cancelled();++row){
    const auto& s=rows[trace_index(row,count,selected)];
    diag_print("TRACE %u,%u,%d,%.6g,%.6g,%.6g,%.6g,%.0f,%.0f\n",
      s.ms-began,s.read_end_ms-s.ms,s.valid,s.x,s.y,s.h,s.rotation,s.raw[0],s.raw[1]);
    pros::delay(trace_print_interval_ms);
  }
}
void summary(){
  if(summary_printed)return;
  summary_printed=true;
  diag_print("\n=== FULL_SYSTEM_DIAG SUMMARY (PASSIVE) ===\n");
  diag_print("HEALTH validity: %s | STATIONARY: %s drift=%.6g deg (existing 1-deg screen)\n",
    names[health_status],names[stationary_status],stationary_drift);
  if(manual_captured){
    const auto& a=manual_start;const auto& b=manual_end;
    diag_print("MANUAL validity: %s | sensor CW90 +/-5 screen: %s\n",manual_valid?"PASS":"FAIL",names[manual_screen]);
    diag_print("IMU rotation=%.6g->%.6g delta=%+.6g deg; heading_end=%.6g; valid_start/end=%d/%d\n",
      a.rotation,b.rotation,b.rotation-a.rotation,b.heading,a.imu_valid,b.imu_valid);
    for(unsigned i=0;i<2;++i)diag_print("TRACKER %s P%d valid=%d/%d raw_cd=%.0f->%.0f delta=%+.0f; in=%.6g->%.6g delta=%+.6g; errno=%d/%d\n",
      i?"V":"H",tracker(i)?tracker(i)->smart_encoder.get_port():0,a.tracker_valid[i],b.tracker_valid[i],
      a.raw[i],b.raw[i],a.tracker_valid[i]&&b.tracker_valid[i]?b.raw[i]-a.raw[i]:NAN,
      a.distance[i],b.distance[i],b.distance[i]-a.distance[i],a.error[i],b.error[i]);
    diag_print("ODOM start=(%.6g,%.6g,%.6g) end=(%.6g,%.6g,%.6g); dX=%+.6g dY=%+.6g in\n",
      a.x,a.y,a.h,b.x,b.y,b.h,b.x-a.x,b.y-a.y);
    diag_print("MANUAL elapsed=%u ms samples=%u all-sample validity=%d (includes motors/pose)\n",b.ms-a.ms,row_count.load(),manual_valid);
  }else diag_print("MANUAL: SKIPPED (window not completed)\n");
  diag_print("IMU qualification: %s | H tracker: %s | V tracker: %s\n",names[imu_status],names[tracker_status[0]],names[tracker_status[1]]);
  diag_print("MOTOR qualification [L0,L1,R0,R1]: %s,%s,%s,%s (passive only)\n",names[motor_status[0]],names[motor_status[1]],names[motor_status[2]],names[motor_status[3]]);
  diag_print("Physical angle/geometry: INCONCLUSIVE; powered motor/turn/drive/navigation: SKIPPED\n");
  diag_print("First issue: %s\n",first_failure?first_failure:"none detected; not physical qualification");
  diag_print("=== DIAGNOSTIC SUMMARY END ===\n");
  pros::lcd::set_text(2,output_failed?"SUMMARY OUTPUT ERROR":"SUMMARY READY / PASSIVE");
}
}
bool capture_window(unsigned duration){
  row_count.store(0);began=pros::millis();

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
  output_failed=summary_printed=manual_captured=manual_valid=false;
  health_status=stationary_status=manual_screen=SKIPPED;stationary_drift=NAN;
  diag_print("\n========== 2474V FULL SYSTEM DIAG START ==========\nPASSIVE ONLY: no motor commands or navigation hooks.\n");
  first_failure=nullptr;imu_status=turn_status=drive_status=point_status=SKIPPED;
  for(auto& status:motor_status)status=SKIPPED;
  for(auto& status:tracker_status)status=SKIPPED;
  for(auto& seen:stage_seen)seen=false;
  struct Report {~Report(){summary();}} report;
  odom_display_freeze();
  stage_seen[0]=true;
  pros::lcd::set_text(0,"PASSIVE DIAGNOSTIC");pros::lcd::set_text(1,"HEALTH: STILL FOR 2s");
  diag_print("[STAGE A] SENSOR HEALTH\nKeep stationary for 2 seconds. IMU P%d installed=%d calibrating=%d status=%d\n",
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

  stationary_drift=drift;health_status=healthy?PASS:FAIL;
  stationary_status=healthy && drift<=1?PASS:FAIL;
  imu_status=imu_valid && drift<=1?INCONCLUSIVE:FAIL;
  for(unsigned i=0;i<4;++i)motor_status[i]=motors_valid[i]?INCONCLUSIVE:FAIL;
  for(unsigned i=0;i<2;++i)tracker_status[i]=trackers_valid[i]?INCONCLUSIVE:FAIL;
  diag_print("STATIONARY SCREEN: %s drift=%.6g deg limit=1 deg; physical stillness unverified\n",healthy && drift<=1?"PASS":"FAIL",drift);
  if(!healthy || drift>1)note("passive health/stationary screening failed");
  if(cancelled())return;

    stage_seen[1]=true;
    diag_print("[STAGE B] MANUAL IMU / TRACKING WHEELS\nAfter countdown, manually rotate CW approximately90deg during the 8-second window, then hold still. No resets.\n");

    if(!countdown("MANUAL CW90: WAIT FOR GO"))return;
    const auto start=read_sample();
    const bool window_valid=capture_window(manual_ms);
    const auto end=read_sample();
    pros::lcd::set_text(1,"MANUAL WINDOW FINISHED");
    manual_start=start;manual_end=end;manual_captured=true;
    manual_valid=window_valid && start.valid && end.valid;
    const double angle=end.rotation-start.rotation;
    manual_screen=!manual_valid?FAIL:std::fabs(angle-90)<=5?PASS:INCONCLUSIVE;
    if(!window_valid || !start.valid || !end.valid)note("manual sample invalid or interrupted");
    if(!start.imu_valid || !end.imu_valid)imu_status=FAIL;
    for(unsigned i=0;i<2;++i)if(!start.tracker_valid[i] || !end.tracker_valid[i])tracker_status[i]=FAIL;
    for(unsigned j=0;j<row_count.load();++j){
      if(!rows[j].imu_valid)imu_status=FAIL;
      for(unsigned i=0;i<2;++i)if(!rows[j].tracker_valid[i])tracker_status[i]=FAIL;
      for(unsigned i=0;i<4;++i)if(!rows[j].motor_valid[i])motor_status[i]=FAIL;
    }
    summary(); // All measurements/classifications first, before optional history.
    dump();
}
