// Isolated firmware: this translation unit is the ONLY robot application source.
#include "api.h"
#include "subsystems.hpp" // Configuration constants only; normal implementation is not linked.
#include "policy.hpp"
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <limits>
#include <algorithm>
#include <cstring>
using namespace lift_diag;
namespace {
static_assert(mechanism_config::LIFT_MOTOR_SAFETY_LOCKOUT,"Normal lift lockout must stay on");
static_assert(!mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC,"Do not combine diagnostic modes");
static_assert(LIFT_DIAGNOSTIC_MAX_MV>0 && LIFT_DIAGNOSTIC_MAX_MV<=1000,"Diagnostic voltage ceiling");
const int ports[3]={mechanism_config::THIRD_LIFT_PORT,mechanism_config::LEFT_LIFT_PORT,mechanism_config::RIGHT_LIFT_PORT};
const int signs[3]={mechanism_config::THIRD_LIFT_REVERSED?-1:1,mechanism_config::LEFT_LIFT_REVERSED?-1:1,mechanism_config::RIGHT_LIFT_REVERSED?-1:1};
const char* names[3]={"TOP","LEFT","RIGHT"};
// Low bit allows standalone driver mode; upper bits are the lifecycle generation.
std::atomic<unsigned> mode_word{0};
std::atomic<unsigned> clearance_generation{~0u};
void mode_request(bool enable) {
  unsigned old=mode_word.load();
  while(!mode_word.compare_exchange_weak(old,((old+2)&~1u)|(enable?1u:0u))) {}
}
struct MotorSample {
  long raw=0; double total=NAN,delta=NAN,rpm=NAN,current=NAN,temp=NAN;
  double avg_current=NAN,peak_current=0,avg_rpm=NAN,min_abs_rpm=NAN;
  int cmd=0,write_errno=0,activation_write_errno=0,connection_errno=0,encoder_errno=0,rpm_errno=0,current_errno=0,temp_errno=0;
  bool connected=false,valid=false,write_ok=false;
};
struct Frame {
  unsigned sequence=0,now=0,id=0,elapsed=0,generation=0;
  int direction=0,p16_errno=0; bool summary=false,running=false,fault=false,zero_ok=false,all_write_ok=false;
  double p16_raw=NAN,p16=NAN,p16_delta=NAN;
  bool clearance=false,ready=false,can_confirm=false;
  const char* reason="BOOT"; MotorSample m[3];
};
pros::Mutex screen_mutex;
Frame screen_live,screen_result;
bool have_result=false;
constexpr unsigned CAPACITY=64;
Frame queue[CAPACITY];unsigned head=0,tail=0,count=0;
pros::Mutex queue_mutex;
std::atomic<unsigned> queue_free{CAPACITY},dropped{0};
bool publish(const Frame& f) {
  if(!queue_mutex.take(0))return false;
  const bool ok=count<CAPACITY;
  if(ok){queue[head]=f;head=(head+1)%CAPACITY;++count;queue_free.store(CAPACITY-count);}
  queue_mutex.give();return ok;
}
bool consume(Frame& f) {
  if(!queue_mutex.take(0))return false;
  const bool ok=count!=0;
  if(ok){f=queue[tail];tail=(tail+1)%CAPACITY;--count;queue_free.store(CAPACITY-count);}
  queue_mutex.give();return ok;
}
// Fixed history remains available in RAM; serial output is optional and off by default.
Frame history[128]; unsigned history_next=0,history_count=0,history_overwritten=0;
void logger(void*) {
  if(TERMINAL_TELEMETRY) std::printf("LIFT_DIAG ISOLATED relative_only=1 max_mv=%d runtime_ms=%u\n",LIFT_DIAGNOSTIC_MAX_MV,MAX_ACTIVATION_MS);
  while(true) {
    Frame f;
    if(!consume(f)){pros::delay(20);continue;}
    history[history_next]=f;history_next=(history_next+1)%128;
    if(history_count<128)++history_count;else ++history_overwritten;
    if(!TERMINAL_TELEMETRY)continue;
    // All formatting/terminal operations happen AFTER releasing the snapshot mutex.
    std::printf("LIFT_%s seq=%u id=%u t_ms=%u elapsed_ms=%u direction=%s state=%s reason=%s fault=%d generation=%u P16_deg=%.3f referenced_deg=%.3f P16_delta_deg=%.3f p16_errno=%d writes_ok=%d zero_ack=%d dropped=%u\n",
      f.summary?"SUMMARY":"TEST",f.sequence,f.id,f.now,f.elapsed,f.direction>0?"UP":f.direction<0?"DOWN":"NONE",
      f.running?"RUNNING":f.fault?"FAULT":"STOPPED",f.reason,f.fault,f.generation,f.p16_raw,f.p16,f.p16_delta,f.p16_errno,f.all_write_ok,f.zero_ok,dropped.load());
    bool all_moved=true;
    for(int n=0;n<3;++n){const auto& m=f.m[n];all_moved=all_moved&&std::isfinite(m.total)&&std::fabs(m.total)>=2;
      std::printf("P%d_%s cmd_mv=%d activation_cmd_mv=%d cmd_up_sign=%d raw_counts=%ld total_counts=%.0f sample_delta_counts=%.0f rpm=%+.2f current_ma=%.0f temp_C=%.1f connected=%d valid=%d conn_errno=%d enc_errno=%d rpm_errno=%d current_errno=%d temp_errno=%d write_ok=%d write_errno=%d activation_write_errno=%d avg_current_ma=%.1f peak_current_ma=%.0f avg_rpm=%+.2f min_abs_rpm=%.2f\n",
        ports[n],names[n],m.cmd,f.direction*signs[n]*LIFT_DIAGNOSTIC_MAX_MV,signs[n],m.raw,m.total,m.delta,m.rpm,m.current,m.temp,m.connected,m.valid,m.connection_errno,m.encoder_errno,m.rpm_errno,m.current_errno,m.temp_errno,m.write_ok,m.write_errno,m.activation_write_errno,m.avg_current,m.peak_current,m.avg_rpm,m.min_abs_rpm);
    }
    if(f.summary)std::printf("LIFT_SUMMARY all_encoders_moved=%d lift_displacement_observed=%d mechanical_direction=UNDETERMINED final_sample_before_zero=1 physical_rest=UNVERIFIED\n",all_moved,std::isfinite(f.p16_delta)&&std::fabs(f.p16_delta)>=0.25);
  }
}
void control(void*) {
  // Only this persistent task creates lift devices or writes their outputs.
  pros::Motor motors[3]={pros::Motor(ports[0],pros::v5::MotorGears::green),pros::Motor(ports[1],pros::v5::MotorGears::green),pros::Motor(ports[2],pros::v5::MotorGears::green)};
  pros::Rotation sensor(mechanism_config::LIFT_ROTATION_REVERSED?-mechanism_config::LIFT_ROTATION_PORT:mechanism_config::LIFT_ROTATION_PORT);
  pros::Controller controller(pros::E_CONTROLLER_MASTER);
  Policy policy; Frame f,pending;bool summary_pending=false,reference_sampled=false;
  double reference_raw=NAN,start_raw[3]{},last_report_raw[3]{NAN,NAN,NAN},start_p16=NAN;
  double sum_current[3]{},peak[3]{},sum_rpm[3]{},min_rpm[3]{};unsigned samples=0,id=0,seq=0,last_log=0;
  auto write=[&](int mv,unsigned generation) {
    bool ok=true;
    if(mv && (!POWERED_TEST_APPROVED || clearance_generation.load()!=generation)) return false;
    for(int n=0;n<3;++n) {
      // A lifecycle request during a write batch causes this batch to stop; there is no request queue.
      if(mv && (mode_word.load()!=generation || pros::competition::is_connected() || pros::competition::is_disabled() || pros::competition::is_autonomous())) {ok=false;break;}
      const int command=std::clamp(mv*signs[n],-LIFT_DIAGNOSTIC_MAX_MV,LIFT_DIAGNOSTIC_MAX_MV);
      errno=0;const auto result=motors[n].move_voltage(command);f.m[n].write_errno=errno;f.m[n].write_ok=result!=PROS_ERR;if(!f.m[n].write_ok)f.m[n].activation_write_errno=f.m[n].write_errno?f.m[n].write_errno:EIO;f.m[n].cmd=command;ok=ok&&f.m[n].write_ok;
    }
    if(mv && mode_word.load()!=generation) ok=false;
    f.all_write_ok=ok;f.zero_ok=mv==0&&ok;return ok;
  };
  // Zero first. No encoder reset, P16 reset, brake-mode change, or unrelated device initialization.
  if(!write(0,0))policy.stop("INITIAL_STOP_WRITE_FAILED",true);
  while(true) {
    if(summary_pending && publish(pending))summary_pending=false;
    const unsigned word=mode_word.load();Input in;in.generation=word;
    errno=0;const int controller_connected=controller.is_connected();const bool cc_ok=controller_connected==1&&errno==0;
    errno=0;const int up=controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);const bool up_ok=up!=PROS_ERR&&errno==0;
    errno=0;const int down=controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);const bool down_ok=down!=PROS_ERR&&errno==0;
    in.up=up==1;in.down=down==1;
    in.permitted=(word&1)&&cc_ok&&up_ok&&down_ok&&!pros::competition::is_connected()&&!pros::competition::is_disabled()&&!pros::competition::is_autonomous();
    in.approved=POWERED_TEST_APPROVED;
    in.reference=clearance_generation.load()==word;
    in.stopping_margin=STOPPING_MARGIN_DEG;
    in.minimum=-RELATIVE_ENVELOPE_DEG;in.maximum=RELATIVE_ENVELOPE_DEG;
    in.buffer_ok=!summary_pending&&queue_free.load()>4;
    errno=0;const auto p16=sensor.get_position();f.p16_errno=errno;
    const bool p16_valid=p16!=PROS_ERR&&f.p16_errno==0;
    f.p16_raw=p16_valid?p16/100.0:NAN;
    const bool first_reference=!reference_sampled&&p16_valid;
    if(first_reference){reference_sampled=true;reference_raw=f.p16_raw;}
    f.p16=f.p16_raw-reference_raw; // Captured ONCE; never reset by a button or confirmation.
    in.p16=f.p16;in.valid=p16_valid;
    for(int n=0;n<3;++n){auto& m=f.m[n];
      errno=0;m.connected=motors[n].is_installed();m.connection_errno=errno;
      std::uint32_t timestamp=0;errno=0;m.raw=motors[n].get_raw_position(&timestamp);m.encoder_errno=errno;
      errno=0;m.rpm=motors[n].get_actual_velocity();m.rpm_errno=errno;
      errno=0;m.current=motors[n].get_current_draw();m.current_errno=errno;
      errno=0;m.temp=motors[n].get_temperature();m.temp_errno=errno;
      m.valid=m.raw!=PROS_ERR&&m.encoder_errno==0&&std::isfinite(m.rpm)&&m.rpm!=PROS_ERR_F&&m.rpm_errno==0&&m.current!=PROS_ERR&&m.current>=0&&m.current_errno==0&&std::isfinite(m.temp)&&m.temp!=PROS_ERR_F&&m.temp_errno==0;
      in.connected=in.connected&&m.connected&&m.connection_errno==0;in.valid=in.valid&&m.valid;
      in.rpm[n]=m.rpm;in.current[n]=m.current;in.temperature[n]=m.temp;
    }
    if(first_reference)for(int n=0;n<3;++n)start_raw[n]=last_report_raw[n]=f.m[n].raw;
    in.now=pros::millis();const bool was_running=policy.running;
    bool resting=true;for(int n=0;n<3;++n)resting=resting&&std::fabs(in.rpm[n])<STALL_RPM;
    f.can_confirm=in.permitted&&in.valid&&in.connected&&!policy.fault&&!was_running&&
      !in.up&&!in.down&&resting&&f.zero_ok&&std::fabs(f.p16)<RELATIVE_ENVELOPE_DEG;
    // UI confirmation is generation-bound. Reject a stale/unsafe confirmation.
    if(in.reference && (!in.permitted || policy.fault)) {clearance_generation.store(~0u);in.reference=false;}
    if(mode_word.load()!=word)in.permitted=false;
    int mv=policy.step(in);
    const bool started=!was_running&&policy.running;
    if(started){++id;samples=0;start_p16=f.p16_raw;for(int n=0;n<3;++n){f.m[n].activation_write_errno=0;start_raw[n]=last_report_raw[n]=f.m[n].raw;sum_current[n]=peak[n]=sum_rpm[n]=0;min_rpm[n]=std::numeric_limits<double>::infinity();}}
    if(was_running||started){++samples;for(int n=0;n<3;++n){sum_current[n]+=f.m[n].current;peak[n]=std::max(peak[n],f.m[n].current);sum_rpm[n]+=f.m[n].rpm;min_rpm[n]=std::min(min_rpm[n],std::fabs(f.m[n].rpm));}}
    if(!write(mv,word)) {
      bool hardware_error=false;for(int n=0;n<3;++n)hardware_error=hardware_error||f.m[n].activation_write_errno!=0;
      policy.stop(hardware_error?"MOTOR_WRITE_FAULT":"MODE_CHANGED_DURING_WRITE",hardware_error);
      mv=0;if(!write(0,word))policy.stop("STOP_WRITE_FAILED",true);
    }
    // Stop is acknowledged only by the owner after all three zero writes succeed.
    f.now=in.now;f.id=id;f.direction=policy.direction;f.elapsed=id?in.now-policy.began:0;f.running=policy.running;f.fault=policy.fault;f.reason=policy.reason;f.generation=word;
    f.p16_delta=f.p16_raw-start_p16;
    for(int n=0;n<3;++n){auto& m=f.m[n];m.total=reference_sampled?m.raw-start_raw[n]:NAN;m.delta=m.raw-last_report_raw[n];m.avg_current=samples?sum_current[n]/samples:NAN;m.peak_current=peak[n];m.avg_rpm=samples?sum_rpm[n]/samples:NAN;m.min_abs_rpm=samples?min_rpm[n]:NAN;}
    if((was_running||started)&&!policy.running){f.summary=true;f.sequence=++seq;pending=f;summary_pending=!publish(pending);}
    else if(in.now-last_log>=LIFT_DIAGNOSTIC_LOG_PERIOD_MS){f.summary=false;f.sequence=++seq;if(!publish(f)){++dropped;policy.stop("LOGGER_BACKPRESSURE");if(!write(0,word))policy.stop("STOP_WRITE_FAILED",true);if(was_running||started){f.running=false;f.fault=policy.fault;f.reason=policy.reason;f.summary=true;pending=f;summary_pending=true;}}last_log=in.now;for(int n=0;n<3;++n)last_report_raw[n]=f.m[n].raw;}
    f.clearance=in.reference;
    f.ready=policy.armed&&!policy.fault&&in.reference&&std::isfinite(STOPPING_MARGIN_DEG)&&STOPPING_MARGIN_DEG>0&&STOPPING_MARGIN_DEG<RELATIVE_ENVELOPE_DEG;
    if(screen_mutex.take(0)) {
      screen_live=f;
      if((was_running||started)&&!policy.running){screen_result=f;have_result=true;}
      screen_mutex.give();
    }
    pros::delay(CONTROL_PERIOD_MS);
  }
}
// Screen task only: never writes a motor; releases snapshot lock before screen IO.
void display(void*) {
  bool touched=false,detail=false;
  while(true) {
    Frame live,result;bool saved=false;
    if(!screen_mutex.take(0)){pros::delay(30);continue;}
    live=screen_live;result=screen_result;saved=have_result;screen_mutex.give();
    const bool fresh=pros::millis()-live.now<300;
    const auto touch=pros::screen::touch_status();
    const bool down=touch.touch_status==pros::E_TOUCH_PRESSED||touch.touch_status==pros::E_TOUCH_HELD;
    if(down&&!touched&&fresh) {
      if(!live.clearance&&live.can_confirm&&touch.y>=180&&touch.y<240)
        clearance_generation.store(live.generation);
      else if(live.clearance&&touch.y>=200)detail=!detail;
    }
    touched=down;
    pros::screen::set_pen(0x000000);pros::screen::fill_rect(0,0,479,239);
    pros::screen::set_pen(0xFFFFFF);
    const char* state=!fresh?"STALE":live.fault?"FAULT":live.running?(live.direction>0?"UP":"DOWN"):live.ready?"READY":"STOPPED";
    pros::screen::print(pros::E_TEXT_MEDIUM,0,1,"Lift Conflict | %s",state);
    if(!live.clearance&&!saved) {
      pros::screen::print(pros::E_TEXT_SMALL,0,28,"Startup-relative P16: %+.3f deg (NOT home)",live.p16);
      pros::screen::print(pros::E_TEXT_SMALL,0,48,"Support lift. Verify clearance BOTH directions.");
      pros::screen::print(pros::E_TEXT_SMALL,0,66,"Cutoff +/-3 deg; +/-5 is SOFTWARE envelope.");
      for(int n=0;n<3;++n)pros::screen::print(pros::E_TEXT_SMALL,0,88+n*20,"P%d %s RPM%+.1f mA%.0f valid%d",ports[n],names[n],live.m[n].rpm,live.m[n].current,live.m[n].valid);
      pros::screen::print(pros::E_TEXT_SMALL,0,153,"%s zero_ack=%d",live.reason,live.zero_ok);
      pros::screen::print(pros::E_TEXT_SMALL,0,175,"UNVERIFIED margin: lift may move after STOP.");
      pros::screen::print(pros::E_TEXT_MEDIUM,0,202,live.can_confirm?"TAP: SUPPORTED + CLEAR":"WAIT: healthy sensors / release / zero");
    } else {
      const Frame& f=(!live.running&&saved)?result:live;
      const char* diagnosis="INSUFFICIENT DATA";
      for(int n=0;n<3;++n)if(!f.m[n].connected)diagnosis="MOTOR DISCONNECTED";
      if(std::strcmp(f.reason,"UNEXPECTED_P16_MOTION")==0)diagnosis="UNEXPECTED DIRECTION / TRAVEL";
      // A single outlier is suspect, never automatically diagnosed as wrong polarity.
      if(f.id)for(int n=0;n<3;++n){int a=(n+1)%3,b=(n+2)%3;
        if(std::isfinite(f.m[n].total)&&std::fabs(f.m[n].total)<2&&std::fabs(f.m[a].total)>=2&&std::fabs(f.m[b].total)>=2&&f.m[n].peak_current>MAX_CURRENT_MA)
          diagnosis=n==0?"P13 SUSPECT":n==1?"P14 SUSPECT":"P15 SUSPECT";
      }
      pros::screen::print(pros::E_TEXT_SMALL,0,26,"LIVE: %s zero=%d",live.reason,live.zero_ok);
      pros::screen::print(pros::E_TEXT_SMALL,0,44,"%s #%u: %s",(!live.running&&saved)?"LAST":"LIVE",f.id,diagnosis);
      for(int n=0;n<3;++n){const auto& m=f.m[n];const int y=64+n*36;
        if(!detail){
          pros::screen::print(pros::E_TEXT_SMALL,0,y,"P%d %s test%+d RPM%+.1f",ports[n],names[n],f.id?f.direction*signs[n]*LIFT_DIAGNOSTIC_MAX_MV:0,m.rpm);
          pros::screen::print(pros::E_TEXT_SMALL,0,y+16,"mA%.0f enc%+.0f T%.1fC",m.current,m.total,m.temp);
        }else{
          pros::screen::print(pros::E_TEXT_SMALL,0,y,"P%d raw%ld valid%d conn%d",ports[n],m.raw,m.valid,m.connected);
          pros::screen::print(pros::E_TEXT_SMALL,0,y+16,"peak%.0fmA avgRPM%+.1f write%d err%d",m.peak_current,m.avg_rpm,m.write_ok,m.activation_write_errno);
        }
      }
      pros::screen::print(pros::E_TEXT_SMALL,0,174,"P16 live%+.2f last%+.2f deg",live.p16,f.p16);
      pros::screen::print(pros::E_TEXT_SMALL,0,190,"STOP: %.38s",f.reason);
      pros::screen::print(pros::E_TEXT_SMALL,0,211,"L1 UP / L2 DOWN / release STOP | TAP PAGE");
      pros::screen::print(pros::E_TEXT_SMALL,0,226,"UNVERIFIED stopping margin | dropped %u",dropped.load());
    }
    pros::delay(100);
  }
}

}
extern "C" void initialize() {
  static pros::Task owner(control,nullptr,TASK_PRIORITY_DEFAULT+1,TASK_STACK_DEPTH_DEFAULT,"Lift diag owner");
  static pros::Task log(logger,nullptr,TASK_PRIORITY_DEFAULT-2,TASK_STACK_DEPTH_DEFAULT,"Lift diag logger");
  static pros::Task screen(display,nullptr,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"Lift diag screen");
  (void)owner;(void)log;(void)screen;
}
extern "C" void disabled(){mode_request(false);}
extern "C" void competition_initialize(){mode_request(false);}
extern "C" void autonomous(){mode_request(false);}
extern "C" void opcontrol(){mode_request(true);while(true)pros::delay(100);}
