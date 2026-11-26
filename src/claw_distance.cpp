#include "subsystems.hpp"
#include "claw_distance.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <atomic>
#include <cerrno>
#include <cstdio>

// One cooperative sampler/controller, no background task and no second ADI writer.
namespace claw_distance {
namespace {
using namespace mechanism_config;
pros::Mutex gate;
std::atomic<unsigned> generation{0};
std::atomic<pros::task_t> automatic_task{nullptr};
std::atomic<auto_clamp::State> reported_state{auto_clamp::State::Disabled};
std::atomic<const char*> reason{"DISABLED"};
std::atomic<bool> in_range{false};
std::atomic<unsigned> detection_at{0}, detection_freshness{0};
unsigned consumed_sample=0;bool consumed=false;
auto_clamp::Detector detector;
unsigned seen_generation=0, close_generation=0;
int selected_mode=-1;
bool autonomous_armed=false;
struct Snapshot {
  Status status=Status::Unconfigured;
  int mm=0, confidence=0, error=0;
  unsigned at=0;
  bool sampled=false, confidence_available=false;
} reading;
unsigned last_print=0;
bool printed=false;
struct Lock { bool held=gate.take(0); ~Lock(){if(held)gate.give();} };
bool port_valid(){return CLAW_DISTANCE_PORT>=1 && CLAW_DISTANCE_PORT<=21;}
pros::Distance* sensor() {
  if(!port_valid())return nullptr; // NEVER construct port zero.
  static pros::Distance device(CLAW_DISTANCE_PORT);
  return &device;
}
int mode(){return pros::competition::is_disabled()?0:pros::competition::is_autonomous()?2:1;}
auto_clamp::Config config(bool autonomous) {
  if(autonomous)return {AUTON_AUTO_CLAMP_MIN_MM,AUTON_AUTO_CLAMP_MAX_MM,
    AUTON_AUTO_CLAMP_READINGS,AUTON_AUTO_CLAMP_SAMPLE_MS,AUTON_AUTO_CLAMP_FRESHNESS_MS,
    AUTON_AUTO_CLAMP_TIMEOUT_MS,CLAW_DETECT_REMOVAL_READINGS};
  return {CLAW_DETECT_MIN_MM,CLAW_DETECT_MAX_MM,CLAW_DETECT_CONSISTENT_READINGS,
    CLAW_DISTANCE_SAMPLE_MS,CLAW_DISTANCE_FRESHNESS_MS,CLAW_AUTO_CLAMP_TIMEOUT_MS,
    CLAW_DETECT_REMOVAL_READINGS};
}
bool allowed(bool autonomous) {
  return port_valid() && claw_pneumatic_available() && config(autonomous).valid() &&
    (autonomous ? ENABLE_AUTON_AUTO_CLAMP && AUTON_AUTO_CLAMP_RANGE_CALIBRATED &&
      AUTON_AUTO_CLAMP_POLARITY_VERIFIED && mode()==2 && !ez::autonomous_cancelled()
      : ENABLE_DRIVER_AUTO_CLAMP && CLAW_DISTANCE_CALIBRATED &&
      CLAW_AUTO_CLAMP_POLARITY_VERIFIED && mode()==1);
}
void reconcile() {
  const int current=mode(); const unsigned epoch=generation.load();
  if(current!=selected_mode || epoch!=seen_generation) {
    // A write failure stays faulted until a program restart; cleanup never clears it.
    const bool fault=detector.state==auto_clamp::State::Fault;
    detector.disarm();
    if(!fault)detector.state=auto_clamp::State::Idle;
    autonomous_armed=false; in_range=false;consumed=false;
    selected_mode=current;seen_generation=epoch;
  }
}
void sample(unsigned interval) {
  const unsigned now=pros::millis();
  if(reading.sampled && now-reading.at<interval)return;
  reading=Snapshot{};reading.at=now;reading.sampled=true;
  auto* device=sensor();
  if(!device)return;
  errno=0;
  const bool installed=device->is_installed();
  if(!installed){reading.status=Status::Disconnected;reading.error=errno;return;}
  errno=0;reading.mm=device->get();reading.error=errno;
  int confidence_error=0;
  if(reading.error==0 && reading.mm>200 && reading.mm!=9999 && reading.mm!=PROS_ERR){
    errno=0;reading.confidence=device->get_confidence();confidence_error=errno;
    reading.confidence_available=confidence_error==0 && reading.confidence!=PROS_ERR;
  }
  reading.status=classify(true,true,reading.mm,reading.error,reading.confidence,
                         confidence_error,CLAW_DETECT_MIN_CONFIDENCE);
  if(confidence_error)reading.error=confidence_error;
}
const char* status_name(Status status){
  switch(status){
    case Status::Unconfigured:return "UNCONFIGURED";
    case Status::Disconnected:return "DISCONNECTED";
    case Status::Error:return "READ_ERROR";
    case Status::NoObject:return "NO_OBJECT";
    case Status::Invalid:return "INVALID";
    case Status::LowConfidence:return "LOW_CONFIDENCE";
    case Status::Valid:return "VALID";
  }return "INVALID";
}
}
void cancel(){
  generation.fetch_add(1);in_range=false;
  reported_state=auto_clamp::State::Disabled;reason="CANCELLED_OR_CLEANUP";
}
CommandGuard::CommandGuard(){
  if(automatic_task.load()==pros::c::task_get_current()){
    nested_=true;
    const auto c=config(selected_mode==2);
    acquired_=generation.load()==close_generation && allowed(selected_mode==2) &&
      reading.status==Status::Valid && pros::millis()-reading.at<=c.freshness_ms &&
      detector.check(pros::millis(),c,true);
    return;
  }
  // Revoke automatic intent BEFORE trying the lock. Busy means rejected, not queued.
  generation.fetch_add(1);in_range=false;
  reported_state=auto_clamp::State::Idle;reason="EXPLICIT_CLAW_OVERRIDE";
  acquired_=gate.take(0);
  if(acquired_)reconcile();
}
CommandGuard::~CommandGuard(){if(acquired_&&!nested_)gate.give();}
void diagnostic_update(){
  if(!ENABLE_CLAW_DISTANCE_DIAGNOSTIC)return;
  const unsigned now=pros::millis();
  if(printed && now-last_print<CLAW_DISTANCE_DIAGNOSTIC_MS)return;
  Snapshot copy;
  {Lock lock;if(!lock.held)return;sample(CLAW_DISTANCE_DIAGNOSTIC_MS);copy=reading;}
  printed=true;last_print=now;
  // Printing is outside the gate, never inside a motor-owner callback.
  const bool numeric=copy.status==Status::Valid || copy.status==Status::LowConfidence;
  if(numeric){
    if(copy.confidence_available)
      std::printf("CLAW_DISTANCE port=%d distance_mm=%d confidence=%d status=%s age_ms=%u errno=%d\n",
        CLAW_DISTANCE_PORT,copy.mm,copy.confidence,status_name(copy.status),pros::millis()-copy.at,copy.error);
    else std::printf("CLAW_DISTANCE port=%d distance_mm=%d confidence=NA status=%s age_ms=%u errno=%d\n",
        CLAW_DISTANCE_PORT,copy.mm,status_name(copy.status),pros::millis()-copy.at,copy.error);
  }else std::printf("CLAW_DISTANCE port=%d status=%s errno=%d\n",CLAW_DISTANCE_PORT,status_name(copy.status),copy.error);
}
void update(){
  Lock lock;if(!lock.held)return;
  reconcile();
  const bool autonomous=selected_mode==2;
  if(!allowed(autonomous)){
    detector.disarm();autonomous_armed=false;in_range=false;
    reported_state=detector.state==auto_clamp::State::Fault?detector.state:auto_clamp::State::Disabled;
    reason="DISABLED_UNCALIBRATED_OR_MODE";return;
  }
  const auto c=config(autonomous);const unsigned epoch=generation.load();
  if(!autonomous && detector.state==auto_clamp::State::Idle)
    detector.arm(pros::millis(),c,true);
  if(autonomous && !autonomous_armed){reported_state=detector.state;return;}
  if(!detector.check(pros::millis(),c,true)){
    in_range=false;reported_state=detector.state;reason=detector.state==auto_clamp::State::Timeout?"TIMEOUT":"NOT_ARMED";return;
  }
  if(!detector.due(pros::millis(),c))return;
  sample(c.sample_ms);
  const unsigned now=pros::millis();
  if(consumed && consumed_sample==reading.at)return;
  consumed=true;consumed_sample=reading.at;detection_at=reading.at;detection_freshness=c.freshness_ms;
  const bool valid=reading.status==Status::Valid && now-reading.at<=c.freshness_ms;
  in_range=valid && reading.mm>=c.min_mm && reading.mm<=c.max_mm;
  const bool trigger=detector.sample(now,c,valid,reading.mm);
  reason=valid?(detector.removal_confirmed?"WAITING_ENTRY":"WAITING_CONFIRMED_REMOVAL"):"INVALID_OR_STALE_SENSOR";
  if(trigger){
    if(epoch!=generation.load() || !allowed(autonomous) || !detector.check(pros::millis(),c,true)){
      detector.disarm();reason="CANCELLED_BEFORE_CLOSE";
    }else if(claw_commanded_closed()){
      detector.finish(true);reason="ALREADY_COMMANDED_CLOSED";
    }else{
      close_generation=epoch;automatic_task=pros::c::task_get_current();
      const bool ok=claw_close(); // Existing guarded polarity/output path, exactly once.
      automatic_task=nullptr;
      detector.finish(ok);reason=ok?"CLOSE_COMMAND_SUCCEEDED_NOT_POSITION_FEEDBACK":"CLOSE_COMMAND_FAILED";
    }
  }
  reported_state=detector.state;
}
bool detected(){return in_range.load() && pros::millis()-detection_at.load()<=detection_freshness.load();}
}
// Public autonomous APIs: explicit arming only; no route is changed to call them.
bool auton_auto_clamp_arm(){
  using namespace claw_distance;
  Lock lock;if(!lock.held)return false;
  reconcile();
  const bool ok=detector.arm(pros::millis(),config(true),allowed(true));
  autonomous_armed=ok;reported_state=detector.state;reason=ok?"WAITING_CONFIRMED_REMOVAL":"ARM_REJECTED";
  return ok;
}
void auton_auto_clamp_disarm(){claw_distance::cancel();}
void auton_auto_clamp_update(){claw_distance::update();}
bool auton_auto_clamp_triggered(){return claw_distance::reported_state.load()==auto_clamp::State::Clamped;}
auto_clamp::State auton_auto_clamp_state(){return claw_distance::reported_state.load();}
const char* auton_auto_clamp_reason(){return claw_distance::reason.load();}
