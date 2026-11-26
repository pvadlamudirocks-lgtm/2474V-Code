const fs=require('fs'),cp=require('child_process'),os=require('os'),path=require('path'),assert=require('assert');
const root=process.cwd(),dir=fs.mkdtempSync(path.join(os.tmpdir(),'claw-distance-'));
const source=fs.readFileSync('src/claw_distance.cpp','utf8').replace('#include "subsystems.hpp"','').replace('#include "EZ-Template/navigation_hooks.hpp"','');
const subsystem=fs.readFileSync('src/subsystems.cpp','utf8');
assert(subsystem.includes('claw_distance::CommandGuard command;'));
assert(subsystem.includes('void update_object_detection() { claw_distance::update(); }'));
assert(!subsystem.includes('if (object_detected && !previous_detection'));
const code=String.raw`#include "auton_auto_clamp_state.hpp"
#include "claw_distance.hpp"
#include <cstdint>
#include <cerrno>
#undef NDEBUG
#include <cassert>
#include <cstdio>
constexpr int PROS_ERR=INT32_MAX;
namespace mechanism_config {
int CLAW_DISTANCE_PORT=0;bool ENABLE_CLAW_DISTANCE_DIAGNOSTIC=true;
unsigned CLAW_DISTANCE_DIAGNOSTIC_MS=100;
bool ENABLE_DRIVER_AUTO_CLAMP=false,CLAW_DISTANCE_CALIBRATED=false,CLAW_AUTO_CLAMP_POLARITY_VERIFIED=false;
int CLAW_DETECT_MIN_MM=0,CLAW_DETECT_MAX_MM=0,CLAW_DETECT_MIN_CONFIDENCE=20;
unsigned CLAW_DETECT_CONSISTENT_READINGS=3,CLAW_DISTANCE_SAMPLE_MS=20,CLAW_DISTANCE_FRESHNESS_MS=100,CLAW_AUTO_CLAMP_TIMEOUT_MS=0,CLAW_DETECT_REMOVAL_READINGS=3;
bool ENABLE_AUTON_AUTO_CLAMP=false,AUTON_AUTO_CLAMP_RANGE_CALIBRATED=false,AUTON_AUTO_CLAMP_POLARITY_VERIFIED=false;
int AUTON_AUTO_CLAMP_MIN_MM=0,AUTON_AUTO_CLAMP_MAX_MM=0;
unsigned AUTON_AUTO_CLAMP_READINGS=3,AUTON_AUTO_CLAMP_SAMPLE_MS=20,AUTON_AUTO_CLAMP_FRESHNESS_MS=100,AUTON_AUTO_CLAMP_TIMEOUT_MS=0;
}
namespace pros {
using task_t=void*;unsigned clock=100;unsigned millis(){return clock;}
namespace c {task_t task_get_current(){return reinterpret_cast<void*>(1);}}
namespace competition {bool disabled=false,autonomous=false;bool is_disabled(){return disabled;}bool is_autonomous(){return autonomous;}}
struct Mutex {bool held=false;bool take(unsigned){if(held)return false;held=true;return true;}void give(){held=false;}};
int constructed=0,mm=40,confidence=63,error=0;bool installed=true,cancel_on_read=false;
struct Distance {explicit Distance(int port){assert(port>0);++constructed;}bool is_installed(){return installed;}int get(){if(cancel_on_read)claw_distance::cancel();errno=error;return mm;}int get_confidence(){return confidence;}};
}
namespace ez {bool cancelled=false;bool autonomous_cancelled(){return cancelled;}}
bool available=true,closed=false,write_ok=true;int writes=0;
bool claw_pneumatic_available(){return available;}
bool claw_commanded_closed(){return closed;}
bool claw_close(){claw_distance::CommandGuard guard;if(!guard)return false;++writes;if(write_ok)closed=true;return write_ok;}
`+source+String.raw`
void tick(int mm,unsigned dt=20){pros::mm=mm;pros::clock+=dt;claw_distance::update();}
void removal(){tick(120);tick(120);tick(120);}
void entry(){tick(60);tick(100);tick(80);}
void manual_open(){claw_distance::CommandGuard guard;assert(guard);closed=false;}
int main(){
 using namespace mechanism_config;
 claw_distance::diagnostic_update();claw_distance::update();assert(pros::constructed==0&&writes==0);
 CLAW_DISTANCE_PORT=8;pros::clock+=100;claw_distance::diagnostic_update();assert(pros::constructed==1&&writes==0);
 ENABLE_DRIVER_AUTO_CLAMP=true;claw_distance::update();removal();entry();assert(writes==0);
 CLAW_DISTANCE_CALIBRATED=CLAW_AUTO_CLAMP_POLARITY_VERIFIED=true;CLAW_DETECT_MIN_MM=60;CLAW_DETECT_MAX_MM=100;CLAW_AUTO_CLAMP_TIMEOUT_MS=2000;
 claw_distance::cancel();tick(80);entry();entry();assert(writes==0);
 removal();entry();assert(writes==1&&closed);entry();assert(writes==1);
 manual_open();tick(80);entry();assert(writes==1&&!closed);
 tick(120);pros::installed=false;tick(120);pros::installed=true;tick(120);entry();assert(writes==1);
 tick(9999);entry();assert(writes==1);
 removal();entry();assert(writes==2);
 manual_open();tick(80);removal();tick(80,150);entry();assert(writes==2);
 removal();pros::cancel_on_read=true;entry();pros::cancel_on_read=false;assert(writes==2);
 pros::competition::disabled=true;tick(80);assert(writes==2);pros::competition::disabled=false;
 tick(80);removal();entry();assert(writes==3);
 manual_open();tick(80);pros::clock+=2000;tick(80);removal();entry();assert(writes==3);
 pros::competition::autonomous=true;tick(80);assert(!auton_auto_clamp_arm());
 ENABLE_AUTON_AUTO_CLAMP=AUTON_AUTO_CLAMP_RANGE_CALIBRATED=AUTON_AUTO_CLAMP_POLARITY_VERIFIED=true;
 AUTON_AUTO_CLAMP_MIN_MM=60;AUTON_AUTO_CLAMP_MAX_MM=100;AUTON_AUTO_CLAMP_TIMEOUT_MS=2000;
 assert(auton_auto_clamp_arm());entry();assert(writes==3);removal();ez::cancelled=true;entry();assert(writes==3);ez::cancelled=false;
 assert(auton_auto_clamp_arm());removal();entry();assert(writes==4&&auton_auto_clamp_triggered());
 auton_auto_clamp_disarm();assert(!auton_auto_clamp_triggered());
 manual_open();assert(auton_auto_clamp_arm());removal();write_ok=false;entry();assert(writes==5&&auton_auto_clamp_state()==auto_clamp::State::Fault);
 auton_auto_clamp_disarm();assert(!auton_auto_clamp_arm());
 puts("PASS real adapter: passive no-actuation/port-zero, gates, initial presence, manual override, removal, one-shot, invalid/disconnected/stale, cleanup, disabled/mode, timeout, autonomous cancellation, output failure");
}
`;
fs.writeFileSync(path.join(dir,'integration.cpp'),code);
for(const [src,name] of [[path.join(root,'tests/auton_auto_clamp_test.cpp'),'policy'],[path.join(dir,'integration.cpp'),'integration']]){
 const exe=path.join(dir,name+'.exe');cp.execFileSync(process.argv[2],['c++','-std=c++17','-O1','-UNDEBUG','-I'+path.join(root,'include'),src,'-o',exe],{stdio:'inherit'});cp.execFileSync(exe,[],{stdio:'inherit'});
}
