#include "auton_auto_clamp_state.hpp"
#include "claw_distance.hpp"
using namespace auto_clamp;
constexpr Config c{20,60,3,20,60,1000,3}; // Synthetic fixtures, NOT robot calibration.
constexpr bool tests(){
 Detector d;
 if(d.arm(0,c,false))return false;
 if(!d.arm(0,c,true))return false;
 // Present at arming cannot close, including exact range boundaries.
 for(unsigned t=20;t<=80;t+=20)if(d.sample(t,c,true,20))return false;
 d.sample(100,c,true,80);d.sample(120,c,false,9999);d.sample(140,c,true,80);
 if(d.removal_confirmed)return false; // Invalid reading interrupts removal.
 d.sample(160,c,true,80);d.sample(180,c,true,80);
 if(!d.removal_confirmed)return false;
 if(d.sample(200,c,true,20)||d.sample(220,c,true,60)||!d.sample(240,c,true,40))return false;
 d.finish(true);
 if(d.state!=State::Clamped || d.sample(260,c,true,40))return false;
 // Explicit/manual rearm while object remains present must wait for removal again.
 if(!d.arm(300,c,true))return false;
 for(unsigned t=320;t<=380;t+=20)if(d.sample(t,c,true,40))return false;
 d.sample(400,c,true,80);d.sample(420,c,true,80);d.sample(440,c,true,80);
 if(d.sample(520,c,true,40)||d.removal_confirmed)return false; // Stale gap revokes removal.
 d.sample(540,c,true,80);d.sample(560,c,true,80);d.sample(580,c,true,80);
 d.sample(600,c,true,40);d.sample(620,c,true,40);
 if(!d.sample(640,c,true,40))return false;
 d.finish(false);if(d.state!=State::Fault||d.arm(660,c,true))return false;
 Detector disabled;disabled.arm(0,c,true);disabled.check(20,c,false);
 if(disabled.state!=State::Idle||disabled.sample(40,c,true,40))return false;
 Detector timeout;timeout.arm(0,c,true);if(timeout.check(1000,c,true)||timeout.state!=State::Timeout)return false;
 Detector wrap;wrap.arm(0xfffffff0u,c,true);
 wrap.sample(4,c,true,80);wrap.sample(24,c,true,80);wrap.sample(44,c,true,80);
 wrap.sample(64,c,true,40);wrap.sample(84,c,true,40);if(!wrap.sample(104,c,true,40))return false;
 return true;
}
static_assert(tests(),"range/removal, startup presence, manual rearm, stale, one-shot, fault, disable, timeout, wrap");
static_assert(!Config{0,0,0,0,0,0}.valid());
static_assert(!Config{20,60,1,20,60,300}.valid());
static_assert(!Config{20,60,3,20,10,300}.valid());
static_assert(!Config{20,60,3,20,60,0}.valid());
using claw_distance::Status;using claw_distance::classify;
static_assert(classify(false,false,0,0,0,0,20)==Status::Unconfigured);
static_assert(classify(true,false,100,0,0,0,20)==Status::Disconnected);
static_assert(classify(true,true,INT32_MAX,0,0,0,20)==Status::Error);
static_assert(classify(true,true,100,5,0,0,20)==Status::Error);
static_assert(classify(true,true,9999,0,0,0,20)==Status::NoObject);
static_assert(classify(true,true,0,0,0,0,20)==Status::Invalid);
static_assert(classify(true,true,-1,0,0,0,20)==Status::Invalid);
static_assert(classify(true,true,60,0,INT32_MAX,5,20)==Status::Valid);
static_assert(classify(true,true,201,0,19,0,20)==Status::LowConfidence);
static_assert(classify(true,true,201,0,20,0,20)==Status::Valid);
static_assert(classify(true,true,201,0,20,5,20)==Status::Error);
int main(){return tests()?0:1;}
