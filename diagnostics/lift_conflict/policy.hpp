#pragma once
#include "config.hpp"
#include <cmath>
#include <cstdint>
namespace lift_diag {
struct Input {
  unsigned now=0, generation=0;
  bool permitted=false, up=false, down=false, connected=true, valid=true;
  bool approved=false, reference=false, buffer_ok=true;
  double p16=0, minimum=0, maximum=0, stopping_margin=NAN;
  double rpm[3]{}, current[3]{}, temperature[3]{};
};
struct Policy {
  bool running=false, armed=false, fault=false, sampled=false;
  unsigned generation=0, last=0, began=0, stall_since=0;
  bool stalling=false;
  int direction=0;
  double start=0;
  const char* reason="WAIT_RELEASE";
  void stop(const char* why,bool hard=false) {
    running=false; armed=false; stalling=false; reason=why; fault=fault||hard;
  }
  int step(const Input& i) {
    const unsigned gap=i.now-last; last=i.now;
    if (generation!=i.generation) {generation=i.generation;stop("MODE_CHANGE");}
    if (sampled && running && gap>MAX_CONTROL_GAP_MS) stop("CONTROL_GAP",true);
    sampled=true;
    if (fault) return 0;
    if (!i.permitted) {stop("MODE_OR_CONTROLLER");return 0;}
    if (!i.connected) {stop("MOTOR_DISCONNECTED",true);return 0;}
    if (!i.valid || !std::isfinite(i.p16)) {stop("INVALID_FEEDBACK",true);return 0;}
    for(int n=0;n<3;++n) {
      if (!std::isfinite(i.rpm[n]) || !std::isfinite(i.current[n]) || !std::isfinite(i.temperature[n])) {stop("INVALID_FEEDBACK",true);return 0;}
      if (i.current[n]>MAX_CURRENT_MA) {stop("CURRENT_CUTOFF");return 0;}
      if (i.temperature[n]>=MAX_TEMPERATURE_C) {stop("TEMPERATURE_CUTOFF");return 0;}
      if (std::fabs(i.rpm[n])>MAX_RPM) {stop("UNEXPECTED_SPEED");return 0;}
    }
    if (!i.approved) {stop("POWER_DISABLED");return 0;}
    if (!i.reference || !std::isfinite(i.minimum) || !std::isfinite(i.maximum) || i.maximum<=i.minimum) {stop("CLEARANCE_UNCONFIRMED");return 0;}
    if (!std::isfinite(i.stopping_margin) || i.stopping_margin<=0 ||
        2*i.stopping_margin>=i.maximum-i.minimum) {stop("INVALID_MARGIN_CONFIG");return 0;}
    if (!i.buffer_ok) {stop("LOGGER_BACKPRESSURE");return 0;}
    if (i.p16<i.minimum || i.p16>i.maximum) {stop("SOFTWARE_ENVELOPE_EXCEEDED",true);return 0;}
    if (!i.up && !i.down) {
      if (running) stop("RELEASED");
      bool resting=true;for(int n=0;n<3;++n)resting=resting&&std::fabs(i.rpm[n])<STALL_RPM;
      armed=resting; return 0;
    }
    if (i.up && i.down) {stop("BOTH_BUTTONS");return 0;}
    const int requested=i.up?1:-1;
    if (running && requested!=direction) {stop("DIRECTION_CHANGE_RELEASE_REQUIRED");return 0;}
    if (!running) {
      if (!armed) return 0;
      running=true;armed=false;direction=requested;began=i.now;start=i.p16;stalling=false;
    }
    if (i.now-began>=MAX_ACTIVATION_MS) {stop("RUNTIME_CUTOFF");return 0;}
    if ((direction>0 && i.p16>=i.maximum-i.stopping_margin) || (direction<0 && i.p16<=i.minimum+i.stopping_margin)) {stop("TRAVEL_LIMIT");return 0;}
    const double delta=i.p16-start;
    if (delta*direction < -WRONG_DIRECTION_DEG || std::fabs(delta)>MAX_DISPLACEMENT_DEG) {stop("UNEXPECTED_P16_MOTION");return 0;}
    bool slow=false;for(int n=0;n<3;++n)slow=slow||std::fabs(i.rpm[n])<STALL_RPM;
    if(i.now-began>=STALL_GRACE_MS && slow) {
      if(!stalling){stalling=true;stall_since=i.now;}
      if(i.now-stall_since>=STALL_DURATION_MS){stop("STALL_OR_INSUFFICIENT_MOTION");return 0;}
    } else stalling=false;
    reason="RUNNING";return direction*LIFT_DIAGNOSTIC_MAX_MV;
  }
};
}
