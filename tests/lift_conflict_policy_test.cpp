#include "../diagnostics/lift_conflict/policy.hpp"
#include <cassert>
#include <cstring>
#include <cstdio>
using namespace lift_diag;
struct Fake {
 Policy p;Input i;int volts[3]{};bool fail=false,zero_ack=true;unsigned writes=0;
 Fake(){i.permitted=i.approved=i.reference=true;i.p16=100;i.minimum=2;i.maximum=3000;i.stopping_margin=0.5;i.generation=2;for(int n=0;n<3;++n){i.rpm[n]=10;i.current[n]=100;i.temperature[n]=25;}}
 int tick(){i.now+=20;int mv=p.step(i);++writes;if(fail&&mv){p.stop("MOTOR_WRITE_FAULT",true);mv=0;}volts[0]=-mv;volts[1]=mv;volts[2]=-mv;zero_ack=mv==0&&!fail;return mv;}
 void release(){i.up=i.down=false;for(int n=0;n<3;++n)i.rpm[n]=0;assert(tick()==0);for(int n=0;n<3;++n)i.rpm[n]=10;}
 void start(bool up=true){release();i.up=up;i.down=!up;assert(tick()==(up?1000:-1000));}
 void reason(const char* r){assert(!strcmp(p.reason,r));}
};
int main(){
 {Fake f;f.i.up=true;assert(f.tick()==0);f.start();f.release();assert(f.zero_ack);f.reason("RELEASED");}
 {Fake f;f.start(false);f.release();assert(f.zero_ack);}
 {Fake f;f.start();f.i.down=true;assert(f.tick()==0);f.reason("BOTH_BUTTONS");f.i.down=false;assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 {Fake f;f.start();for(int n=0;n<50;++n)f.tick();f.reason("RUNTIME_CUTOFF");assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 {Fake f;f.start();f.i.current[0]=751;assert(f.tick()==0);f.reason("CURRENT_CUTOFF");f.i.current[0]=100;assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 {Fake f;f.start();f.i.rpm[0]=0;for(int n=0;n<25;++n)f.tick();f.reason("STALL_OR_INSUFFICIENT_MOTION");assert(!f.p.running);}
 {Fake f;f.start();f.i.valid=false;assert(f.tick()==0);assert(f.p.fault);f.i.valid=true;f.release();f.i.up=true;assert(f.tick()==0);}
 {Fake f;f.start();f.i.connected=false;assert(f.tick()==0);f.reason("MOTOR_DISCONNECTED");}
 {Fake f;f.start();f.fail=true;assert(f.tick()==0);f.reason("MOTOR_WRITE_FAULT");assert(!f.zero_ack&&f.p.fault);}
 {Fake f;f.start();f.i.generation+=2;assert(f.tick()==0);assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 {Fake f;f.start();f.i.permitted=false;assert(f.tick()==0);f.i.permitted=true;assert(f.tick()==0);}
 {Fake f;f.start();f.i.now+=101;assert(f.tick()==0);f.reason("CONTROL_GAP");}
 {Fake f;f.start();f.i.buffer_ok=false;assert(f.tick()==0);f.reason("LOGGER_BACKPRESSURE");assert(f.zero_ack);}
 {Fake f;f.start();f.i.p16=99;assert(f.tick()==0);f.reason("UNEXPECTED_P16_MOTION");}
 {Fake f;f.i.approved=false;f.release();f.i.up=true;assert(f.tick()==0);f.reason("POWER_DISABLED");}
 {Fake f;f.i.reference=false;f.release();f.i.up=true;assert(f.tick()==0);f.reason("CLEARANCE_UNCONFIRMED");}
 {Fake f;f.start();f.i.p16=NAN;assert(f.tick()==0);f.reason("INVALID_FEEDBACK");}
 {Fake f;f.start();f.i.p16=3001;assert(f.tick()==0);f.reason("SOFTWARE_ENVELOPE_EXCEEDED");}
 {Fake f;f.i.stopping_margin=NAN;f.release();f.i.up=true;assert(f.tick()==0);f.reason("INVALID_MARGIN_CONFIG");}
 {Fake f;f.i.minimum=-5;f.i.maximum=5;f.i.p16=0;f.start();f.i.p16=4.6;assert(f.tick()==0);f.reason("TRAVEL_LIMIT");f.release();f.i.up=true;assert(f.tick()==0);f.i.up=false;f.release();f.i.down=true;assert(f.tick()==-1000);}
 // Approval while held cannot start; release is still required.
 {Fake f;f.i.minimum=-RELATIVE_ENVELOPE_DEG;f.i.maximum=RELATIVE_ENVELOPE_DEG;f.i.stopping_margin=STOPPING_MARGIN_DEG;f.i.p16=0;f.i.reference=false;f.release();f.i.up=true;assert(f.tick()==0);f.reason("CLEARANCE_UNCONFIRMED");f.i.reference=true;assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);f.release();assert(f.zero_ack);}
 // Repeated activations do not renew the startup envelope.
 {Fake f;f.i.minimum=-5;f.i.maximum=5;f.i.stopping_margin=STOPPING_MARGIN_DEG;f.i.p16=0;f.start();f.i.p16=2;assert(f.tick()==1000);f.release();f.i.up=true;assert(f.tick()==1000);f.i.p16=3;assert(f.tick()==0);f.reason("TRAVEL_LIMIT");assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==0);f.release();f.i.down=true;assert(f.tick()==-1000);}
 {Fake f;f.i.minimum=-5;f.i.maximum=5;f.i.stopping_margin=STOPPING_MARGIN_DEG;f.i.p16=0;f.start(false);f.i.p16=-2.99;assert(f.tick()==-1000);f.i.p16=-3;assert(f.tick()==0);f.reason("TRAVEL_LIMIT");f.i.down=false;f.i.up=true;assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 {Fake f;f.start();f.i.down=true;assert(f.tick()==0&&f.zero_ack);f.i.down=false;assert(f.tick()==0);f.release();f.i.down=true;assert(f.tick()==-1000);f.release();assert(f.zero_ack);}
 {Fake f;f.start();f.i.reference=false;assert(f.tick()==0&&f.zero_ack);f.reason("CLEARANCE_UNCONFIRMED");f.i.reference=true;assert(f.tick()==0);f.release();f.i.up=true;assert(f.tick()==1000);}
 std::puts("PASS 25 isolated lift policy scenarios; fake adapter, no RTOS/hardware timing claim");
}
