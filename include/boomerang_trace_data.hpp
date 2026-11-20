#pragma once
#include <cmath>
namespace boomerang_trace {
inline constexpr unsigned sample_ms=30, capture_ms=12000, tail_ms=1000, capacity=450;
inline constexpr unsigned unknown=0xffffffffu;
inline constexpr double valid(double x) { return std::isfinite(x) && x!=2147483647.0 ? x : NAN; }
inline constexpr double distance(double x,double y,double tx,double ty) {
  x=valid(x);y=valid(y);tx=valid(tx);ty=valid(ty);
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(tx)||!std::isfinite(ty))return NAN;
  return __builtin_hypot(tx-x,ty-y);
}
inline constexpr double heading_error(double target,double current) {
  target=valid(target);current=valid(current);
  if(!std::isfinite(target)||!std::isfinite(current))return NAN;
  return __builtin_remainder(target-current,360.0);
}
enum EndFlag : unsigned { EndCancelled=1, EndTimeout=2, EndInterrupted=4, EndContended=8 };
// End flags survive a failed try-lock; finalize later under the recorder mutex.
struct Window {
  bool armed=false, active=false, ready=false;
  unsigned id=0,count=0,dropped=0;
  constexpr bool arm(unsigned test) { if(active||ready) return false; armed=true; id=test; return true; }
  constexpr bool begin() { if(!armed||active||ready) return false; armed=false; active=true; count=dropped=0; return true; }
  constexpr bool append() { if(!active) return false; if(count>=capacity){++dropped;return false;} ++count;return true; }
  constexpr void stop() { armed=false; if(active)ready=true;active=false; }
  constexpr void finalize(unsigned flags) { if(flags)stop(); }
};
constexpr double absolute(double x) { return x<0 ? -x : x; }
// Observation thresholds only, never applied to the robot/controller.
struct Signal {
  bool have=false;
  double previous=0,peak_step=0;
  int sign=0,delta_sign=0;
  unsigned reversals=0,oscillations=0,comparisons=0;
  constexpr void add(double x,double deadband) {
    if(!std::isfinite(x)) { have=false;sign=delta_sign=0;return; }
    const int s=x>deadband?1:x<-deadband?-1:0;
    if(s&&sign&&s!=sign) ++reversals;
    if(s)sign=s;
    if(have) {
      ++comparisons;
      const double d=x-previous;
      if(absolute(d)>peak_step)peak_step=absolute(d);
      const int ds=d>deadband?1:d<-deadband?-1:0;
      if(ds&&delta_sign&&ds!=delta_sign)++oscillations;
      if(ds)delta_sign=ds;
    }
    previous=x;have=true;
  }
};
struct Rest {
  unsigned since=unknown,observed=unknown,last=unknown;
  constexpr void sample(unsigned ms,bool all_still) {
    if(observed!=unknown)return;
    if(last!=unknown && ms-last>3*sample_ms)since=unknown;
    last=ms;
    if(!all_still){since=unknown;return;}
    if(since==unknown)since=ms;
    if(ms-since>=100)observed=ms;
  }
};
struct TimestampOrder {
  unsigned previous=0,equal=0,backwards=0,first_bad=unknown,first_equal=unknown,bad_previous=0,bad_current=0;
  bool have=false;
  constexpr void add(unsigned ms,unsigned index) {
    if(have) {
      if(ms==previous){++equal;if(first_equal==unknown)first_equal=index;}
      if(ms<previous){++backwards;if(first_bad==unknown){first_bad=index;bad_previous=previous;bad_current=ms;}}
    }
    previous=ms;have=true;
  }
  constexpr bool ordered() const {return backwards==0;}
};}