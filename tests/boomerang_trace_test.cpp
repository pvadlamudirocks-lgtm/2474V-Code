#include "boomerang_trace_data.hpp"
using namespace boomerang_trace;
constexpr bool lifecycle() {
  Window w;
  if(w.active||w.armed||w.ready||w.begin()||w.append())return false;
  if(!w.arm(7)||!w.begin()||w.arm(8))return false;
  for(unsigned i=0;i<capacity;++i)if(!w.append())return false;
  if(w.append()||w.count!=capacity||w.dropped!=1)return false;
  w.stop();
  if(w.active||!w.ready||w.append()||w.arm(8))return false;
  w.ready=false;
  return w.arm(8)&&w.begin()&&w.count==0&&w.dropped==0;
}
static_assert(lifecycle());
static_assert(distance(0,0,3,4)==5);
static_assert(heading_error(1,359)==2 && heading_error(359,1)==-2);
static_assert(!std::isfinite(valid(INFINITY)) && !std::isfinite(valid(2147483647.0)));
static_assert(!std::isfinite(distance(NAN,0,3,4)));
static_assert([] { Signal s; s.add(-20,1);s.add(0,1);s.add(20,1);s.add(10,1);
 return s.reversals==1&&s.oscillations==1&&s.peak_step==20; }());
static_assert([] { Signal s;s.add(20,1);s.add(NAN,1);s.add(-20,1);return s.reversals==0; }());
static_assert([] { Rest r;r.sample(0,true);r.sample(30,true);r.sample(60,true);r.sample(90,true);r.sample(120,true);
 return r.observed==120; }());
static_assert([] { Rest r;r.sample(0,true);r.sample(120,true);r.sample(150,false);return r.observed==unknown; }());
// Failed try-lock cannot consume cancellation/timeout: apply flags on next lock.
static_assert([] { Window w;w.arm(1);w.begin();w.append();
 const unsigned pending=EndCancelled|EndContended;
 if(!w.active||w.ready)return false;
 w.finalize(pending);
 return !w.active&&w.ready&&!w.append()&&!w.arm(2); }());
static_assert([] { Window w;w.arm(1);w.begin();w.finalize(EndTimeout);
 w.ready=false;return w.arm(2)&&w.begin(); }());
static_assert([] { Window w;w.arm(1);w.begin();w.finalize(EndInterrupted);
 return w.ready&&!w.active; }());
// Equal millisecond timestamps are valid; row index establishes sample order.
static_assert([] { TimestampOrder t;t.add(0,0);t.add(0,1);t.add(30,2);
 return t.ordered()&&t.equal==1&&t.first_bad==unknown; }());
static_assert([] { TimestampOrder t;t.add(30,0);t.add(20,1);
 return !t.ordered()&&t.backwards==1&&t.first_bad==1; }());
