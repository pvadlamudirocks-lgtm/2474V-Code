#include "geometry_turn_deadline.hpp"
#include <limits>
using namespace geometry_turn;
constexpr bool blocked_wait() {
  Deadline d; d.arm(100,5000);
  // Only controller ticks run: no wait-task progress or completion callback.
  d.update(5099,false);
  if(d.must_stop())return false;
  d.update(5100,false);
  if(d.result!=Result::Timeout || !d.must_stop())return false;
  d.update(5200,false); d.finish(5300,false);
  return d.result==Result::Timeout && d.must_stop();
}
constexpr bool normal_completion() {
  Deadline d; d.arm(100,5000); d.finish(1000,false); d.update(6000,false);
  return d.result==Result::Returned && !d.must_stop();
}
constexpr bool cancellation() {
  Deadline d; d.arm(100,5000); d.update(110,true); d.update(120,false);
  d.finish(130,false);
  return d.result==Result::Interrupted && d.must_stop();
}
constexpr bool wraparound() {
  Deadline d; d.arm(std::numeric_limits<std::uint32_t>::max()-99,5000);
  d.update(4899,false); if(d.must_stop())return false;
  d.update(4900,false); return d.result==Result::Timeout;
}
constexpr bool finish_at_deadline() {
  Deadline d; d.arm(0,5000); d.finish(5000,false);
  return d.result==Result::Timeout;
}
constexpr bool explicit_rearm() {
  Deadline d; d.arm(0,5000); d.update(5000,false); d.arm(6000,5000);
  d.update(6001,false); return d.result==Result::Running && !d.must_stop();
}
static_assert(maximum_powered_ms==5000);
static_assert(blocked_wait());
static_assert(normal_completion());
static_assert(cancellation());
static_assert(wraparound());
static_assert(finish_at_deadline());
static_assert(explicit_rearm());
static_assert(!Deadline{}.must_stop());
