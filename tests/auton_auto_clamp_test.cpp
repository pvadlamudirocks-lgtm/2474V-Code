#include "auton_auto_clamp_state.hpp"
using namespace auto_clamp;
constexpr Config c{20,60,3,20,60,300};
constexpr bool tests() {
  Detector d;
  if (d.arm(0,c,false) || d.sample(20,c,true,30)) return false;
  if (!d.arm(0,c,true) || d.sample(19,c,true,30)) return false;
  if (d.sample(20,c,true,30) || d.sample(40,c,true,30)) return false;
  if (!d.sample(60,c,true,30)) return false;
  d.finish(true);
  if (d.state != State::Clamped || d.sample(80,c,true,30)) return false;
  if (!d.arm(100,c,true)) return false;
  d.sample(120,c,true,30); d.sample(140,c,false,30);
  d.sample(160,c,true,30); d.sample(180,c,true,80);
  if (d.count != 0) return false;
  d.sample(200,c,true,30);
  if (d.sample(280,c,true,30) || d.count != 1) return false; // stale gap
  if (d.check(400,c,true) || d.state != State::Timeout) return false;
  if (!d.arm(500,c,true)) return false;
  d.sample(520,c,true,30); d.sample(540,c,true,30);
  if (d.check(550,c,false) || d.sample(560,c,true,30)) return false;
  if (!d.arm(600,c,true)) return false;
  d.disarm();
  if (d.sample(620,c,true,30) || !d.arm(700,c,true)) return false;
  d.sample(720,c,true,30); d.sample(740,c,true,30); d.sample(760,c,true,30);
  d.finish(false);
  if (d.state != State::Fault || d.arm(800,c,true)) return false; // no retry
  Detector wrap;
  if (!wrap.arm(0xfffffff0u,c,true)) return false;
  wrap.sample(4,c,true,30); wrap.sample(24,c,true,30);
  if (!wrap.sample(44,c,true,30)) return false;
  Detector deadline;
  deadline.arm(0,c,true);
  deadline.sample(260,c,true,30); deadline.sample(280,c,true,30);
  if (deadline.sample(300,c,true,30) || deadline.state != State::Timeout) return false;
  return true;
}
static_assert(tests(), "auto-clamp debounce, one-shot, cancellation, stale, timeout, failure, wrap");
static_assert(!Config{0,0,0,0,0,0}.valid());
static_assert(!Config{20,60,1,20,60,300}.valid());
static_assert(!Config{20,60,3,20,10,300}.valid());
static_assert(!Config{20,60,3,20,60,0}.valid());