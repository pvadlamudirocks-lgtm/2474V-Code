#include "EZ-Template/slew.hpp"
#include "EZ-Template/motion_output_safety.hpp"
#include <cassert>
#include <climits>
#include <cstdio>
#include <random>
#include <vector>
using namespace ez;
static output_safety::Latch latch;
void ez::motion_output_fault(output_safety::Fault f) { latch.set(f); }
output_safety::Fault ez::motion_output_fault_get() { return latch.get(); }
static void near(double a,double b) { assert(std::abs(a-b)<1e-9); }
int main() {
 struct Pose { double x,y,theta; }; struct Movement { Pose target; int drive_direction,max_xy_speed,turn_behavior; };
 std::vector<Movement> path;
 assert(!output_safety::path_valid(path));
 path.push_back({{0,0,0},0,70,3}); assert(output_safety::path_valid(path)); assert(!output_safety::path_valid(path,2));
 path.push_back({{19.94,6.93,56.83},0,70,3}); assert(output_safety::path_valid(path,2));
 path[1].max_xy_speed=128; assert(!output_safety::path_valid(path)); path[1].max_xy_speed=70;
 path[1].drive_direction=2; assert(!output_safety::path_valid(path)); path[1].drive_direction=0;
 path[1].target.theta=INFINITY; assert(!output_safety::path_valid(path)); path[1].target.theta=56.83;
 path[1].target.x=NAN; assert(!output_safety::path_valid(path));
 const double nan=std::numeric_limits<double>::quiet_NaN(), inf=std::numeric_limits<double>::infinity();
 slew s(10,55);
 s.initialize(true,70,100,0); near(s.iterate(0),55); near(s.iterate(5),62.5); near(s.iterate(10),62.5); near(s.iterate(11),70);
 s.initialize(true,70,-100,0); near(s.iterate(-5),62.5);
 s.constants_set(10,70); s.initialize(true,70,100,0); near(s.iterate(0),70);
 s.initialize(false,70,0,0); near(s.iterate(0),70); assert(motion_output_fault_get()==output_safety::Fault::None);
 s.initialize(true,70,0,0); assert(s.output()==0); assert(motion_output_fault_get()==output_safety::Fault::Slew);
 s.speed_max_set(127); assert(s.iterate(1)==0); // sticky invalid ramp, never full-speed fallback
 latch.reset(); s.constants_set(10,55); s.initialize(true,70,100,0); s.iterate(nan); assert(s.output()==0); assert(s.iterate(0)==0);
 for(double bad:{nan,inf,-inf}) {
  for(int field=0;field<5;++field) {
   latch.reset(); s.constants_set(10,55);
   double maximum=70,target=100,current=0;
   if(field==0)maximum=bad; if(field==1)target=bad; if(field==2)current=bad;
   if(field==3)s.constants.min_speed=bad; if(field==4)s.constants.distance_to_travel=bad;
   s.initialize(true,maximum,target,current); assert(s.output()==0); assert(s.iterate(0)==0);
   assert(motion_output_fault_get()!=output_safety::Fault::None);
  }
  assert(!output_safety::mix(bad,1,1,true,70,70,70).valid);
  assert(!output_safety::mix(1,bad,1,true,70,70,70).valid);
  assert(!output_safety::mix(1,1,bad,true,70,70,70).valid);
  assert(!output_safety::mix(1,1,1,true,bad,70,70).valid);
  assert(!output_safety::mix(1,1,1,true,70,bad,70).valid);
  assert(!output_safety::mix(1,1,1,true,70,70,bad).valid);
 }
 // Finite inputs whose arithmetic overflows or loses the ramp denominator also fail closed.
 latch.reset(); s.constants_set(10,55); s.initialize(true,70,1e308,-1e308); assert(s.iterate(0)==0);
 latch.reset(); s.constants_set(1e-320,55); s.initialize(true,70,100,0); assert(s.iterate(0)==0);
 latch.reset(); s.constants_set(10,55); s.initialize(true,70,1e308,1e308-1e292); assert(s.iterate(0)==0);
 assert(!output_safety::mix(70,10,1e308,true,70,70,70).valid);
 // Zero-speed, valid disabled slew is a zero-output request, not a full-speed fallback.
 latch.reset(); s.constants_set(10,55); s.initialize(false,0,0,0); assert(s.iterate(0)==0); assert(motion_output_fault_get()==output_safety::Fault::None);
 latch.reset(); s.constants_set(0,55); s.initialize(true,70,100,0); assert(s.iterate(0)==0);
 latch.reset(); s.constants_set(10,55); s.initialize(false,70,100,0); near(s.iterate(3),70);
 assert(output_safety::motor_boundary(INT_MIN)==-127); assert(output_safety::motor_boundary(INT_MAX)==127);
 for(int i=-127;i<=127;++i)assert(output_safety::motor_boundary(i)==i);
 for(double cap:{0.,30.,55.,70.,100.,127.}) {
  const auto m=output_safety::mix(70,487.584,1,true,cap,cap,cap);
  assert(m.valid); assert(std::abs(m.left)<=cap && std::abs(m.right)<=cap);
 }
 auto m=output_safety::mix(70,487.584,1,true,100,100,30); assert(m.valid); assert(std::abs(m.left)<=30&&std::abs(m.right)<=30);
 assert(!output_safety::mix(1,1,1,true,128,70,70).valid);
 // Same finite native calculations for 100,000 normal input combinations.
 std::mt19937 rng(2474); std::uniform_real_distribution<double> out(-1000,1000),bias(-1,1),caps(1,127);
 for(int i=0;i<100000;++i) {
  double x=out(rng),a=out(rng),b=bias(rng),cap=caps(rng);
  const auto got=output_safety::mix(x,a,b,true,cap,cap,cap); assert(got.valid);
  x=std::max(-cap,std::min(cap,x))*b;
  double faster=std::max(std::abs(x),std::abs(a)); if(faster>cap){x*=cap/faster;a*=cap/faster;}
  double l=x+a,r=x-a; faster=std::max(std::abs(l),std::abs(r)); if(faster>cap){l*=cap/faster;r*=cap/faster;}
  near(got.left,l);near(got.right,r);assert(std::abs(got.left)<=cap&&std::abs(got.right)<=cap);
 }
 latch.reset(); motion_output_fault(output_safety::Fault::Path); motion_output_fault(output_safety::Fault::Output);
 assert(motion_output_fault_get()==output_safety::Fault::Path);
 for(int i=0;i<1000;++i)assert(motion_output_fault_get()!=output_safety::Fault::None);
 latch.reset();assert(motion_output_fault_get()==output_safety::Fault::None);
 std::puts("PASS: actual patched slew + production mixer/latch/boundary; 100000 finite regressions; invalid/zero/speed/fault tests");
}
