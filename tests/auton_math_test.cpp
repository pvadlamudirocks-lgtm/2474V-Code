#include "auton_math.hpp"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace auton_math;
bool near(double a,double b) { return std::fabs(a-b)<1e-8; }
int main() {
  // Synthetic geometry only; none of these are robot calibration data.
  Sensor s{true,9,2,3,0,1,100};
  Wall w{true,Axis::Y,40,1,10,5};
  auto c=correct({0,7,0},s,w,30);
  assert(c.gate==Gate::Accepted && near(c.coordinate,7) && near(c.predicted_range,30));
  c=correct({0,9,0},s,w,30); assert(near(c.residual,-2));
  c=correct({0,20,0},s,w,30); assert(c.gate==Gate::Residual);
  assert(correct({0,7,90},s,w,30).gate==Gate::Heading);
  assert(correct({0,7,0},s,w,NAN).gate==Gate::InvalidReading);
  assert(correct({0,7,0},s,w,101).gate==Gate::InvalidReading);
  assert(correct({0,7,0},{},w,30).gate==Gate::Unconfigured);
  w.axis=Axis::X; c=correct({7,0,90},s,w,30);
  assert(c.gate==Gate::Accepted && near(c.coordinate,7));
  w.facing=-1; w.coordinate=-40;
  c=correct({-7,0,270},s,w,30); assert(c.gate==Gate::Accepted && near(c.coordinate,-7));
  w.axis=Axis::Y; c=correct({0,-7,180},s,w,30); assert(c.gate==Gate::Accepted && near(c.coordinate,-7));
  w.coordinate=40; assert(correct({0,-7,180},s,w,30).gate==Gate::InvalidGeometry);
  assert(near(paired_heading_delta(10,12,2),45));
  assert(near(paired_heading_delta(12,10,-2),45));
  assert(std::isnan(paired_heading_delta(10,12,0)));
  Sensor a{true,9,-2,0,0,1,100}, b{true,11,2,0,0,1,100};
  Wall parallel{true,Axis::Y,30,1,20,10};
  assert(near(paired_heading_estimate({0,0,0},a,b,parallel,30,30),0));
  b.forward=1; assert(std::isnan(paired_heading_estimate({0,0,0},a,b,parallel,30,30)));
  auto p=mirror({3,7,90}); assert(p.x==-3 && p.y==7 && p.h==270);
  p=mirror(p); assert(p.x==3 && p.y==7 && p.h==90);
  assert(near(error(1,359),2)); assert(mirror({0,0,0}).h==0);
  assert(!within({0,0,0},{0,0,0},{}));
  for(int sign:{-1,1}) {
    LiftLimits l{true,sign,0,sign*10.0};
    assert(!lift_direction_allowed(l,0,-1));
    assert(lift_direction_allowed(l,sign*5.0,1));
    assert(!lift_direction_allowed(l,sign*10.0,1));
    assert(lift_direction_allowed(l,sign*10.0,-1));
  }
  assert(!lift_direction_allowed({},1,-1));
  std::puts("auton_math: all synthetic assertions passed");
}
