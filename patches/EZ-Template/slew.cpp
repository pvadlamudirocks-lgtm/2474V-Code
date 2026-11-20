/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/api.hpp"
#include "EZ-Template/motion_output_safety.hpp"

using namespace ez;

// Constructor
slew::slew() {}
slew::slew(double distance, int minimum_speed) {
  constants_set(distance, minimum_speed);
}

// Set constants
void slew::constants_set(double distance, int minimum_speed) {
  constants.min_speed = minimum_speed;
  constants.distance_to_travel = distance;
}

void slew::speed_max_set(double speed) { max_speed = speed; }
double slew::speed_max_get() { return max_speed; }

bool slew::enabled() { return is_enabled; }                  // Is slew currently enabled?
double slew::output() { return last_output; }                // Returns output
slew::Constants slew::constants_get() { return constants; }  // Get constants

// Preserve the native object layout. NaN slope is an internal sticky-invalid marker;
// public output is zero and the existing motor-owner emergency latch is requested.
void slew::initialize(bool enabled, double maximum_speed, double target, double current) {
  is_enabled=false; last_output=0; max_speed=maximum_speed;
  slope=std::numeric_limits<double>::quiet_NaN();
  if(!output_safety::speed_valid(maximum_speed) || !std::isfinite(target) || !std::isfinite(current) ||
     !output_safety::speed_valid(constants.min_speed) || !std::isfinite(constants.distance_to_travel) ||
     constants.distance_to_travel<0 || !std::isfinite(target-current)) {
    motion_output_fault(output_safety::Fault::Slew); return;
  }
  is_enabled=maximum_speed<constants.min_speed?false:enabled;
  sign=util::sgn(target-current);
  if(!is_enabled) { slope=0; last_output=max_speed; return; }
  // A zero-distance enabled ramp has no direction or distance parameterization.
  // Reject explicitly; never turn it into a full-speed heading command.
  if(sign==0 || constants.distance_to_travel<=0) {
    is_enabled=false; motion_output_fault(output_safety::Fault::Slew); return;
  }
  x_intercept=current+constants.distance_to_travel*sign;
  y_intercept=max_speed*sign;
  const double denominator=x_intercept-current;
  if(!std::isfinite(x_intercept) || !std::isfinite(denominator) || denominator==0) {
    is_enabled=false; motion_output_fault(output_safety::Fault::Slew); return;
  }
  slope=(sign*constants.min_speed-y_intercept)/denominator;
  if(!std::isfinite(slope)) { is_enabled=false; motion_output_fault(output_safety::Fault::Slew); return; }
  last_output=constants.min_speed;
}
double slew::iterate(double current) {
  if(!std::isfinite(slope) || !std::isfinite(current) || !output_safety::speed_valid(max_speed)) {
    is_enabled=false; slope=std::numeric_limits<double>::quiet_NaN(); last_output=0;
    motion_output_fault(output_safety::Fault::Slew); return 0;
  }
  if(is_enabled) {
    error=x_intercept-current;
    if(!std::isfinite(error)) { slope=std::numeric_limits<double>::quiet_NaN(); return iterate(current); }
    if(util::sgn(error)!=sign) is_enabled=false;
    else last_output=(slope*error+y_intercept)*sign;
  } else last_output=max_speed;
  if(!std::isfinite(last_output)) { slope=std::numeric_limits<double>::quiet_NaN(); return iterate(current); }
  last_output=std::max(0.0,std::min(max_speed,last_output));
  return last_output;
}
