#include <cassert>
#include "pinnacle_auton.hpp"
#include "mock.hpp"
#include "pulse.inc"
#include "ACTUAL_COORDINATOR"
unsigned scenarios=0;
void reset(){
 ++scenarios;
 test::now=test::cancel_at=test::fail_sensor_at=0;test::nav_duration=600;
 test::cancel=false;test::ready=true;test::lift_active=test::settled=false;test::stop_ok=test::rest_ok=true;
 test::wrong_heading=test::wrong_pose=test::native_bad=test::nav_fail=test::reached_early=false;
 test::connected=test::toggle_start_ok=test::toggle_stop_ok=test::claw_ok=test::closed=true;
 test::geometry_active=test::sequence_active=false;test::position=test::target=0;
 test::lift_start=test::lift_budget=test::in_tolerance=test::lift_stop_calls=test::updates=test::nonzero_updates=0;
 test::nav_calls=test::open_calls=test::close_calls=test::toggle_starts=test::toggle_stops=0;
 test::close_at=test::toggle_at=0;test::mode=0;test::reason="NONE";chassis.interfered=false;pose={};events.clear();
 lift_calibration::hold_validated=true;pinnacle_config::claw_toggle_overlap_verified=true;
}
bool combined(PinnacleArrival* a=nullptr,unsigned lt=1500){return auton_boomerang_with_lift(1,2,270,70,100,lt,4000,a);}
int main(){
 reset();lift_calibration::hold_validated=false;assert(!combined());assert(test::nav_calls==0&&test::nonzero_updates==0);
 reset();assert(combined());assert(test::reached_early&&!test::lift_active&&test::nav_calls==1);
 reset();test::nav_duration=100;assert(combined());assert(test::now>=400); // Lift finishes after navigation, same deadline.
 reset();assert(!combined(nullptr,250));assert(!test::lift_active&&test::now<600);
 reset();test::wrong_pose=true;assert(!combined());assert(test::open_calls==0);
 reset();test::native_bad=true;assert(!combined());assert(test::open_calls==0);
 reset();test::rest_ok=false;assert(!combined());
 reset();test::cancel_at=100;assert(!combined());assert(!test::lift_active);
 reset();test::fail_sensor_at=100;assert(!combined());assert(!test::lift_active);
 reset();test::stop_ok=false;assert(!combined());assert(test::nav_calls==0);
 reset();lift_auton_target_start(100,250);test::now=240;assert(!auton_wait_for_lift(100,1500));assert(test::now<=260);
 reset();lift_auton_target_start(100,1500);test::cancel=true;assert(!auton_wait_for_lift(100,1500));assert(test::nonzero_updates==0);
 reset();lift_auton_target_start(100,1500);test::position=100;test::settled=true;test::now=400;test::in_tolerance=100;assert(auton_wait_for_lift(100,1500));assert(test::now==400);
 reset();test::position=100;assert(auton_lift_to_intake(1500));assert(std::fabs(test::position-38.5)<=2);
 reset();assert(auton_lift_to_intake(1500));assert(std::fabs(test::position-38.5)<=2);
 reset();pinnacle_config::claw_toggle_overlap_verified=false;assert(!auton_claw_close_with_toggle());assert(test::close_calls==0&&test::toggle_starts==0);
 reset();assert(auton_claw_close_with_toggle());assert(test::now==395&&test::close_at==test::toggle_at&&test::toggle_stops==1);
 reset();assert(auton_claw_close_with_toggle(123));assert(test::now==123);
 reset();test::cancel_at=50;assert(!auton_claw_close_with_toggle());assert(test::now==50&&test::toggle_stops>=1);
 reset();test::claw_ok=false;assert(!auton_claw_close_with_toggle());assert(test::toggle_starts==0&&test::toggle_stops>=1);
 reset();test::connected=false;assert(!auton_claw_close_with_toggle());assert(test::toggle_starts==0);
 reset();test::toggle_start_ok=false;assert(!auton_claw_close_with_toggle());assert(test::toggle_stops>=1);
 reset();test::toggle_stop_ok=false;assert(!auton_claw_close_with_toggle());
 reset();PinnacleArrival missing;assert(!auton_score_at_goal(missing,50,1500,true,true));assert(test::open_calls==0);
 reset();PinnacleArrival arrival;assert(combined(&arrival));assert(auton_score_at_goal(arrival,50,1500,true,true));assert(test::open_calls==1);
 assert(!auton_score_at_goal(arrival,50,1500,true,true));assert(test::open_calls==1);
 reset();PinnacleArrival no_clearance;assert(combined(&no_clearance));assert(!auton_score_at_goal(no_clearance,50,1500,false,true));assert(test::open_calls==0);
 reset();PinnacleArrival drift;assert(combined(&drift));pose.x+=1;assert(!auton_score_at_goal(drift,50,1500,true,true));assert(test::open_calls==0);
 reset();PinnacleArrival timeout;assert(combined(&timeout));assert(!auton_score_at_goal(timeout,50,10,true,true));assert(test::open_calls==0);
 reset();assert(!auton_acquire_from_loader({}));assert(test::nav_calls==0&&test::open_calls==0);
 reset();assert(!auton_run_pinnacle_route({}));assert(test::nav_calls==0);
 reset();test::position=200;assert(combined());assert(test::position==100); // Concurrent descent.
 reset();test::wrong_heading=true;assert(!combined());
 reset();test::nav_duration=5100;assert(!combined());assert(!test::lift_active);
 reset();assert(!auton_boomerang_with_lift(1,2,270,70,100,1000,0));assert(test::nav_calls==0);
 reset();assert(!auton_boomerang_with_lift(NAN,2,270,70,100,1000,4000));assert(test::nav_calls==0);
 reset();assert(!auton_boomerang_with_lift(1,2,270,70,3001,1000,4000));assert(test::nav_calls==0);
 reset();test::mode=1;assert(!combined());assert(test::nav_calls==0);
 reset();assert(!auton_claw_close_with_toggle(0));assert(test::toggle_starts==0);
 reset();PinnacleArrival unsecured;assert(combined(&unsecured));assert(!auton_score_at_goal(unsecured,50,1500,true,false));assert(test::open_calls==0);
 std::printf("PASS: %u coordinator/pulse scenarios (actual source, mocked hardware/navigation).\n",scenarios);
}
