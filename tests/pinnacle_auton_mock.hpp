#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>
#include <algorithm>
namespace test {
inline unsigned now=0,cancel_at=0,fail_sensor_at=0,nav_duration=600;
inline bool cancel=false,ready=true,lift_active=false,settled=false,stop_ok=true,rest_ok=true;
inline bool wrong_heading=false,wrong_pose=false,native_bad=false,nav_fail=false,reached_early=false;
inline bool connected=true,toggle_start_ok=true,toggle_stop_ok=true,claw_ok=true,closed=true;
inline bool geometry_active=false,sequence_active=false;
inline double position=0,target=0;
inline unsigned lift_start=0,lift_budget=0,in_tolerance=0,lift_stop_calls=0,updates=0,nonzero_updates=0;
inline unsigned nav_calls=0,open_calls=0,close_calls=0,toggle_starts=0,toggle_stops=0;
inline unsigned close_at=0,toggle_at=0;
inline int mode=0;
inline const char* reason="NONE";
}
namespace pros {
inline unsigned millis(){return test::now;}
inline void delay(unsigned n){test::now+=n;if(test::cancel_at && test::now>=test::cancel_at)test::cancel=true;}
namespace c { inline void* task_get_current(){return reinterpret_cast<void*>(1);} }
}
namespace geometry_turn {inline constexpr unsigned maximum_powered_ms=5000;enum class Result{Idle,Running,Returned,Timeout,Interrupted,SensorFault};}
namespace ez {
inline constexpr int DISABLE=0,SMALL_EXIT=1,BIG_EXIT=2;
struct pose{double x=0,y=0,theta=0;};
struct Drive{bool interfered=false;bool odom_enabled(){return true;}bool pid_drive_toggle_get(){return true;}int drive_mode_get(){return test::mode;}};
struct NativeOdomWaitResult{bool completed=true;int xy=SMALL_EXIT,angular=SMALL_EXIT;};
inline geometry_turn::Result result=geometry_turn::Result::Idle;
inline bool autonomous_cancelled(){return test::cancel;}
inline void autonomous_emergency_stop(){test::cancel=true;}
inline void drive_stop(Drive*){test::mode=DISABLE;if(test::geometry_active){result=test::now>=5000?geometry_turn::Result::Timeout:test::cancel?geometry_turn::Result::Interrupted:geometry_turn::Result::Returned;test::geometry_active=false;}}
inline bool geometry_turn_deadline_active(Drive*){return test::geometry_active;}
inline bool geometry_turn_deadline_arm(Drive*,bool(*)(),int){test::geometry_active=true;result=geometry_turn::Result::Running;return true;}
inline geometry_turn::Result geometry_turn_deadline_result(){return result;}
inline NativeOdomWaitResult native_odom_wait_result(Drive*){return {true,test::native_bad?99:SMALL_EXIT,SMALL_EXIT};}
}
inline ez::Drive chassis;
namespace mechanism_config {inline constexpr double LIFT_MIN_DEG=0,LIFT_MAX_DEG=3000,LIFT_TARGET_TOLERANCE_DEG=2;inline constexpr unsigned UPDATE_PERIOD_MS=10;}
namespace lift_calibration {inline bool hold_validated=false;enum class Position:unsigned{INTAKE};struct Preset{bool configured;double degrees;};inline constexpr Preset positions[]={{true,38.5}};}
namespace preload_config{inline constexpr double xy_tolerance_in=.75,heading_tolerance_deg=3;}
enum class LiftFinish{STOP};
struct LiftAutonTargetStatus{bool active,settled,ready;double target_deg,position_deg;unsigned started_ms,timeout_ms;const char* reason;};
inline bool lift_target_ready(){return test::ready;}
inline double lift_position_deg(){return test::position;}
inline bool lift_at_target(){return test::lift_active && test::settled;}
inline const char* lift_auton_last_reason(){return test::reason;}
inline bool lift_auton_stop(){++test::lift_stop_calls;test::lift_active=false;return test::stop_ok;}
inline bool lift_auton_target_start(double p,unsigned ms){test::target=p;test::lift_budget=ms;test::lift_start=test::now;test::lift_active=true;test::settled=false;test::in_tolerance=0;return true;}
inline LiftAutonTargetStatus lift_auton_target_status(){return {test::lift_active,test::settled,test::ready,test::target,test::position,test::lift_start,test::lift_budget,test::reason};}
inline void mechanisms_update(){
 ++test::updates;
 if(test::fail_sensor_at && test::now>=test::fail_sensor_at){test::ready=false;test::lift_active=false;test::reason="SENSOR_FAULT";return;}
 if(!test::lift_active)return;
 ++test::nonzero_updates;
 if(std::fabs(test::target-test::position)<=2){
   if(!test::in_tolerance)test::in_tolerance=test::now;
   test::settled=test::now-test::in_tolerance>=200;
 }else{test::position+=std::clamp(test::target-test::position,-5.0,5.0);test::settled=false;test::in_tolerance=0;}
 if(!test::settled && test::now-test::lift_start>=test::lift_budget){test::lift_active=false;test::reason="TARGET_TIMEOUT";}
}
inline bool lift_auton_move_to_deg(double p,unsigned ms,LiftFinish){
 if(ez::autonomous_cancelled()||!test::ready||!ms)return false;
 lift_auton_target_start(p,ms);
 while(test::lift_active&&!test::settled&&!ez::autonomous_cancelled()){mechanisms_update();pros::delay(10);}
 bool ok=test::settled&&!ez::autonomous_cancelled();return lift_auton_stop()&&ok;
}
inline bool lift_move_calibrated_position(unsigned,unsigned ms,LiftFinish f){return lift_auton_move_to_deg(lift_calibration::positions[0].degrees,ms,f);}
inline bool toggle_connected(){return test::connected;}
inline bool toggle_run(){++test::toggle_starts;test::toggle_at=test::now;return test::toggle_start_ok;}
inline bool toggle_stop(){++test::toggle_stops;return test::toggle_stop_ok;}
bool pulse_toggle(unsigned);
inline bool claw_close(){++test::close_calls;test::close_at=test::now;if(test::claw_ok)test::closed=true;return test::claw_ok;}
inline bool claw_open(){++test::open_calls;if(test::claw_ok)test::closed=false;return test::claw_ok;}
inline bool claw_commanded_closed(){return test::closed;}
inline ez::pose pose;
inline ez::pose nav_pose_get(){return pose;}
inline bool nav_sensors_ready(){return test::ready;}
struct NavProgress{};
struct NavEvent{std::function<bool(const NavProgress&)> when;std::function<bool()> action;bool required=true;};
inline std::vector<NavEvent> events;
inline bool nav_ok(){return !test::cancel&&!chassis.interfered;}
inline bool nav_sequence_active(){return test::sequence_active;}
inline bool nav_sequence_rest_verified(){return test::rest_ok;}
inline void nav_abort(const char*){ez::drive_stop(&chassis);lift_auton_stop();toggle_stop();}
inline bool nav_events(std::vector<NavEvent> e){events=std::move(e);return true;}
struct NavSequence{
 NavSequence(bool,unsigned){test::sequence_active=true;ez::drive_stop(&chassis);lift_auton_stop();chassis.interfered=false;events.clear();}
 bool ok(){return nav_ok();}
 bool finish(){if(!test::sequence_active)return false;ez::drive_stop(&chassis);for(int i=0;i<10&&!test::cancel;++i){mechanisms_update();pros::delay(10);}lift_auton_stop();test::sequence_active=false;events.clear();return nav_ok();}
 ~NavSequence(){if(test::sequence_active)finish();}
};
enum class NavEndMode{SETTLE};
inline bool nav_go_to_pose(double x,double y,double h,int,NavEndMode){
 ++test::nav_calls;test::mode=1;unsigned began=test::now;
 while(test::now-began<test::nav_duration&&!test::cancel){
   mechanisms_update();test::reached_early|=lift_at_target();
   for(auto& e:events)if(e.when({})&&!e.action()){nav_abort("event");return false;}
   pros::delay(10);
 }
 pose={x+(test::wrong_pose?1:0),y,h+(test::wrong_heading?4:0)};return !test::nav_fail&&!test::cancel;
}
