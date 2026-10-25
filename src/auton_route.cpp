#include "auton_route.hpp"
#include "nav_diagnostics.hpp"
#include "subsystems.hpp"
#include "wall_correction.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <cstdio>
#include <cstring>
namespace auton {
namespace {
// ----- Route ownership -----
// One Session owns route commands and the event log until cleanup finishes.
Session* owner = nullptr;
unsigned motion_started = 0, motion_timeout = 0;
Result wait_result = Result::PASS;
auton_math::Pose pose() { const auto p=nav_pose_get(); return {p.x,p.y,p.theta}; }
bool enabled() { return !pros::competition::is_disabled() && pros::competition::is_autonomous(); }
bool sensors_ready() {
  return nav_sensors_ready();
}
// Keep checking sensors, mode and deadline while an ordinary EZ movement blocks.
bool wait_service(ez::Drive* drive) {
  if (drive != &chassis) return true;
  if (!enabled()) wait_result=Result::ABORTED;
  else if (!sensors_ready()) wait_result=Result::SENSOR_FAULT;
  else if (pros::millis()-motion_started >= motion_timeout) wait_result=Result::TIMEOUT;
  if (wait_result!=Result::PASS) {
    ez::navigation_lock(); chassis.drive_set(0,0); ez::navigation_unlock();
    return false;
  }
  // Deliberately no mechanism service: trusted primitive ownership is unchanged.
  return true;
}
bool rest() {
  if (chassis.left_motors.empty() || chassis.right_motors.empty()) return false;
  for (const auto* motors : {&chassis.left_motors,&chassis.right_motors})
    for (const auto& m:*motors) {
      const double rpm=m.get_actual_velocity();
      if (!std::isfinite(rpm) || std::fabs(rpm)>=3) return false;
    }
  return true;
}
bool settle() {
  unsigned stable=0, began=pros::millis();
  while (enabled() && pros::millis()-began<2000) {
    stable=rest() ? stable+10 : 0;
    if(stable>=100) return true;
    pros::delay(10);
  }
  return false;
}
bool target_ok(const Checkpoint& c) {
  return (!c.expected_start || auton_math::finite(*c.expected_start)) &&
      (!c.target || auton_math::finite(*c.target)) &&
      (!c.start_tolerance || (c.expected_start && c.start_tolerance->valid())) &&
      (!c.final_tolerance || (c.target && c.final_tolerance->valid())) &&
      (c.correction == CorrectionPolicy::NONE || c.correction == CorrectionPolicy::ALLOWED || c.correction == CorrectionPolicy::REQUIRED);
}
Result execute_segment(Session& session, RouteSegmentTest id, const Segment& s) {
  if (!s.configured || !s.execute) return session.abort(Result::NOT_CONFIGURED,segment_name(id));
  if (!session.checkpoint(id,s.checkpoint)) return session.result();
  return session.end_segment(s.execute(session,s));
}
}
const char* segment_name(RouteSegmentTest s) {
  switch(s) {
#define S(x) case RouteSegmentTest::x: return #x
    S(NONE); S(SNAPSHOT); S(B1_APPROACH); S(B1_FINAL_ALIGN); S(B1_SCORE); S(B1_EXIT);
    S(B2_PICKUP_APPROACH); S(B2_PICKUP); S(B2_RETURN); S(B2_FINAL_ALIGN); S(B2_SCORE); S(B2_EXIT);
    S(B3_PICKUP_APPROACH); S(B3_PICKUP); S(B3_RETURN); S(B3_FINAL_ALIGN); S(B3_SCORE); S(B3_EXIT);
    S(B4_PICKUP_APPROACH); S(B4_PICKUP); S(B4_RETURN); S(B4_FINAL_ALIGN); S(B4_SCORE); S(B4_EXIT);
    S(COUNT);
#undef S
  }
  return "INVALID";
}
// ----- Route lifetime and abort -----
// Clear earlier outputs on entry; latch failures so later Session commands cannot run.
Session::Session(const char* route) {
  if (owner || nav_sequence_active()) { result_=Result::ABORTED; return; }
  owner=this;
  nav_cancel_sequence();
  log_begin(route);
  log_event("AUTON_START","none","none",Result::RETURNED_UNVERIFIED);
  if (!enabled()) abort(Result::ABORTED,"not autonomous");
}
Session::~Session() {
  finish();
}
Result Session::finish() {
  if(owner!=this) return result_;
  if(!enabled() && result_==Result::PASS) result_=Result::ABORTED;
  nav_cancel_sequence();
  if(!stop_all_mechanisms() && result_==Result::PASS) result_=Result::FAIL;
  log_event("AUTON_END",segment_,"none",result_,segment_start_,checkpoint_.target.value_or(auton_math::Pose{}));
  owner=nullptr;
  log_flush();
  return result_;
}
bool Session::ok() const { return owner==this && result_==Result::PASS && enabled(); }
Result Session::abort(Result r,const char* reason) {
  if (result_==Result::PASS) result_= r==Result::PASS ? Result::FAIL : r;
  if(owner==this) {
    nav_cancel_sequence();
    log_event("AUTON_ABORT",segment_,reason,result_);
  }
  return result_;
}
// ----- Named stage checks -----
// Record the expected and actual poses without automatically starting or stopping motion.
bool Session::checkpoint(RouteSegmentTest id,const Checkpoint& c) {
  if(!ok()) { abort(Result::ABORTED,"inactive route"); return false; }
  segment_=segment_name(id); checkpoint_=c; corrected_=false; segment_start_=pose();
  if(!target_ok(c)) { abort(Result::NOT_CONFIGURED,"checkpoint invalid"); return false; }
  log_event("SEGMENT_START",segment_,c.mechanism_expectation,Result::RETURNED_UNVERIFIED,
            segment_start_,c.expected_start.value_or(auton_math::Pose{}));
  if(c.start_tolerance && !auton_math::within(segment_start_,*c.expected_start,*c.start_tolerance)) {
    abort(Result::FAIL,"start pose mismatch"); return false;
  }
  return true; // Checkpoint itself does not stop or dispatch anything.
}
Result Session::end_segment(Result r) {
  if(!ok()) r=result_==Result::PASS ? Result::ABORTED : result_;
  if(r==Result::PASS && checkpoint_.settle_required && !settle()) r=Result::TIMEOUT;
  if(r==Result::PASS && checkpoint_.correction==CorrectionPolicy::REQUIRED && !corrected_) r=Result::FAIL;
  if(r==Result::PASS && checkpoint_.verify_mechanisms && !checkpoint_.verify_mechanisms()) r=Result::FAIL;
  if(r==Result::PASS && checkpoint_.final_tolerance &&
      !auton_math::within(pose(),*checkpoint_.target,*checkpoint_.final_tolerance)) r=Result::FAIL;
  // PASS means configured checks passed, never independent physical truth.
  log_event("SEGMENT_END",segment_,"none",r,segment_start_,checkpoint_.target.value_or(auton_math::Pose{}));
  if(r!=Result::PASS) return abort(r, checkpoint_.failure==FailurePolicy::STOP_PREFIX ? "prefix stopped" : "segment failed");
  return r;
}
// ----- Trusted point and turn commands -----
// Add route safety and logging around the ordinary navigation wrappers.
// Active lift/arm control is rejected because these waits do not service mechanisms.
Result Session::move(const char* name,auton_math::Pose target,int speed,unsigned timeout,bool turn_only,bool reverse) {
  if(!ok() || moving_ || nav_sequence_active()) return abort(Result::ABORTED,"motion ownership conflict");
  if(speed<1 || speed>127 || !timeout || (!std::isfinite(target.h) && turn_only) ||
      (!turn_only && (!std::isfinite(target.x)||!std::isfinite(target.y)))) return abort(Result::NOT_CONFIGURED,"motion data missing");
  if(!sensors_ready()) return abort(Result::SENSOR_FAULT,"navigation sensors");
  const auto lm=lift_mode();
  if(lm==LiftMode::Hold || lm==LiftMode::MoveToTarget || lm==LiftMode::ReturningHome ||
      lm==LiftMode::AutonFull || lm==LiftMode::DriverFull || lm==LiftMode::Manual ||
      arm_mode()==ArmMode::Hold || arm_mode()==ArmMode::Manual || arm_mode()==ArmMode::MoveToTarget)
    return abort(Result::ABORTED,"lift/arm needs serviced navigation");
  const auto start=pose();
  const char* command=turn_only ? "TURN_TO" : reverse ? "GO_TO_REVERSE" : "GO_TO";
  log_event("MOTION_START",segment_,name,Result::RETURNED_UNVERIFIED,start,target,speed,NAN,command);
  moving_=true; motion_started=pros::millis(); motion_timeout=timeout; wait_result=Result::PASS;
  chassis.interfered=false; // Explicit new route motion, after all preconditions.
  ez::wait_service_set(wait_service);
  if(turn_only) nav_turn_to(target.h,speed);
  else if(reverse) nav_go_to_reverse(target.x,target.y,speed);
  else nav_go_to(target.x,target.y,speed);
  ez::wait_service_set(nullptr);
  moving_=false;
  Result r=wait_result;
  if(!enabled()) r=Result::ABORTED;
  else if(!sensors_ready()) r=Result::SENSOR_FAULT;
  else if(r==Result::PASS && chassis.interfered) r=Result::FAIL;
  log_event("MOTION_END",segment_,name,r==Result::PASS ? Result::RETURNED_UNVERIFIED : r,start,target,speed,NAN,command);
  return r==Result::PASS ? r : abort(r,"motion failed");
}
Result Session::point(const char* name,double x,double y,int speed,unsigned timeout,MotionClass kind,bool reverse) {
  if(kind!=MotionClass::TRANSIT && kind!=MotionClass::APPROACH && kind!=MotionClass::PRECISION)
    return abort(Result::NOT_CONFIGURED,"invalid motion class");
  return move(name,{x,y,NAN},speed,timeout,false,reverse);
}
Result Session::turn(const char* name,double h,int speed,unsigned timeout) { return move(name,{NAN,NAN,h},speed,timeout,true,false); }
Result Session::mechanism(const char* name,bool (*command)()) {
  if(!ok() || !command) return abort(Result::ABORTED,"mechanism ownership/request");
  const bool accepted=command();
  const auto r=accepted && enabled() ? Result::PASS : Result::FAIL;
  log_event("MECHANISM_COMMAND",segment_,name,r,{}, {},0,lift_position_deg());
  return r==Result::PASS ? r : abort(r,"mechanism rejected");
}
Result Session::stop_motion() {
  if(!ok() || nav_sequence_active()) return abort(Result::ABORTED,"stop ownership conflict");
  ez::navigation_lock(); chassis.drive_set(0,0); ez::navigation_unlock();
  return Result::PASS;
}
// ----- Optional chained movement -----
// NavSequence owns the waits and mechanism updates; all step data is checked first.
Result Session::sequence(const std::vector<auton_actions::SequenceStep>& steps,unsigned timeout) {
  if(!ok() || moving_ || nav_sequence_active()) return abort(Result::ABORTED,"sequence ownership conflict");
  if(!auton_actions::ENABLE_ADVANCED_PATHS) return abort(Result::PHYSICAL_CALIBRATION_REQUIRED,"chaining disabled");
  if(steps.empty() || !timeout || steps.back().finish!=NavEndMode::SETTLE)
    return abort(Result::NOT_CONFIGURED,"sequence invalid");
  bool previous_chain=false;
  for(const auto& step:steps) {
    const auto& p=step.target;
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || p.speed<1 || p.speed>127 || p.heading ||
        (step.finish!=NavEndMode::CHAIN && step.finish!=NavEndMode::SETTLE) ||
        (step.direction!=nav_path_direction::forward && step.direction!=nav_path_direction::reverse) ||
        (previous_chain && (step.hooks.before_move || !step.hooks.during_move.empty())) ||
        (step.finish==NavEndMode::CHAIN && step.hooks.arrival)) return abort(Result::NOT_CONFIGURED,"sequence step invalid/pose prohibited");
    if((step.hooks.before_move || step.hooks.arrival || !step.hooks.during_move.empty()) &&
        !auton_actions::ENABLE_MECHANISM_ACTIONS) return abort(Result::PHYSICAL_CALIBRATION_REQUIRED,"mechanism events disabled");
    for(const auto& e:step.hooks.during_move) if(!e.when || !e.action) return abort(Result::NOT_CONFIGURED,"event invalid");
    previous_chain=step.finish==NavEndMode::CHAIN;
  }
  if(!require_nav_ready()) return abort(Result::SENSOR_FAULT,"sequence preflight");
  bool success=true;
  {
    NavSequence sequence(false,timeout); // Preserves native CHAIN/SETTLE and same-task service.
    for(const auto& step:steps) {
      if(!ok() || !sequence.ok()) { success=false; break; }
      if(step.hooks.before_move && !step.hooks.before_move()) { nav_abort("before hook failed"); success=false; break; }
      if(!step.hooks.during_move.empty() && !nav_events(step.hooks.during_move)) { success=false; break; }
      const auto start=pose(); const auto& p=step.target;
      const char* command=step.direction==nav_path_direction::reverse ?
          (step.finish==NavEndMode::CHAIN ? "XY_REVERSE_CHAIN" : "XY_REVERSE_SETTLE") :
          (step.finish==NavEndMode::CHAIN ? "XY_CHAIN" : "XY_SETTLE");
      log_event("MOTION_START",segment_,step.name,Result::RETURNED_UNVERIFIED,start,{p.x,p.y,NAN},p.speed,NAN,command);
      success=step.direction==nav_path_direction::reverse ? nav_go_to_reverse(p.x,p.y,p.speed,step.finish) : nav_go_to(p.x,p.y,p.speed,step.finish);
      log_event("MOTION_END",segment_,step.name,success ? Result::RETURNED_UNVERIFIED : Result::FAIL,start,{p.x,p.y,NAN},p.speed,NAN,command);
      if(!success) break;
      if(step.hooks.arrival && !step.hooks.arrival()) { nav_abort("arrival hook failed"); success=false; break; }
    }
    if(!success) nav_abort("route sequence failed");
    success=sequence.finish() && success;
  }
  if(success) return Result::PASS;
  const char* fault=nav_fault();
  const auto r=!enabled() ? Result::ABORTED : std::strstr(fault,"timeout") ? Result::TIMEOUT :
      std::strstr(fault,"sensor") || std::strstr(fault,"odometry") ? Result::SENSOR_FAULT : Result::FAIL;
  return abort(r,fault);
}
bool Session::correction_allowed() const { return ok() && !moving_ && checkpoint_.correction!=CorrectionPolicy::NONE; }
void Session::correction_accepted() { if(correction_allowed()) corrected_=true; }
bool require_nav_ready() { return sensors_ready(); }
bool require_lift_calibrated() { return lift_target_ready(); }
bool require_toggle_ready() { return toggle_available() && toggle_connected(); }
bool preflight(bool nav,bool lift_required,bool toggle_required,bool flex_required) {
  const bool n=nav ? nav_print_sensor_health() : true;
  std::printf("PREFLIGHT NAV: %s\n",nav ? "REQUIRED FOR NAV" : "OPTIONAL");
  const bool m=mechanisms_preflight(lift_required,toggle_required,flex_required);
  wall_correction::health_snapshot();
  return n && m;
}
// ----- Isolated route test -----
// Run one selected stage from its actual current pose, with no automatic start reset.
Result run_route_segment_test(RouteSegmentTest test,const RouteConfig& config) {
  Session s(config.name);
  if(!s.ok()) return s.result();
  const auto i=static_cast<unsigned>(test);
  if(test==RouteSegmentTest::SNAPSHOT) {
    if(!s.checkpoint(test,{})) return s.result();
    s.end_segment(auton_math::finite(pose()) ? Result::PASS : Result::SENSOR_FAULT);
    return s.finish();
  }
  if(i<=static_cast<unsigned>(RouteSegmentTest::SNAPSHOT) || i>=config.segments.size())
    return s.abort(Result::NOT_CONFIGURED,"no segment selected");
  execute_segment(s,test,config.segments[i]); // No start reset, no prefix, no selector.
  return s.finish();
}
// ----- One-object through four-object route -----
// Run the same ordered prefix each time; adding a block requires the earlier blocks qualified.
Result run_route_prefix(const RouteConfig& c) {
  Session s(c.name);
  if(!s.ok()) return s.result();
  if(!c.enabled || c.objects<1 || c.objects>4 || !c.start || !c.start_tolerance ||
      !auton_math::finite(*c.start) || !c.start_tolerance->valid()) return s.abort(Result::NOT_CONFIGURED,"route/start unconfigured");
  const unsigned end=static_cast<unsigned>(RouteSegmentTest::B1_EXIT)+6*(c.objects-1);
  for(unsigned b=0;b+1<c.objects;++b) if(!c.qualified_prefix[b]) return s.abort(Result::PHYSICAL_CALIBRATION_REQUIRED,"prior block not qualified");
  for(unsigned i=2;i<=end;++i) if(!c.segments[i].configured || !c.segments[i].execute || !target_ok(c.segments[i].checkpoint))
    return s.abort(Result::NOT_CONFIGURED,segment_name(static_cast<RouteSegmentTest>(i)));
  if(!preflight(true)) return s.abort(Result::SENSOR_FAULT,"preflight");
  nav_set_start(c.start->x,c.start->y,c.start->h);
  if(!auton_math::within(pose(),*c.start,*c.start_tolerance)) return s.abort(Result::FAIL,"start readback");
  for(unsigned i=2;i<=end;++i) {
    const auto r=execute_segment(s,static_cast<RouteSegmentTest>(i),c.segments[i]);
    if(r!=Result::PASS) return r;
  }
  return s.finish();
}
void lifecycle_stop(const char* reason) {
  // Prior autonomous task has been terminated by PROS: do not dereference its stack.
  const bool interrupted=owner!=nullptr;
  if(interrupted) { log_event("AUTON_ABORT","lifecycle",reason,Result::ABORTED);
    log_event("AUTON_END","lifecycle",reason,Result::ABORTED); owner=nullptr; }
  nav_cancel_sequence();
  if(interrupted) log_flush(); // Motors stopped first; interrupted run remains analyzable.
}
}
