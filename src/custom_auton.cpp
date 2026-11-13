#include "custom_auton.hpp"
#include "subsystems.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <cmath>
#include <cstdio>

namespace custom_auton {
namespace {
bool navigation_ready() { return chassis.odom_enabled() && nav_sensors_ready(); }
// User confirmed passive retention at 850 P16 degrees during driving.
// Keep STOP behavior; this does not enable powered HOLD or qualify other heights.
bool travel_height_verified(double position) {
    return std::isfinite(position) &&
           position >= mechanism_config::LIFT_MIN_DEG &&
           position <= mechanism_config::LIFT_MAX_DEG;
}
bool stopped_lift_for_travel() {
  return lift_target_ready() && lift_mode()==LiftMode::Disabled &&
      travel_height_verified(lift_position_deg());
}
bool cleanup() {
    ez::drive_stop(&chassis);

    const bool lift_ok = lift_auton_stop();
    const bool mechanisms_ok = stop_all_mechanisms();

    std::printf(
        "CLEANUP: lift=%d mechanisms=%d\n",
        lift_ok, mechanisms_ok
    );
    std::fflush(stdout);

    return lift_ok && mechanisms_ok;
}

bool toggle_pulse() {
  if(ez::autonomous_cancelled() || !toggle_connected() || !::toggle_run()) {
    ::toggle_stop(); return false;
  }
  const unsigned began=pros::millis();
  bool healthy=true;
  while(pros::millis()-began<395) {
    if(ez::autonomous_cancelled() || !toggle_connected()) { healthy=false; break; }
    const unsigned elapsed=pros::millis()-began;
    if(elapsed>=395) break;
    pros::delay((395-elapsed)<5 ? 395-elapsed : 5);
  }
  const bool stopped=::toggle_stop();
  return healthy && stopped && !ez::autonomous_cancelled();
}
bool drive_to(double x,double y,int speed) {
  // Preserve STOP at the user-verified travel height; no powered HOLD.
  if(!stopped_lift_for_travel() || !navigation_ready() || ez::autonomous_cancelled()) return false;
  ez::drive_stop(&chassis);
  if(!ez::geometry_turn_deadline_arm(&chassis,navigation_ready)) return false;
  ::nav_go_to(x,y,speed); // Preserve existing 3-inch lookahead and controller.
  ez::drive_stop(&chassis);
  const auto deadline=ez::geometry_turn_deadline_result();
  const unsigned began=pros::millis();
  unsigned stable_since=began;
  bool rested=false;
  while(pros::millis()-began<2000 && !ez::autonomous_cancelled()) {
    bool still=true;
    for(const auto* motors:{&chassis.left_motors,&chassis.right_motors}) {
      if(motors->empty()) still=false;
      for(const auto& motor:*motors) {
        const double rpm=motor.get_actual_velocity();
        if(!std::isfinite(rpm) || std::fabs(rpm)>=3) still=false;
      }
    }
    if(!still) stable_since=pros::millis();
    else if(pros::millis()-stable_since>=100) { rested=true; break; }
    pros::delay(10);
  }
  const auto pose=nav_pose_get();
  return deadline==geometry_turn::Result::Returned && rested &&
      !ez::autonomous_cancelled() && !chassis.interfered && navigation_ready() &&
      stopped_lift_for_travel() && std::hypot(x-pose.x,y-pose.y)<=0.75;
}
}
void run(std::initializer_list<Command> commands) {
  // No command executes until ALL qualification and request checks pass.
const bool cleanup_ok = cleanup();
const bool cancelled = ez::autonomous_cancelled();
const bool disabled = pros::competition::is_disabled();
const bool autonomous = pros::competition::is_autonomous();

std::printf(
    "CUSTOM START CHECK: cleanup=%d cancelled=%d disabled=%d autonomous=%d\n",
    cleanup_ok, cancelled, disabled, autonomous
);
std::fflush(stdout);

if (!cleanup_ok || cancelled || disabled || !autonomous) {
    std::printf("CUSTOM AUTON BLOCKED AT START\n");
    return;
}
  bool qualified=true;
  double planned_lift=lift_position_deg();
  for(const auto& c:commands) {
    if(c.kind==Kind::Lift) {
      if(!lift_target_ready() || !std::isfinite(c.x) ||
         c.x<mechanism_config::LIFT_MIN_DEG || c.x>mechanism_config::LIFT_MAX_DEG) qualified=false;
      planned_lift=c.x;
    } else if(c.kind==Kind::Drive) {
      if(!travel_height_verified(planned_lift)) {
        std::printf("CUSTOM AUTON BLOCKED: travel height is not home or the user-verified 850 degrees\n");
        qualified=false;
      }
      if(!navigation_ready() || !std::isfinite(c.x) || !std::isfinite(c.y) ||
         c.speed<1 || c.speed>127) qualified=false;
    } else if(c.kind==Kind::Toggle) {
      if(!toggle_connected()) {
        std::printf("CUSTOM AUTON BLOCKED: configured Toggle hardware disconnected\n"); qualified=false;
      }
    } else if(!claw_pneumatic_available()) qualified=false;
  }
  if(!qualified) {
    cleanup(); std::printf("CUSTOM AUTON: no commands dispatched\n"); return;
  }
  unsigned step=0;
  for(const auto& c:commands) {
    ++step;
    bool ok=false;  
    if(!ez::autonomous_cancelled()) {
      switch(c.kind) {
        case Kind::Close:
            ok = ::claw_open();
            break;

        case Kind::Open:
            ok = ::claw_close();
            break;

        case Kind::Toggle: ok=toggle_pulse(); break;
        case Kind::Lift:
          ok=::lift_auton_move_to_deg(c.x,mechanism_config::TARGET_TIMEOUT_MS,LiftFinish::STOP);
          break;
        case Kind::Drive: ok=drive_to(c.x,c.y,c.speed); break;
      }
    }
    if (ez::autonomous_cancelled() ||
    pros::competition::is_disabled() ||
    !pros::competition::is_autonomous()) {

    const bool stopped = cleanup();

    std::printf(
        "CUSTOM AUTON STOPPED: step=%u cleanup=%d\n",
        step, stopped
    );
    return;
}

if (!ok) {
    std::printf(
        "CUSTOM AUTON WARNING: step=%u failed\n",
        step
    );

    // A failed lift command must leave the lift stopped.
    // Never continue with an unverified lift position.
    if (c.kind == Kind::Lift) {
        const bool stopped = lift_auton_stop();

        const double actual = lift_position_deg();

        if (!stopped ||
            !std::isfinite(actual) ||
            !travel_height_verified(actual)) {

            cleanup();

            std::printf(
                "CUSTOM AUTON STOPPED: unsafe lift position %.2f\n",
                actual
            );
            return;
        }
    }

    std::printf(
        "CUSTOM AUTON CONTINUING AFTER STEP %u\n",
        step
    );
}
  }
  const bool stopped=cleanup();
  std::printf("CUSTOM AUTON %s cleanup=%d\n",stopped?"COMPLETE":"FAILED",stopped);
}
}
