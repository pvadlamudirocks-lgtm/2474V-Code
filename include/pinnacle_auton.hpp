#pragma once
#include <limits>

namespace pinnacle_config {
// Physical qualification is NOT established by compilation or a pneumatic write.
inline constexpr bool claw_toggle_overlap_verified = false;
}

// Single-use evidence from THIS autonomous caller's successful combined segment.
// Keep local to one autonomous invocation; never persist across mode transitions.
class PinnacleArrival {
 public:
  PinnacleArrival() = default;
  PinnacleArrival(const PinnacleArrival&) = delete;
  PinnacleArrival& operator=(const PinnacleArrival&) = delete;
 private:
  bool valid_ = false;
  double x_ = 0, y_ = 0, heading_ = 0;
  void* owner_ = nullptr;
  friend bool auton_boomerang_with_lift(double,double,double,int,double,unsigned,unsigned,PinnacleArrival*);
  friend bool auton_score_at_goal(PinnacleArrival&,double,unsigned,bool,bool);
};

// Explicit drive budget: 1..5000 ms, checked cooperatively; existing motor-owner
// watchdog independently imposes 5000 ms. STOP/rest/final-pose check, never CHAIN.
// Continued active lift hold requires existing lift_calibration::hold_validated.
bool auton_boomerang_with_lift(double x, double y, double heading_deg, int drive_speed,
    double lift_target_deg, unsigned lift_timeout_ms, unsigned drive_timeout_ms,
    PinnacleArrival* arrival = nullptr);
bool auton_lift_to_intake(unsigned timeout_ms);
bool auton_claw_close_with_toggle(unsigned toggle_pulse_ms = 395);
// Explicit per-delivery physical evidence: commanded-closed is NOT object sensing.
// Arrival is consumed even on failure. No release after any failed check.
bool auton_score_at_goal(PinnacleArrival& arrival, double score_height_deg,
    unsigned timeout_ms, bool descent_clearance_verified, bool object_secured_verified);
// Waits on an ALREADY STARTED normal lift target; never restarts its deadline.
// Stops lift on success/failure. Same autonomous task only; no drive may be active.
bool auton_wait_for_lift(double target_deg, unsigned timeout_ms);
void auton_safe_abort(const char* reason); // reason must have static lifetime

// Deliberately inactive future interfaces. No guessed coordinates/heights/timings.
struct PinnacleLoaderPlan {
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double heading_deg = std::numeric_limits<double>::quiet_NaN();
};
struct PinnacleRoutePlan {}; // Populate only after the measured route is qualified.
bool auton_acquire_from_loader(const PinnacleLoaderPlan&);
bool auton_run_pinnacle_route(const PinnacleRoutePlan&);
