#include "wall_correction.hpp"
#include "subsystems.hpp"
#include <cstdio>
namespace wall_correction {
// ----- Distance sensor configuration checks -----
// Reject invalid mounts and occupied ports before creating a sensor device.
bool available(const auton_math::Sensor& s) {
  if(!auton_math::sensor_valid(s)) return false;
  using namespace mechanism_config;
  // Current navigation ports plus hardware declarations, and P18/P19 reserved.
  for(int used : {1,2,3,4,5,7,20,18,19,LEFT_LIFT_PORT,RIGHT_LIFT_PORT,LIFT_ROTATION_PORT,
      ARM_MOTOR_PORT,ARM_ROTATION_PORT,CLAW_DISTANCE_PORT,FLEX_LEFT_PORT,FLEX_RIGHT_PORT,
      TOGGLE_PORT,TOGGLE_OTHER_PORT,LIFT_HEIGHT_DISTANCE_PORT}) if(s.port==used) return false;
  unsigned matches=0;
  for(const auto& other:sensors) if(other.configured && other.port==s.port) ++matches;
  return matches<=1;
}
void health_snapshot() {
  for(unsigned i=0;i<sensors.size();++i) {
    const auto& s=sensors[i];
    const bool ready=available(s);
    const bool connected=ready && pros::Distance(s.port).is_installed();
    std::printf("PREFLIGHT distance[%u] port=%d OPTIONAL %s\n",i,s.port,
        !s.configured ? "UNCONFIGURED" : !ready ? "INVALID/CONFLICT" : connected ? "CONNECTED" : "SENSOR_FAULT");
  }
}
// ----- Stationary wall correction -----
// Verify rest, validate the reading, then update only the measured odometry axis.
auton::Result apply(auton::Session& session,unsigned index,const auton_math::Wall& wall,
                   double rpm,unsigned stable_ms,unsigned timeout_ms) {
  using auton::Result;
  auto reject=[&](Result r,const char* reason) {
    auton::log_event("CORRECTION_REJECTED","wall",reason,r);
    return r; // Required correction is enforced by checkpoint; optional rejection preserves odom.
  };
  if(!enable_application) return reject(Result::NOT_CONFIGURED,"application disabled");
  if(!session.correction_allowed() || nav_sequence_active() || chassis.drive_mode_get()!=ez::DISABLE)
    return reject(Result::ABORTED,"requires stopped checkpoint");
  if(index>=sensors.size() || !available(sensors[index]) || !std::isfinite(rpm) || rpm<=0 ||
      !stable_ms || timeout_ms<stable_ms) return reject(Result::NOT_CONFIGURED,"sensor/stationary gates");
  const auto& s=sensors[index];
  unsigned stable_since=0, began=pros::millis(); bool stable=false;
  while(pros::millis()-began<timeout_ms) {
    if(!session.ok()) return reject(Result::ABORTED,"mode changed");
    bool stopped=!chassis.left_motors.empty() && !chassis.right_motors.empty();
    for(const auto* motors:{&chassis.left_motors,&chassis.right_motors})
      for(const auto& m:*motors) {
        const double v=m.get_actual_velocity();
        if(!std::isfinite(v) || std::fabs(v)>=rpm) stopped=false;
      }
    if(!stopped) stable=false;
    else if(!stable) { stable=true; stable_since=pros::millis(); }
    else if(pros::millis()-stable_since>=stable_ms) break;
    pros::delay(10);
  }
  if(!stable || pros::millis()-stable_since<stable_ms) return reject(Result::TIMEOUT,"not stationary");
  pros::Distance device(s.port);
  const auto mm=device.get();
  if(!device.is_installed() || mm==PROS_ERR || mm<=0 || mm==9999)
    return reject(Result::SENSOR_FAULT,"invalid reading");
  if(!auton::require_nav_ready()) return reject(Result::SENSOR_FAULT,"nav sensors");
  const auto p=nav_pose_get();
  const auto c=auton_math::correct({p.x,p.y,p.theta},s,wall,mm/25.4);
  if(c.gate!=auton_math::Gate::Accepted) return reject(Result::FAIL,"geometry/range/heading/residual gate");
  if(!session.ok()) return reject(Result::ABORTED,"mode changed");
  // Supported per-axis setters preserve IMU heading and the untouched axis.
  // Stationary-only. Do not call from a NavEvent, background task or active PID.
  if(wall.axis==auton_math::Axis::X) chassis.odom_x_set(c.coordinate);
  else chassis.odom_y_set(c.coordinate);
  session.correction_accepted();
  auton::log_event("CORRECTION_ACCEPTED","wall","single_axis",Result::PASS,{p.x,p.y,p.theta});
  return Result::PASS;
}
}
