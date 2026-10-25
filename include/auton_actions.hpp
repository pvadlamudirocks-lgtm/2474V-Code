#pragma once
#include "field_points.hpp"
#include "nav_sequence.hpp"
#include <functional>
#include <optional>
#include <vector>

namespace auton_actions {
inline constexpr bool ENABLE_AUTON_TELEMETRY = false;
inline constexpr bool ENABLE_ADVANCED_PATHS = false;
inline constexpr bool ENABLE_MECHANISM_ACTIONS = false;
inline constexpr bool ENABLE_DISTANCE_CORRECTION = false;

using field_points::FieldPoint;
struct RoutePoint {
  double x;
  double y;
  int speed;
  std::optional<double> heading = std::nullopt; // FRONT heading; nullopt or EZ sentinel = XY only.
};
struct Context {
  const char* route = "unnamed route";
  const char* step = "unnamed step";
  bool telemetry = ENABLE_AUTON_TELEMETRY;
};
enum class Status {
  ReturnedUnverified, SequenceComplete, Disabled, InvalidRequest,
  Failed, Cancelled, Unconfigured
};
struct ActionResult {
  Status status;
  unsigned elapsed_ms = 0;
  // Continued execution is permitted, NOT proof of physical arrival.
  bool can_continue() const {
    return status == Status::ReturnedUnverified || status == Status::SequenceComplete;
  }
};
const char* status_name(Status status);

// All actions run synchronously in the autonomous task. Do not nest in NavSequence
// or call from a NavEvent. Basic helpers invoke the original non-sequence overloads.
ActionResult drive_to_point(FieldPoint point, int speed, Context context = {});
ActionResult drive_to_point_reverse(FieldPoint point, int speed, Context context = {});
ActionResult drive_to_pose(RoutePoint point, Context context = {});
ActionResult turn_to_heading(double heading, int speed, Context context = {});
ActionResult approach_goal(FieldPoint point, int speed, Context context = {});
ActionResult follow_path(const std::vector<nav_waypoint>& points, int speed,
                         std::optional<double> heading = std::nullopt,
                         nav_path_direction direction = nav_path_direction::forward,
                         Context context = {});

using MechanismAction = std::function<bool()>;
struct MoveHooks {
  MechanismAction before_move;
  std::vector<NavEvent> during_move; // Existing same-task wait-service hooks, not threads.
  MechanismAction arrival;          // SETTLE only; never at a CHAIN release.
};
struct SequenceStep {
  RoutePoint target;
  NavEndMode finish = NavEndMode::SETTLE;
  nav_path_direction direction = nav_path_direction::forward;
  MoveHooks hooks;
  const char* name = "move";
};
// Explicit advanced opt-in. Owns one NavSequence, keeps its tuning/exit behavior.
// Validates ALL steps before starting. Final step must SETTLE. No telemetry at handoffs.
ActionResult run_sequence(const std::vector<SequenceStep>& steps, Context context = {});
ActionResult mechanism_action(const MechanismAction& action, Context context = {});
// Explicit stationary hook. Bind an existing dist_reset_* API after commissioning
// and enabling its source in the build. No distance-module linkage or automatic reset.
ActionResult correct_pose_from_wall(const MechanismAction& correction, Context context = {});
// Immediate capture = close+intake; release = open+outtake. No hidden travel/delay.
ActionResult collect_object(int intake_power, Context context = {});
ActionResult score_object(int outtake_power, Context context = {});
// Explicit calibrated preset, caller-measured timeout, then release. Stops lift
// before release. Requires scoring calibration and existing feedback/limit gates.
ActionResult score_high(unsigned timeout_ms, int outtake_power, Context context = {});
ActionResult score_low(unsigned timeout_ms, int outtake_power, Context context = {});
// Compatibility: configured power for collect/release; score presets without a
// measured timeout reject as Unconfigured. Prefer the explicit overloads above.
ActionResult collect_object(Context context = {});
ActionResult score_object(Context context = {});
ActionResult score_high(Context context = {});
ActionResult score_low(Context context = {});
// Explicit caller cancellation through existing navigation cleanup. No retry motion.
ActionResult cancel_route();

// Reflect across the caller's known X=axis_x line; never infer an origin/field size.
constexpr FieldPoint mirror_x(FieldPoint point, double axis_x) {
  return {2.0 * axis_x - point.x, point.y};
}
// Direction vector is (sin h, cos h); X reflection gives (-sin h, cos h), hence -h.
// Input must be normalized [0,360); the EZ heading sentinel is preserved separately.
constexpr double mirror_normalized_heading(double heading) {
  return heading == 0.0 ? 0.0 : 360.0 - heading;
}
std::optional<RoutePoint> mirror_x(RoutePoint point, double axis_x);
void run_helper_math_test(); // Pure math, no device reads/resets/motor calls.
} // namespace auton_actions
