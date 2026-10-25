#pragma once
#include "auton_log.hpp"
#include "nav_sequence.hpp"
#include "auton_actions.hpp"
#include <array>
#include <optional>
namespace auton {
enum class MotionClass { TRANSIT, APPROACH, PRECISION };
enum class CorrectionPolicy { NONE, ALLOWED, REQUIRED };
enum class FailurePolicy { ABORT_ROUTE, STOP_PREFIX }; // Both stop; never skip failed block.
enum class RouteSegmentTest {
  NONE, SNAPSHOT,
  B1_APPROACH, B1_FINAL_ALIGN, B1_SCORE, B1_EXIT,
  B2_PICKUP_APPROACH, B2_PICKUP, B2_RETURN, B2_FINAL_ALIGN, B2_SCORE, B2_EXIT,
  B3_PICKUP_APPROACH, B3_PICKUP, B3_RETURN, B3_FINAL_ALIGN, B3_SCORE, B3_EXIT,
  B4_PICKUP_APPROACH, B4_PICKUP, B4_RETURN, B4_FINAL_ALIGN, B4_SCORE, B4_EXIT,
  COUNT
};
const char* segment_name(RouteSegmentTest segment);
// Stage expectations are optional; only configured checks are enforced.
// A checkpoint describes a boundary, not necessarily a stop.
struct Checkpoint {
  std::optional<auton_math::Pose> expected_start, target;
  std::optional<auton_math::Tolerance> start_tolerance, final_tolerance;
  const char* mechanism_expectation = "unspecified";
  bool (*verify_mechanisms)() = nullptr; // Explicit feedback check, not inferred state.
  bool settle_required = false;
  CorrectionPolicy correction = CorrectionPolicy::NONE;
  FailurePolicy failure = FailurePolicy::ABORT_ROUTE;
};
class Session;
// Pair measured stage settings with one explicit route function.
// Missing configuration never substitutes a guessed movement.
struct Segment {
  bool configured = false;
  Checkpoint checkpoint;
  Result (*execute)(Session&, const Segment&) = nullptr;
  // Named measured values for an explicit stage callback; no automatic movements.
  std::optional<auton_math::Pose> point;
  int drive_speed = 0, turn_speed = 0;
  unsigned timeout_ms = 0;
  MotionClass motion_class = MotionClass::PRECISION;
};
// Each route chooses a start and object count while reusing the same stage order.
struct RouteConfig {
  const char* name = "unconfigured";
  bool enabled = false;
  unsigned objects = 0;
  std::optional<auton_math::Pose> start;
  std::optional<auton_math::Tolerance> start_tolerance;
  std::array<Segment, static_cast<unsigned>(RouteSegmentTest::COUNT)> segments{};
  std::array<bool,4> qualified_prefix{}; // B2 requires proven B1, etc.
};
// Single autonomous task owner. New route layer never uses pose-Boomerang.
class Session {
 public:
  explicit Session(const char* route);
  ~Session();
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;
  bool ok() const;
  Result abort(Result result, const char* reason);
  Result result() const { return result_; }
  Result finish(); // Includes stop-write result before returning a test/route result.
  bool checkpoint(RouteSegmentTest id, const Checkpoint& checkpoint);
  Result end_segment(Result result);
  Result point(const char* name, double x, double y, int speed, unsigned timeout_ms,
               MotionClass kind, bool reverse = false);
  Result turn(const char* name, double heading, int speed, unsigned timeout_ms);
  Result mechanism(const char* name, bool (*command)());
  Result stop_motion(); // Explicit stationary boundary before wall correction.
  Result sequence(const std::vector<auton_actions::SequenceStep>& steps, unsigned timeout_ms);
  bool correction_allowed() const;
  void correction_accepted();
 private:
  Result result_ = Result::PASS;
  const char* segment_ = "none";
  Checkpoint checkpoint_{};
  bool corrected_ = false;
  bool moving_ = false;
  auton_math::Pose segment_start_{};
  Result move(const char*, auton_math::Pose, int, unsigned, bool, bool);
};
bool require_nav_ready();
bool require_lift_calibrated();
bool require_toggle_ready();
bool preflight(bool navigation, bool lift = false, bool toggle = false, bool flex = false);
Result run_route_segment_test(RouteSegmentTest test, const RouteConfig& config);
Result run_route_prefix(const RouteConfig& config);
void lifecycle_stop(const char* reason); // disabled/opcontrol; existing nav cleanup owner
}
