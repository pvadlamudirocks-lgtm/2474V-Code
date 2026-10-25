#include "auton_actions.hpp"
#include <cstdio>

// Development template only: never registered with the Brain selector.
// Unset optionals cannot silently turn into (0,0) competition targets.
namespace example_auton_points {
using field_points::StartPose;
using auton_actions::RoutePoint;
constexpr std::optional<StartPose> START = std::nullopt;
constexpr std::optional<RoutePoint> APPROACH = std::nullopt;
constexpr bool MEASURED_AND_ENABLED = false;
}

void auton_example_template() {
  using namespace example_auton_points;
  if (!MEASURED_AND_ENABLED || !START || !APPROACH) {
    std::printf("DEVELOPMENT EXAMPLE NOT CONFIGURED -- no movement\n");
    return;
  }
  nav_set_start(START->position.x,START->position.y,START->heading);
  const auto result = auton_actions::drive_to_point(
      {APPROACH->x,APPROACH->y},APPROACH->speed,{"example","approach"});
  if (!result.can_continue()) return;
  // Add explicit, physically commissioned mechanism actions here.
  // No collect/score recipe, timing, or lift target is assumed.
}
