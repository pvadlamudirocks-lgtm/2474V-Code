#pragma once
#include "api.h"
#include "auton_math.hpp"
namespace pose_recorder {
inline constexpr bool enabled = false; // Explicit development opt-in.
inline constexpr bool start_configured = false;
inline constexpr auton_math::Pose start{}; // Supply documented physical reference.
// Call from the driver loop. Capture/reset are one-shot; start initialization may briefly wait.
void update(pros::Controller& controller);
}
