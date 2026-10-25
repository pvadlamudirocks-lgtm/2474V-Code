#pragma once
#include "auton_math.hpp"
#include <array>
namespace lift_calibration {
// ----- Measured cascade positions -----
// Fill these entries after physical commissioning; an unset preset cannot move the lift.
enum class Position { HOME_INTAKE, CLEARANCE, SCORE, MAX_SAFE };
struct Preset { bool configured=false; double degrees=NAN; };
inline constexpr std::array<Preset,4> positions{};
inline constexpr auton_math::LiftLimits limits{};
// Enable only after the home reference, rise direction and safe maximum are verified.
inline constexpr bool enable_limits=false;
inline constexpr bool hold_validated=false;
// Configure in the same zeroed sensor frame used by lift_zero_at_current_position.
// Feedback controller still requires normalized increasing-up feedback separately.
}
