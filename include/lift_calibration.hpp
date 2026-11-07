
#pragma once
#include "auton_math.hpp"
#include <array>
#include <cmath>

namespace lift_calibration {

// =========================================
// EDIT THESE SIX VALUES TOMORROW
// All values are P16 rotation sensor degrees.
// NAN = not measured yet.
// =========================================

// Highest SAFE position, below damaging overtravel.
inline constexpr double MAX_SAFE_DEG = 3000;

// Intake position, separate from physical 0 degrees.
inline constexpr double INTAKE_DEG = 38.5;

// Alliance Goal scoring positions.
inline constexpr double ALLIANCE_GOAL_1_DEG = 250;
inline constexpr double ALLIANCE_GOAL_2_DEG = NAN;

// Neutral Goal scoring positions.
inline constexpr double NEUTRAL_SHORT_DEG = NAN;
inline constexpr double NEUTRAL_TALL_DEG = NAN;

// =========================================
// DO NOT CHANGE BELOW THIS LINE
// =========================================

enum class Position : unsigned {
  INTAKE,
  ALLIANCE_GOAL_1,
  ALLIANCE_GOAL_2,
  NEUTRAL_SHORT,
  NEUTRAL_TALL,
  MAX_SAFE
};

struct Preset {
  bool configured;
  double degrees;
};

inline constexpr std::array<Preset, 6> positions{{
    {true, INTAKE_DEG},
    {true, ALLIANCE_GOAL_1_DEG},
    {true, ALLIANCE_GOAL_2_DEG},
    {true, NEUTRAL_SHORT_DEG},
    {true, NEUTRAL_TALL_DEG},
    {true, MAX_SAFE_DEG}
}};

inline constexpr auton_math::LiftLimits limits{};
inline constexpr bool enable_limits = false;
inline constexpr bool hold_validated = false;

} // namespace lift_calibration
