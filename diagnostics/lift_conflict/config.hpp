#pragma once
#include <cmath>
namespace lift_diag {
// Relative diagnostic only: startup position is NOT physical home.
inline constexpr bool POWERED_TEST_APPROVED = true;
inline constexpr double RELATIVE_ENVELOPE_DEG = 5.0;
// Provisional reserve, NOT a measured stopping-distance bound.
// Outward cutoff +/-3 deg; physical motion can continue past +/-5.
inline constexpr double STOPPING_MARGIN_DEG = 2.0;
inline constexpr bool TERMINAL_TELEMETRY = false;
inline constexpr int LIFT_DIAGNOSTIC_MAX_MV = 1000;
inline constexpr unsigned LIFT_DIAGNOSTIC_LOG_PERIOD_MS = 100;
inline constexpr unsigned CONTROL_PERIOD_MS = 20;
inline constexpr unsigned MAX_CONTROL_GAP_MS = 100;
inline constexpr unsigned MAX_ACTIVATION_MS = 1000;
inline constexpr double MAX_CURRENT_MA = 750;
inline constexpr double MAX_TEMPERATURE_C = 45;
inline constexpr unsigned STALL_GRACE_MS = 150;
inline constexpr unsigned STALL_DURATION_MS = 250;
inline constexpr double STALL_RPM = 2;
inline constexpr double MAX_RPM = 100;
inline constexpr double MAX_DISPLACEMENT_DEG = 5;
inline constexpr double WRONG_DIRECTION_DEG = 0.5;
}
