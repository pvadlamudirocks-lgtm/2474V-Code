#pragma once
#include "subsystems.hpp"

namespace preload_config {
// Reporting only; neither comparison qualifies native navigation completion.
inline constexpr double xy_tolerance_in = 0.75, heading_tolerance_deg = 3.0;
}
// Same autonomous task services the lift through the native navigation wait hook.
// Success leaves the lift actively holding; call lowering immediately, without delays.
bool lift_boomerang_to(double lift_deg, double x, double y, double heading, int speed);
bool preload_lower_for_release(double target_deg, unsigned timeout_ms);
// Buffered events only; call after motion/hold outputs have stopped.
void preload_log_flush(unsigned autonomous_started_ms);
void preload_auton_finish_log(unsigned began, bool released);
