#pragma once
#include <limits>

// Pure calculations, independent of PROS and motor ownership.
namespace stability {
constexpr double invalid = std::numeric_limits<double>::quiet_NaN();
constexpr bool finite(double v) {
  return v == v && v <= std::numeric_limits<double>::max() &&
         v >= -std::numeric_limits<double>::max();
}
constexpr double magnitude(double v) { return v < 0 ? -v : v; }
constexpr double clamp(double v, double limit) {
  return !finite(v) || !finite(limit) || limit < 0 ? 0 :
      v > limit ? limit : v < -limit ? -limit : v;
}
constexpr double transform(double angle, int sign) {
  return finite(angle) && magnitude(angle) <= 180 && (sign == 1 || sign == -1)
      ? angle * sign : invalid;
}
enum class State { DISABLED, STABLE, WARNING, RECOVERY, SEVERE };
struct Settings {
  bool calibrated = false;
  double warning_pitch = invalid, recovery_pitch = invalid, severe_pitch = invalid;
  double exit_pitch = invalid, warning_roll = invalid, severe_roll = invalid;
  double exit_roll = invalid, rate_horizon_s = invalid;
  double k_angle = invalid, k_rate = invalid, max_correction = invalid;
};
constexpr bool valid(const Settings& c) {
  return c.calibrated && finite(c.exit_pitch) && c.exit_pitch >= 0 &&
      finite(c.warning_pitch) && c.exit_pitch < c.warning_pitch &&
      finite(c.recovery_pitch) && c.warning_pitch < c.recovery_pitch &&
      finite(c.severe_pitch) && c.recovery_pitch < c.severe_pitch && c.severe_pitch <= 180 &&
      finite(c.exit_roll) && c.exit_roll >= 0 && finite(c.warning_roll) &&
      c.exit_roll < c.warning_roll && finite(c.severe_roll) &&
      c.warning_roll < c.severe_roll && c.severe_roll <= 180 &&
      finite(c.rate_horizon_s) && c.rate_horizon_s >= 0;
}
struct Input { double pitch, roll, pitch_rate, roll_rate; };
constexpr bool valid(Input v) {
  return finite(v.pitch) && magnitude(v.pitch) <= 180 && finite(v.roll) &&
      magnitude(v.roll) <= 180 && finite(v.pitch_rate) && finite(v.roll_rate);
}
constexpr State classify(State previous, Input v, const Settings& c) {
  if (!valid(c) || !valid(v)) return State::DISABLED;
  const double p = magnitude(v.pitch), r = magnitude(v.roll);
  // Project toward/away from upright without treating a fast return through zero
  // as a predicted tip in the opposite direction.
  const double projected_p_raw = p + c.rate_horizon_s * (v.pitch < 0 ? -v.pitch_rate : v.pitch_rate);
  const double projected_r_raw = r + c.rate_horizon_s * (v.roll < 0 ? -v.roll_rate : v.roll_rate);
  if (!finite(projected_p_raw) || !finite(projected_r_raw)) return State::DISABLED;
  const double projected_p = projected_p_raw > 0 ? projected_p_raw : 0;
  const double projected_r = projected_r_raw > 0 ? projected_r_raw : 0;
  if (!finite(projected_p) || !finite(projected_r)) return State::DISABLED;
  if (p >= c.severe_pitch || r >= c.severe_roll ||
      projected_p >= c.severe_pitch || projected_r >= c.severe_roll) return State::SEVERE;
  if (previous == State::SEVERE && (p >= c.recovery_pitch || r >= c.warning_roll))
    return State::SEVERE;
  if (p >= c.recovery_pitch || projected_p >= c.recovery_pitch ||
      ((previous == State::RECOVERY || previous == State::SEVERE) &&
       (p > c.exit_pitch || projected_p > c.exit_pitch))) return State::RECOVERY;
  if (p >= c.warning_pitch || r >= c.warning_roll ||
      projected_p >= c.warning_pitch || projected_r >= c.warning_roll ||
      (previous != State::STABLE && previous != State::DISABLED &&
       (p > c.exit_pitch || r > c.exit_roll))) return State::WARNING;
  return State::STABLE;
}
constexpr bool gains_valid(const Settings& c) {
  return finite(c.k_angle) && finite(c.k_rate) && finite(c.max_correction) &&
      c.max_correction > 0 && c.max_correction <= 127;
}
constexpr double correction(State state, Input v, const Settings& c) {
  if (!valid(c) || !valid(v) || !gains_valid(c) ||
      (state != State::RECOVERY && state != State::SEVERE)) return 0;
  // Signed gains must be measured; no assumed motor/tilt sign relationship.
  return clamp(c.k_angle * v.pitch + c.k_rate * v.pitch_rate, c.max_correction);
}
struct DriverRequest { double forward, turn; };
// Proposal only. The caller must own drivetrain output before applying it.
constexpr DriverRequest propose_driver_command(DriverRequest requested, State state,
                                               Input v, const Settings& c) {
  if (!valid(c) || !valid(v) || !gains_valid(c) || state == State::DISABLED)
    return {clamp(requested.forward, 127), clamp(requested.turn, 127)};
  // A tank drive cannot fix roll by strafing or by an invented steering correction.
  if (magnitude(v.roll) >= c.severe_roll) return {0, 0};
  return {clamp(requested.forward + correction(state, v, c), 127),
          clamp(requested.turn, 127)};
}
// Angle low-pass followed by a rate low-pass; reset after invalid/gapped samples.
// 80 ms is signal smoothing, not a physical anti-tip threshold or tuned gain.
struct RateFilter {
  bool ready = false;
  double angle = 0, rate = 0;
  constexpr void reset() { ready = false; angle = rate = 0; }
  constexpr double update(double sample, double dt) {
    if (!finite(sample) || magnitude(sample) > 180 || !finite(dt) || dt <= 0 || dt > .2) {
      reset(); return invalid;
    }
    if (!ready) { ready = true; angle = sample; return invalid; }
    double delta = sample - angle;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    const double alpha = dt / (.08 + dt);
    const double step = alpha * delta;
    angle += step;
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    rate += alpha * (step / dt - rate);
    return rate;
  }
};
}
