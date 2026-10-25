#pragma once
#include <cmath>

// Device-independent, inches/degrees, FRONT +Y and clockwise positive heading.
namespace auton_math {
constexpr double pi = 3.14159265358979323846;
inline double rad(double d) { return d * pi / 180.0; }
inline double wrap(double d) { double h = std::fmod(d, 360.0); return h < 0 ? h + 360 : h; }
inline double error(double target, double actual) { return std::remainder(target - actual, 360.0); }
// ----- Pose checks -----
// NaN marks a missing measurement so an unset value cannot pass a target check.
struct Pose { double x = NAN, y = NAN, h = NAN; };
inline bool finite(Pose p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.h); }
inline Pose mirror(Pose p) { return {-p.x, p.y, wrap(-p.h)}; }
struct Tolerance {
  double xy = NAN, heading = NAN;
  bool valid() const { return std::isfinite(xy) && xy >= 0 && std::isfinite(heading) && heading >= 0 && heading <= 180; }
};
inline bool within(Pose actual, Pose target, Tolerance t) {
  return finite(actual) && finite(target) && t.valid() &&
      std::hypot(actual.x-target.x, actual.y-target.y) <= t.xy &&
      std::fabs(error(target.h, actual.h)) <= t.heading;
}
// ----- Wall measurement geometry -----
// Sensor offsets use the robot frame; wall coordinates use the field frame.
enum class Axis { X, Y };
struct Sensor {
  bool configured = false;
  int port = 0;
  double right = NAN, forward = NAN, heading = NAN; // mount relative to tracking center
  double min_in = NAN, max_in = NAN;
};
struct Wall {
  bool configured = false;
  Axis axis = Axis::X;
  double coordinate = NAN;
  int facing = 0; // +1 or -1 normal pointing FROM robot TO wall
  double max_angle = NAN, max_residual = NAN;
};
enum class Gate { Accepted, Unconfigured, InvalidReading, InvalidGeometry, Heading, Residual };
struct Correction { Gate gate; double coordinate = NAN, residual = NAN, predicted_range = NAN; };
inline bool sensor_valid(const Sensor& s) {
  return s.configured && s.port >= 1 && s.port <= 21 &&
      std::isfinite(s.right) && std::isfinite(s.forward) && std::isfinite(s.heading) &&
      std::isfinite(s.min_in) && std::isfinite(s.max_in) && s.min_in > 0 && s.max_in > s.min_in;
}
// Calculate a candidate position only; the caller decides whether to apply it.
inline Correction correct(Pose p, const Sensor& s, const Wall& w, double range) {
  if (!s.configured || !w.configured) return {Gate::Unconfigured};
  if (!finite(p) || !sensor_valid(s) ||
      (w.axis != Axis::X && w.axis != Axis::Y) || (w.facing != 1 && w.facing != -1) ||
      !std::isfinite(w.coordinate) || !std::isfinite(w.max_angle) || w.max_angle < 0 || w.max_angle >= 90 ||
      !std::isfinite(w.max_residual) || w.max_residual < 0) return {Gate::InvalidGeometry};
  if (!std::isfinite(range) || range < s.min_in || range > s.max_in) return {Gate::InvalidReading};
  const double normal = w.axis == Axis::X ? (w.facing > 0 ? 90 : 270) : (w.facing > 0 ? 0 : 180);
  if (std::fabs(error(normal, p.h+s.heading)) > w.max_angle) return {Gate::Heading};
  // Rotate the sensor mount and its viewing direction into the field frame.
  const double offset = w.axis == Axis::X ? s.right*std::cos(rad(p.h)) + s.forward*std::sin(rad(p.h))
                                         : -s.right*std::sin(rad(p.h)) + s.forward*std::cos(rad(p.h));
  const double ray = w.axis == Axis::X ? std::sin(rad(p.h+s.heading)) : std::cos(rad(p.h+s.heading));
  const double current = w.axis == Axis::X ? p.x : p.y;
  const double predicted = (w.coordinate-current-offset)/ray;
  if (!std::isfinite(predicted) || predicted <= 0) return {Gate::InvalidGeometry};
  const double coordinate = w.coordinate-offset-range*ray;
  // Reject large disagreements so a bad distance reading cannot corrupt the pose.
  const double residual = coordinate-current;
  if (!std::isfinite(coordinate) || std::fabs(residual) > w.max_residual)
    return {Gate::Residual, coordinate, residual, predicted};
  return {Gate::Accepted, coordinate, residual, predicted};
}
// Parallel rays and perpendicular baseline, equal forward mount projection.
// baseline is signed sensor B displacement along the RIGHT of the ray of A.
// Positive result = robot heading clockwise of the wall normal. Estimate only.
inline double paired_heading_delta(double a, double b, double baseline) {
  if (!std::isfinite(a) || !std::isfinite(b) || a <= 0 || b <= 0 ||
      !std::isfinite(baseline) || baseline == 0) return NAN;
  return std::atan((b-a)/baseline)*180.0/pi;
}
inline double paired_heading_estimate(Pose p, const Sensor& a, const Sensor& b,
                                      const Wall& wall, double range_a, double range_b) {
  // Reject nonparallel rays or a baseline with an unknown longitudinal component.
  if (!sensor_valid(a) || !sensor_valid(b) || a.port==b.port ||
      std::fabs(error(a.heading,b.heading))>1e-9) return NAN;
  const double dx=b.right-a.right, dy=b.forward-a.forward;
  const double longitudinal=dx*std::sin(rad(a.heading))+dy*std::cos(rad(a.heading));
  const double baseline=dx*std::cos(rad(a.heading))-dy*std::sin(rad(a.heading));
  if(std::fabs(longitudinal)>1e-9 || std::fabs(baseline)<1e-9 ||
      correct(p,a,wall,range_a).gate!=Gate::Accepted || correct(p,b,wall,range_b).gate!=Gate::Accepted) return NAN;
  const double delta=paired_heading_delta(range_a,range_b,baseline);
  if(!std::isfinite(delta) || std::fabs(delta)>wall.max_angle) return NAN;
  const double normal=wall.axis==Axis::X ? (wall.facing>0 ? 90 : 270) : (wall.facing>0 ? 0 : 180);
  return wrap(normal+delta-a.heading); // Report-only; never writes IMU/odometry heading.
}
// ----- Direction-aware lift limits -----
// Measured sensor direction makes these checks work with either rotation polarity.
struct LiftLimits {
  bool configured = false;
  int rising_sign = 0; // measured +1/-1; never inferred
  double home = NAN, max_safe = NAN;
  bool valid() const {
    return configured && (rising_sign == 1 || rising_sign == -1) &&
        std::isfinite(home) && std::isfinite(max_safe) && (max_safe-home)*rising_sign > 0;
  }
};
inline bool lift_direction_allowed(const LiftLimits& l, double position, int up_direction) {
  if (!l.valid() || !std::isfinite(position) || (up_direction != 1 && up_direction != -1)) return false;
  const double height = (position-l.home)*l.rising_sign;
  const double maximum = (l.max_safe-l.home)*l.rising_sign;
  return up_direction > 0 ? height < maximum : height > 0;
}
} // namespace auton_math
