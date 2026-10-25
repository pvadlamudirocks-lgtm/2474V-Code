#include "lift_height.hpp"
#include "lift_height_math.hpp"
#include "subsystems.hpp"
#include "wall_correction.hpp"
#include <cstdio>

namespace {
using namespace mechanism_config;
constexpr unsigned SAMPLE_MS = 20;
constexpr unsigned STALE_MS = 100;
lift_height_math::Median distance_filter;
unsigned last_sample = 0;
bool sampled = false;
LiftHeightCalibrationSample latest;

bool port_free() {
  const int port = LIFT_HEIGHT_DISTANCE_PORT;
  if (port < 1 || port > 21) return false;
  // Current drive, tracking and IMU map; P13 is reserved by existing port checks.
  for (int used : {1, 2, 3, 4, 5, 7, 13, 20, LEFT_LIFT_PORT, RIGHT_LIFT_PORT,
                   LIFT_ROTATION_PORT, ARM_MOTOR_PORT, ARM_ROTATION_PORT,
                   CLAW_DISTANCE_PORT, FLEX_LEFT_PORT, FLEX_RIGHT_PORT,
                   TOGGLE_PORT, TOGGLE_OTHER_PORT})
    if (port == used) return false;
  for (const auto& sensor : wall_correction::sensors)
    if (sensor.configured && sensor.port == port) return false;
  return true;
}
pros::Distance& lift_height_distance_sensor() {
  // Called only after port validation: never construct a device on port zero.
  static pros::Distance sensor(LIFT_HEIGHT_DISTANCE_PORT);
  return sensor;
}
double vertical_height_in(double beam_mm) {
  if (!std::isfinite(beam_mm)) return NAN;
  if (LIFT_HEIGHT_SENSOR_MOUNTED_VERTICALLY &&
      !LIFT_HEIGHT_SENSOR_FIXED_ANGLE_CONFIGURED) return beam_mm / 25.4;
  const double angle = LIFT_HEIGHT_SENSOR_ANGLE_FROM_VERTICAL_DEG;
  if (!LIFT_HEIGHT_SENSOR_MOUNTED_VERTICALLY &&
      LIFT_HEIGHT_SENSOR_FIXED_ANGLE_CONFIGURED && std::isfinite(angle) &&
      angle >= 0.0 && angle < 90.0)
    return beam_mm * std::cos(angle * 3.14159265358979323846 / 180.0) / 25.4;
  return NAN; // Unknown, conflicting or dynamic geometry is not vertical height.
}
}

bool lift_height_sensor_available() {
  return port_free() && lift_height_distance_sensor().is_installed();
}
double lift_ground_distance_mm() {
  if (!lift_height_sensor_available()) return NAN;
  const auto mm = lift_height_distance_sensor().get();
  // Installed PROS: PROS_ERR on failure, 9999 for no object. VEX range: 20..2000 mm.
  if (mm == PROS_ERR || mm == 9999 || !lift_height_math::valid_distance(mm)) return NAN;
  if (LIFT_HEIGHT_DISTANCE_BOUNDS_CONFIGURED) {
    const double low = LIFT_HEIGHT_DISTANCE_MIN_MM, high = LIFT_HEIGHT_DISTANCE_MAX_MM;
    if (!std::isfinite(low) || !std::isfinite(high) || low > high ||
        mm < low || mm > high) return NAN;
  }
  return static_cast<double>(mm);
}
double lift_ground_distance_in() { return lift_ground_distance_mm() / 25.4; }

void lift_height_telemetry_reset() {
  distance_filter.reset();
  sampled = false;
  latest = LiftHeightCalibrationSample{};
}
void lift_height_telemetry_update() {
  const unsigned now = pros::millis();
  if (sampled && now - last_sample < SAMPLE_MS) return;
  if (sampled && now - last_sample > STALE_MS) distance_filter.reset();
  sampled = true;
  last_sample = now;
  latest = LiftHeightCalibrationSample{};
  latest.timestamp_ms = now;
  latest.rotation_deg = lift_position_deg();
  latest.raw_distance_mm = lift_ground_distance_mm();
  if (!std::isfinite(latest.raw_distance_mm)) {
    // Never display an old height after a failed reading or blend across a dropout.
    distance_filter.reset();
    return;
  }
  latest.sensor_valid = true;
  // Three-sample median rejects isolated spikes, with only ~20 ms motion lag.
  latest.filtered_distance_mm = distance_filter.add(latest.raw_distance_mm);
  if (latest.raw_distance_mm > 200) {
    const auto confidence = lift_height_distance_sensor().get_confidence();
    if (confidence != PROS_ERR && confidence >= 0 && confidence <= 63)
      latest.confidence = confidence; // Diagnostic only; no invented quality cutoff.
  }
  latest.sensor_height_in = vertical_height_in(latest.filtered_distance_mm);
  if (LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_CONFIGURED &&
      std::isfinite(LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_IN))
    latest.claw_reference_height_in = latest.sensor_height_in + LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_IN;
}
LiftHeightCalibrationSample lift_height_calibration_sample() {
  if (!sampled || pros::millis() - last_sample > STALE_MS) return {};
  return latest;
}
void print_lift_height_calibration() {
  lift_height_telemetry_update();
  const auto row = lift_height_calibration_sample();
  std::printf("LIFT HEIGHT CALIBRATION t=%u ms\nRotation: %.2f deg\n"
              "Raw beam distance: %.0f mm\nFiltered beam distance: %.2f mm\n"
              "Sensor-face vertical height: %.2f in\nClaw reference height: %.2f in\n"
              "Sensor valid: %s (does not identify floor); confidence=%d (-1 unavailable)\n",
              row.timestamp_ms, row.rotation_deg, row.raw_distance_mm,
              row.filtered_distance_mm, row.sensor_height_in, row.claw_reference_height_in,
              row.sensor_valid ? "YES" : "NO", row.confidence);
}
