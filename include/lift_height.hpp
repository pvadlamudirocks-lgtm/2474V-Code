#pragma once
#include <cmath>

// Telemetry only. Rotation Sensor remains the primary lift-control feedback.
bool lift_height_sensor_available();
double lift_ground_distance_mm(); // Fresh, validated, unfiltered beam distance; NAN on failure.
double lift_ground_distance_in();
void lift_height_telemetry_reset();
void lift_height_telemetry_update(); // Driver owner calls every loop; samples at 20 ms.
struct LiftHeightCalibrationSample {
  unsigned timestamp_ms = 0;
  double rotation_deg = NAN;
  double raw_distance_mm = NAN;
  double filtered_distance_mm = NAN;
  double sensor_height_in = NAN; // Only with confirmed vertical/fixed-angle mounting.
  double claw_reference_height_in = NAN; // Also requires a known signed offset.
  bool sensor_valid = false; // Valid return does not prove the target is the floor.
  int confidence = -1; // Diagnostic 0..63 only above 200 mm; -1 unavailable, no cutoff.
};
// Copyable row for future logging of real (rotation, height) pairs. No fitted mapping.
LiftHeightCalibrationSample lift_height_calibration_sample();
void print_lift_height_calibration(); // On demand only; no button or automatic output.
