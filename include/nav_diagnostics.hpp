#pragma once
bool nav_sensors_ready(); // Read-only, no prints; required IMU and both trackers.
// Diagnostic history only; remains until reboot and does not block navigation.
bool nav_sensor_fault_latched();
#include "navigation.hpp"

// Read-only device checks; false means missing, calibrating, or unreadable.
bool nav_print_sensor_health();
void nav_print_snapshot(const char* label);
// Captures final pose after the existing measured-rest check; no motor commands.
bool nav_print_test_result(const char* label, ez::pose start, ez::pose target,
                           unsigned elapsed_ms);
// Read-only sample, rate limited to 250 ms. false resets sample history only.
void nav_manual_odom_sample(bool enabled);

// Formats caller-captured data only; no sensors, rest waits or motor commands.
// The phase distinguishes CHAIN release from SETTLE return and unverified return.
void nav_print_action_record(const char* route, const char* step, ez::pose start,
                             ez::pose target, ez::pose final, unsigned elapsed_ms,
                             const char* phase, bool interfered);

// Passive readers use the same FRONT/LEFT slots attached by recovery startup.
// Index 0 = horizontal FRONT/P5; index 1 = vertical LEFT/P7.
// These functions never reset sensors, change attachment, or command output.
struct NavTrackerSample {
  const char* label;
  int expected_port, actual_port;
  bool installed, configuration_ok, valid;
  double raw_cd, ticks_per_inch, distance_in;
  int error;
};
ez::tracking_wheel* nav_diagnostic_tracker(unsigned index);
NavTrackerSample nav_read_tracker(unsigned index);
