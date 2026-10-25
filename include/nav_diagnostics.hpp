#pragma once
bool nav_sensors_ready(); // Read-only, no prints; required IMU and both trackers.
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
