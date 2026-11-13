#pragma once
#include "api.h"
// Sensor sampling only: never write or clear the VEX Controller competition display.
void telemetry_reset();
void telemetry_update(pros::Controller& controller);
void print_robot_telemetry(); // Owner-task snapshot; call on demand, never each loop.
void telemetry_monitor_test(); // 30-second no-motion window, diagnostics at 500 ms.
