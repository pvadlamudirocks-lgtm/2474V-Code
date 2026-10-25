#pragma once
#include "api.h"
// Controller text has one owner in driver mode. No control commands or new task.
void telemetry_reset();
void telemetry_update(pros::Controller& controller);
void print_robot_telemetry(); // Owner-task snapshot; call on demand, never each loop.
void telemetry_monitor_test(); // 30-second no-motion window, diagnostics at 500 ms.
