#pragma once
#include "auton_math.hpp"
namespace auton {
// Results distinguish checked success from an action that merely returned.
enum class Result { PASS, FAIL, ABORTED, NOT_CONFIGURED, PHYSICAL_CALIBRATION_REQUIRED, TIMEOUT, SENSOR_FAULT, RETURNED_UNVERIFIED };
const char* result_name(Result result);
// Static labels only; bounded buffer, no heap or serial writes at motion handoffs.
void log_begin(const char* route);
void log_event(const char* event, const char* segment, const char* motion, Result result,
               auton_math::Pose start = {}, auton_math::Pose target = {},
               int speed = 0, double mechanism_position = NAN, const char* command = "none");
void log_flush(); // Only after drivetrain stopped, in the owning autonomous task.
}
