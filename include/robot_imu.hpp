#pragma once
#include "stability_math.hpp"

namespace robot_imu_config {
// Preserve existing sensor-local signs until physical direction tests are recorded.
inline constexpr int IMU_PITCH_SIGN = 1;
inline constexpr int IMU_ROLL_SIGN = 1;
inline constexpr bool AXES_VERIFIED = false;
// No heading offset: canonical navigation remains the existing source of heading.
}
struct RobotOrientation {
  double heading = stability::invalid, pitch = stability::invalid, roll = stability::invalid;
  bool valid = false;
};
RobotOrientation robot_orientation();
double robot_heading_deg();
double robot_pitch_deg();
double robot_roll_deg();
