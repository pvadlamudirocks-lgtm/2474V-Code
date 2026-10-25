#include "robot_imu.hpp"
#include "navigation.hpp"

RobotOrientation robot_orientation() {
  RobotOrientation result;
  if (!chassis.imu.is_installed() || chassis.imu.get_status() == pros::ImuStatus::error ||
      chassis.imu.is_calibrating()) return result;
  const double rotation = chassis.imu.get_rotation();
  const double pitch = chassis.imu.get_pitch(), roll = chassis.imu.get_roll();
  if (rotation == PROS_ERR_F || pitch == PROS_ERR_F || roll == PROS_ERR_F ||
      !stability::finite(rotation)) return result;
  result.pitch = stability::transform(pitch, robot_imu_config::IMU_PITCH_SIGN);
  result.roll = stability::transform(roll, robot_imu_config::IMU_ROLL_SIGN);
  result.heading = nav_pose_get().theta;
  result.valid = stability::finite(result.pitch) && stability::finite(result.roll) &&
                 stability::finite(result.heading);
  if (!result.valid) return {};
  return result;
}
double robot_heading_deg() { return robot_orientation().heading; }
double robot_pitch_deg() { return robot_orientation().pitch; }
double robot_roll_deg() { return robot_orientation().roll; }
