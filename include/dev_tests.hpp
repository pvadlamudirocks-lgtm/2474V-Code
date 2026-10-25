#pragma once

enum class DevTest {
  SENSOR_HEALTH, START_POSE, MANUAL_ODOM, DRIVE_FORWARD_12, TURN_90,
  GOTO_Y_12, GOTO_X_12, REVERSE_12, RETURN_TO_ORIGIN, TWO_POINT,
  SQUARE, GOTO_POSE, CHAIN_TWO_MOVE, PURE_PURSUIT_BASIC, MIRROR_MATH, STABILITY_MONITOR
};

// Enable only after physically verifying foundation tests.
inline constexpr bool ENABLE_ADVANCED_DEV_TESTS = false;
void run_dev_test(DevTest test);
