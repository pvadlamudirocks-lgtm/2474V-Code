#include "pose_recorder.hpp"
#include "navigation.hpp"
#include "nav_diagnostics.hpp"
#include <cstdio>
namespace pose_recorder {
// ----- Driver pose recording -----
// Called from opcontrol; button edges capture once without changing joystick control.
// Feedback stays in the terminal; telemetry owns all three controller rows.
void update(pros::Controller& c) {
  if (!enabled || pros::competition::is_disabled() || pros::competition::is_autonomous()) return;
  static unsigned index = 0;
  static bool reset_held = false, start_held = false;
  const bool reset = c.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && c.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
  const bool initialize = c.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && c.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
  // LEFT+RIGHT clears numbering only; the robot's odometry reference stays unchanged.
  if (reset && !reset_held) {
    index = 0;
    std::printf("POSE RECORD RESET (counter only; odometry unchanged)\n");
  }
  // DOWN+Y deliberately applies the configured start, after checking that motion is idle.
  if (initialize && !start_held) {
    const bool sticks_idle = c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) == 0 &&
                             c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) == 0;
    const bool lift_idle = !c.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
                           !c.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool at_rest = !chassis.left_motors.empty() && !chassis.right_motors.empty();
    for (const auto* motors : {&chassis.left_motors, &chassis.right_motors})
      for (const auto& m : *motors) {
        const double rpm = m.get_actual_velocity();
        if (!std::isfinite(rpm) || std::fabs(rpm) >= 3) at_rest = false;
      }
    if (start_configured && auton_math::finite(start) && sticks_idle && lift_idle && at_rest && nav_print_sensor_health()) {
      nav_set_start(start.x,start.y,start.h);
      std::printf("POSE START INITIALIZED %.3f %.3f %.3f; verify readback and physical alignment\n",start.x,start.y,start.h);
    } else {
      std::printf("POSE START REJECTED: configure reference; align and stop robot; verify sensors\n");
    }
  }
  reset_held = reset; start_held = initialize;
  // A records the current pose for copying into measured route configuration later.
  if (c.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !reset && !initialize) {
    const auto p = nav_pose_get();
    if (!auton_math::finite({p.x,p.y,p.theta}) || !nav_print_sensor_health()) {
      std::printf("POSE SENSOR FAULT\n"); return;
    }
    ++index;
    std::printf("POSE CAPTURE %u\nX=%.2f\nY=%.2f\nH=%.2f\n{%.2f, %.2f, %.2f}\n",
                index,p.x,p.y,p.theta,p.x,p.y,p.theta);
  }
}
}
