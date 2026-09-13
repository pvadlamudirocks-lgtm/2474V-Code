#include "main.h"

// EZ-Template owns all four drivetrain motors through this chassis object.
// Negative ports reverse the two motors proven backward in physical testing.
ez::Drive chassis(
    {1, -4},         // Left drive: L1 port 1, L2 reversed port 4.
    {3, -2},         // Right drive: R1 port 3, R2 reversed port 2.
    7,               // Existing IMU port; confirm before autonomous testing.
    2.75,            // Drive-wheel diameter in inches.
    600,             // Blue motor cartridge speed in RPM.
    600.0 / 450.0);  // External ratio for 450 RPM effective wheel speed.

// Pneumatic Solenoids on ADI Ports A and B
pros::adi::DigitalOut clawSolenoidA('A');
pros::adi::DigitalOut rotateSolenoidB('B');

// The robot uses one master controller for all driver input.
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Compile-time driver-feel settings.
constexpr double TURN_MULTIPLIER = 1.30;
constexpr double DRIVER_ACTIVE_BRAKE_KP = 2.0;
constexpr int DRIVER_DEADZONE = 5;

// pneumatic Latch States
bool clawOpen = false;
bool roated = false;
bool l1Pressed = false;
bool r1Pressed = false;

// Optional driver response profiles. Disabled in favor of fixed settings.
// enum class DriveProfile {
//   SOFT,
//   NORMAL,
//   AGGRESSIVE
// };
//
// constexpr double SOFT_CURVE = 6.0;
// constexpr double NORMAL_CURVE = 3.0;
// constexpr double AGGRESSIVE_CURVE = 0.0;
//
// DriveProfile driveProfile = DriveProfile::NORMAL;
//
// void apply_drive_profile(DriveProfile profile) {
//   driveProfile = profile;
//
//   double curve = NORMAL_CURVE;
//   const char* label = "DRIVE: NORMAL";
//
//   if (profile == DriveProfile::SOFT) {
//     curve = SOFT_CURVE;
//     label = "DRIVE: SOFT";
//   } else if (profile == DriveProfile::AGGRESSIVE) {
//     curve = AGGRESSIVE_CURVE;
//     label = "DRIVE: AGGRESSIVE";
//   }
//
//   chassis.opcontrol_curve_default_set(curve, curve);
//   pros::lcd::set_text(1, label);
// }
//
// void select_more_aggressive_profile() {
//   if (driveProfile == DriveProfile::SOFT)
//     apply_drive_profile(DriveProfile::NORMAL);
//   else
//     apply_drive_profile(DriveProfile::AGGRESSIVE);
// }
//
// void select_softer_profile() {
//   if (driveProfile == DriveProfile::AGGRESSIVE)
//     apply_drive_profile(DriveProfile::NORMAL);
//   else
//     apply_drive_profile(DriveProfile::SOFT);
// }

int apply_driver_deadzone(int input) {
  // Ignore small controller noise so the drive stays stopped at neutral.
  if (input >= -DRIVER_DEADZONE && input <= DRIVER_DEADZONE)
    return 0;
  return input;
}

int clamp_driver_output(double output) {
  // PROS motor power commands must remain between -127 and 127.
  if (output > 127.0)
    return 127;
  if (output < -127.0)
    return -127;
  return static_cast<int>(output);
}

void driver_arcade_control(int forwardInput, int turnInput) {
  // Left Y remains linear; only Right X receives extra turn response.
  int forward = apply_driver_deadzone(forwardInput);
  int turn = apply_driver_deadzone(turnInput);
  int amplifiedTurn = clamp_driver_output(turn * TURN_MULTIPLIER);

  // Split arcade mixing converts forward and turn into left/right power.
  int leftOutput = clamp_driver_output(forward + amplifiedTurn);
  int rightOutput = clamp_driver_output(forward - amplifiedTurn);

  // Let EZ apply its driver active brake whenever both outputs reach zero.
  chassis.opcontrol_joystick_threshold_iterate(leftOutput, rightOutput);
}

void control_pneumatics() {
  // Claw Open/Close Toggle (Button L1)
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    if (!l1Pressed) {
      clawOpen = !clawOpen;
      clawSolenoidA.set_value(clawOpen);
      l1Pressed = true;
    }
  } else {
    l1Pressed = false;
  }

  // Claw Roation Toggle (Button R1)
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    if (!r1Pressed) {
      roated = !roated;
      rotateSolenoidB.set_value(roated);
      r1Pressed = true;
    }
  } else {
    r1Pressed = false;
  }
}

//-- PROS LIFECYCLE FUNCTIONS -----------------------------------------------------------//

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "INITIALIZING EZ DRIVE");

  // Disable EZ curve softening; turn gain is applied by driver_arcade_control.
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  // Driver-only active brake resists rolling after the sticks return to zero.
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);
  // Profile-mode initialization (disabled).
  // chassis.opcontrol_curve_default_set(NORMAL_CURVE, NORMAL_CURVE);
  chassis.initialize();

  // Set Inital pneumatic states to retracted/closed
  clawSolenoidA.set_value(false);
  rotateSolenoidB.set_value(false);

  pros::lcd::set_text(0, "EZ DRIVE READY");
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
}

// PID and autonomous controller shortcuts stay disabled for this drive test.
void ez_template_extras() {
}

void opcontrol() {
  // BRAKE stops more firmly than COAST without locking the wheels like HOLD.
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  pros::lcd::set_text(1, "OPCONTROL STARTED");

  // apply_drive_profile(DriveProfile::NORMAL);  // Disabled profile startup.

  while (true) {
    // Disabled profile controls:
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    //   select_more_aggressive_profile();
    // else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    //   select_softer_profile();

    ez_template_extras();

    // Split arcade: Left Y drives forward/backward; Right X turns.
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    driver_arcade_control(leftY, rightX);

    // handle pneumatic controls
    control_pneumatics();

    // Diagnostics
    // Keep raw joystick values visible during physical testing.
    pros::lcd::print(2, "LEFT Y:  %4d", leftY);
    pros::lcd::print(3, "RIGHT X: %4d", rightX);
    pros::lcd::print(4, "CLAW: %s | ROTATION: %s", clawOpen ? "OPEN" : "CLOSED", roated ? "ROTATED" : "UNROTATED");


    pros::delay(ez::util::DELAY_TIME);
  }
}
