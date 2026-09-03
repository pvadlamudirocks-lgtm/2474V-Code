#include "subsystems.hpp"

namespace {
// TODO: These mechanism names, motor ports, reversals, and cartridge choices
// are placeholders until the Override robot is built and wired. These avoid
// the current placeholder distance sensor ports 8 through 13.
pros::Motor intake_motor(14, pros::v5::MotorGears::green);
pros::Motor outtake_motor(-15, pros::v5::MotorGears::green);
pros::Motor lift_motor(16, pros::v5::MotorGears::red);
pros::Motor scoring_motor(17, pros::v5::MotorGears::green);

// TODO: These ADI pneumatic ports are placeholders until pneumatics are wired.
pros::adi::DigitalOut clamp_piston('A');
pros::adi::DigitalOut descore_piston('B');

bool clamp_extended = false;
bool descore_extended = false;

constexpr int kMechanismSpeed = 127;
}  // namespace

// Runs intake inward and ensures outtake is off.
void intake_on() {
  intake_motor.move(kMechanismSpeed);
  outtake_motor.move(0);
}

// Runs outtake outward and ensures intake is off.
void outtake_on() {
  intake_motor.move(0);
  outtake_motor.move(kMechanismSpeed);
}

// Stops the intake/outtake pair.
void intake_stop() {
  intake_motor.move(0);
  outtake_motor.move(0);
}

// Toggles the clamp pneumatic state.
void clamp_toggle() {
  clamp_set(!clamp_extended);
}

// Sets the clamp pneumatic state directly.
void clamp_set(bool state) {
  clamp_extended = state;
  clamp_piston.set_value(clamp_extended);
}

// Toggles the descore pneumatic state.
void descore_toggle() {
  descore_set(!descore_extended);
}

// Sets the descore pneumatic state directly.
void descore_set(bool state) {
  descore_extended = state;
  descore_piston.set_value(descore_extended);
}

// Runs the scoring motor forward.
void scoring_on() {
  scoring_motor.move(kMechanismSpeed);
}

// Runs the scoring motor backward.
void scoring_reverse() {
  scoring_motor.move(-kMechanismSpeed);
}

// Stops the scoring motor.
void scoring_stop() {
  scoring_motor.move(0);
}

// Raises the lift.
void lift_up() {
  lift_motor.move(kMechanismSpeed);
}

// Lowers the lift.
void lift_down() {
  lift_motor.move(-kMechanismSpeed);
}

// Stops the lift motor.
void lift_stop() {
  lift_motor.move(0);
}

// Stops every hold-style mechanism motor.
void stop_all_mechanisms() {
  intake_stop();
  scoring_stop();
  lift_stop();
}
