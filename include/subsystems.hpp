#pragma once

#include "api.h"

// Mechanism declarations only. Motor and piston definitions live in subsystems.cpp.

// Runs intake inward and stops outtake.
void intake_on();

// Runs outtake outward and stops intake.
void outtake_on();

// Stops both intake and outtake motors.
void intake_stop();

// Toggles the clamp pneumatic between extended and retracted.
void clamp_toggle();

// Sets the clamp pneumatic to a specific state.
void clamp_set(bool state);

// Toggles the descore pneumatic between extended and retracted.
void descore_toggle();

// Sets the descore pneumatic to a specific state.
void descore_set(bool state);

// Runs the scoring motor forward.
void scoring_on();

// Runs the scoring motor in reverse.
void scoring_reverse();

// Stops the scoring motor.
void scoring_stop();

// Raises the lift.
void lift_up();

// Lowers the lift.
void lift_down();

// Stops the lift motor.
void lift_stop();

// Stops all hold-style mechanism motors.
void stop_all_mechanisms();
