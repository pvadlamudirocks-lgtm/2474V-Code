#include "main.h"

// --- HARDWARE INITIALIZATION ---

// EZ-Template Chassis Definition
ez::Drive chassis(
    {1, -4},         // Left drive: L1 port 1, L2 reversed port 4.
    {3, -2},         // Right drive: R1 port 3, R2 reversed port 2.
    7,               // IMU port
    2.75,            // Wheel diameter (inches)
    600,             // Motor cartridge RPM
    600.0 / 450.0);  // External gear ratio

// Dual 11W Lift Motors (Ports 5 and 6; Port 6 reversed)
pros::MotorGroup liftMotors({5, -6}, pros::v5::MotorGears::green);

// Dual 5.5W Claw Motors (Port 8 clockwise, Port 9 anti-clockwise/reversed)
pros::MotorGroup clawMotors({8, -9}, pros::v5::MotorGears::green);

// Master Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// --- DRIVER SETTINGS ---
constexpr double TURN_MULTIPLIER = 1.30;
constexpr double DRIVER_ACTIVE_BRAKE_KP = 2.0;
constexpr int DRIVER_DEADZONE = 5;

// --- HELPER FUNCTIONS ---

int apply_driver_deadzone(int input) {
    if (input >= -DRIVER_DEADZONE && input <= DRIVER_DEADZONE)
        return 0;
    return input;
}

int clamp_driver_output(double output) {
    if (output > 127.0) return 127;
    if (output < -127.0) return -127;
    return static_cast<int>(output);
}

void driver_arcade_control(int forwardInput, int turnInput) {
    int forward = apply_driver_deadzone(forwardInput);
    int turn = apply_driver_deadzone(turnInput);
    int amplifiedTurn = clamp_driver_output(turn * TURN_MULTIPLIER);

    int leftOutput = clamp_driver_output(forward + amplifiedTurn);
    int rightOutput = clamp_driver_output(forward - amplifiedTurn);

    chassis.opcontrol_joystick_threshold_iterate(leftOutput, rightOutput);
}

void control_lift() {
    // L2 raises the lift up, R2 lowers the lift down
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        liftMotors.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        liftMotors.move(-127);
    } else {
        liftMotors.move(0);
    }
}

void control_claw() {
    // L1 runs the claw inward/forward, R1 runs the claw outward/reverse
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        clawMotors.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        clawMotors.move(-127);
    } else {
        clawMotors.move(0);
    }
}

// --- PROS LIFECYCLE FUNCTIONS ---

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "INITIALIZING SYSTEM");

    chassis.opcontrol_curve_buttons_toggle(false);
    chassis.opcontrol_curve_default_set(0.0, 0.0);
    chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);
    chassis.initialize();

    // Apply motor brake modes so mechanisms hold position when stopped
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    pros::lcd::set_text(0, "SYSTEM READY");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
    pros::lcd::set_text(1, "OPCONTROL STARTED");

    while (true) {
        // Handle Arcade Drivetrain
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        driver_arcade_control(leftY, rightX);

        // Handle Lift Controls (L2 / R2)
        control_lift();

        // Handle Claw Controls (L1 / R1)
        control_claw();

        // Diagnostics
        pros::lcd::print(2, "LEFT Y:  %4d", leftY);
        pros::lcd::print(3, "RIGHT X: %4d", rightX);
        pros::lcd::print(4, "LIFT & CLAW ACTIVE");

        pros::delay(ez::util::DELAY_TIME);
    }
}
