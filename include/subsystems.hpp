#pragma once
#include "api.h"

// All wiring and calibration choices live here. NOTHING is physically validated.
namespace mechanism_config {
// Avoid 11-13: distance_correction.cpp already declares placeholders there.
constexpr int LEFT_LIFT_PORT = 14;
constexpr int RIGHT_LIFT_PORT = 15;
constexpr int LIFT_ROTATION_PORT = 16;
// Confirmed build specification: green cartridges remain 200 RPM motors.
// External gearing turns the lift output twice per motor revolution (2:1 speed-up).
// Documentation only: manual control uses voltage, not a requested output RPM.
constexpr double LIFT_MOTOR_RPM = 200.0;
constexpr double LIFT_SPEED_UP_RATIO = 2.0; // Output speed / motor speed.
constexpr double LIFT_OUTPUT_RPM = 400.0; // Approximate nominal output, not measured.
// Keep both PROS lift motors configured as MotorGears::green, NOT a 400 RPM cartridge.
constexpr int ARM_MOTOR_PORT = 17;
constexpr int ARM_ROTATION_PORT = 18;
constexpr int CLAW_DISTANCE_PORT = 19;
constexpr char CLAW_ADI_PORT = 'A';       // TODO CONFIRM wiring.
constexpr char CLAW_FLIP_ADI_PORT = 'B';  // TODO CONFIRM wiring.
constexpr char LIFT_HOME_SWITCH_ADI_PORT = 'C'; // Teammate-operated command switch.
constexpr bool CLAW_OPEN_OUTPUT = true; // TODO VERIFY pneumatic polarity.
constexpr bool FLIP_REVERSE_OUTPUT = true; // false output should be normal.
// TODO VERIFY PHYSICALLY: BOTH linked motors must drive the axle the same way.
constexpr bool LEFT_LIFT_REVERSED = false;
constexpr bool RIGHT_LIFT_REVERSED = false;
constexpr bool ARM_MOTOR_REVERSED = false;
constexpr bool LIFT_ROTATION_REVERSED = false;
constexpr bool ARM_ROTATION_REVERSED = false;
// TODO VERIFY actual Rotation Sensor mounting: 1.0 assumes direct final-output
// shaft coupling. External motor gearing alone does not change this sensor ratio.
// If mounted before the gearing, measure sensor degrees per lift-output degree
// and update this ratio before enabling feedback; mounting is not yet confirmed.
constexpr double LIFT_SENSOR_DEG_PER_MECHANISM_DEG = 1.0;
constexpr double ARM_SENSOR_DEG_PER_MECHANISM_DEG = 1.0; // TODO VERIFY coupling.

// Commissioning gates. Enable manual tests with mechanisms physically supported.
constexpr bool ENABLE_LIFT_MANUAL = true;
constexpr bool ENABLE_ARM_MANUAL = false;
// Enable only after lift calibration and automatic movement are validated.
constexpr bool ENABLE_LIFT_HOME_SWITCH_COMMAND = false;
constexpr unsigned LIFT_HOME_REQUIRED_PRESSES = 6;
constexpr unsigned LIFT_HOME_SWITCH_DEBOUNCE_MS = 30;
constexpr unsigned LIFT_HOME_PRESS_SEQUENCE_TIMEOUT_MS = 10000;
// Enable feedback ONLY after positive motor output increases sensor position.
constexpr bool LIFT_FEEDBACK_VERIFIED = false;
constexpr bool ARM_FEEDBACK_VERIFIED = false;
constexpr bool ENABLE_PNEUMATICS = false;
// TODO VERIFY claw-close/lift-lower overlap and arm/lift collision clearance.
constexpr bool POST_SCORE_RESET_VERIFIED = false;

// TODO PHYSICAL CALIBRATION REQUIRED. Zero maxima disable automatic movement.
constexpr bool LIFT_LIMITS_CALIBRATED = false;
constexpr bool ARM_LIMITS_CALIBRATED = false;
constexpr double LIFT_MIN_DEG = 0.0;
constexpr double LIFT_MAX_DEG = 0.0; // NOT VALIDATED: replace with safe maximum.
constexpr double ARM_MIN_DEG = 0.0;
constexpr double ARM_MAX_DEG = 0.0;  // NOT VALIDATED: replace with safe maximum.
constexpr double LIFT_HOME_DEG = 0.0;
constexpr double LIFT_LOW_DEG = 0.0;  // TODO CALIBRATE; disabled placeholder.
constexpr double LIFT_MID_DEG = 0.0;  // TODO CALIBRATE.
constexpr double LIFT_HIGH_DEG = 0.0; // TODO CALIBRATE.
constexpr bool LIFT_SCORING_POSITIONS_CALIBRATED = false;
constexpr double ARM_HOME_DEG = 0.0;
constexpr double ARM_SCORE_DEG = 90.0;     // Intended angle, NOT VALIDATED.
constexpr double ARM_MAX_LIFT_DEG = 135.0; // Intended angle, NOT VALIDATED.
constexpr bool ARM_SCORING_POSITIONS_CALIBRATED = false;

// NOT TUNED. Voltage = KP * angle error - KD * measured angular velocity.
// KP: mV/degree. KD: mV/(degree/second). No integral or gravity feedforward.
// P-only hold can sag under load. These low voltage caps may not lift the load.
constexpr double LIFT_HOLD_KP = 80.0;
constexpr double LIFT_HOLD_KD = 0.0;
constexpr double ARM_HOLD_KP = 60.0;
constexpr double ARM_HOLD_KD = 0.0;
constexpr int LIFT_MANUAL_MV = 1500; // Keep low for first direction test with 2:1 speed-up.
constexpr int ARM_MANUAL_MV = 1500;
constexpr int LIFT_CLOSED_LOOP_MAX_MV = 2000;
constexpr int ARM_CLOSED_LOOP_MAX_MV = 2000;
constexpr double LIFT_TARGET_TOLERANCE_DEG = 2.0; // TODO TUNE.
constexpr double ARM_TARGET_TOLERANCE_DEG = 2.0;  // TODO TUNE.
constexpr unsigned UPDATE_PERIOD_MS = 10;
constexpr unsigned MANUAL_COMMAND_TIMEOUT_MS = 100;
constexpr unsigned MAX_UPDATE_GAP_MS = 100;
constexpr unsigned TARGET_SETTLE_MS = 200;
constexpr unsigned TARGET_TIMEOUT_MS = 5000; // TODO VERIFY real travel time.
constexpr unsigned DEBUG_PERIOD_MS = 500;

constexpr bool ENABLE_CLAW_DISTANCE_AUTO_ARM = false;
constexpr bool CLAW_DISTANCE_CALIBRATED = false;
constexpr int CLAW_DETECT_MIN_MM = 0; // TODO CALIBRATE; invalid window by default.
constexpr int CLAW_DETECT_MAX_MM = 0;
constexpr int CLAW_DETECT_MIN_CONFIDENCE = 20; // TODO VALIDATE.
constexpr unsigned CLAW_DETECT_CONSISTENT_READINGS = 3; // TODO VALIDATE.
constexpr unsigned CLAW_DISTANCE_SAMPLE_MS = 20;
constexpr int DISTANCE_CONFIDENCE_AVAILABLE_ABOVE_MM = 200; // Installed PROS API.
}

enum class LiftMode { Disabled, Manual, Hold, MoveToTarget, Fault };
enum class ArmMode { Disabled, Manual, Hold, MoveToTarget, Fault };

// Single-owner API: commands and updates must run from the SAME task.
// No hidden task, delays, controller mappings, or automatic startup homing.
void lift_initialize();
void arm_initialize();
void mechanisms_initialize();
// Physically place/support at known home FIRST. Zeroing leaves motors disabled.
bool lift_zero_at_current_position();
bool arm_zero_at_current_position();
// Refresh every loop while held; manual always cancels automatic movement.
// Call stop_and_hold ONCE on release, not on every idle loop.
void lift_manual_up();
void lift_manual_down();
void lift_manual_stop_and_hold();
void arm_manual_up();
void arm_manual_down();
void arm_manual_stop_and_hold();
void lift_cancel_auto();
void arm_cancel_auto();
// Nonblocking. false = rejected; existing command is unchanged.
// Targets require home, verified feedback, valid limits and no manual command.
bool lift_set_target(double position_deg);
bool lift_move_home();
bool lift_move_low();
bool lift_move_mid();
bool lift_move_high();
bool lift_move_max();
bool arm_set_target(double position_deg);
bool arm_move_home();
bool arm_move_score();
bool arm_move_max_lift_position();
void lift_update();
void arm_update();
void mechanisms_update(); // Call every 10 ms, including during autonomous motion.
double lift_position_deg(); // NAN = invalid reading/configuration.
double arm_position_deg();
bool lift_at_target(); // In tolerance continuously for TARGET_SETTLE_MS.
bool arm_at_target();
LiftMode lift_mode();
ArmMode arm_mode();
// true = open / reverse. Return value reports command success, not cylinder motion.
bool claw_set(bool open);
bool claw_open();
bool claw_close();
bool claw_toggle();
bool claw_flip_set(bool reverse);
bool claw_flip_forward();
bool claw_flip_reverse();
bool claw_flip_toggle();
bool claw_object_detected(); // Cached, debounced, freshness-checked result.
// Close command, then lift home target. Arm stays put until clearance is known.
bool mechanism_return_home();
void stop_all_mechanisms(); // Zero voltage, cancel targets; does NOT hold or vent.
void mechanisms_print_debug(); // Throttled terminal output; preserves odom screen.
// Isolated, nonblocking diagnostics; never run automatically. No button mappings.
// Call manual tests every loop with held inputs, then mechanisms_update().
// Both held commands zero manual voltage; release captures hold once.
void test_lift_manual(bool up_held, bool down_held);
void test_arm_manual(bool up_held, bool down_held);
void test_lift_rotation_sensor(); // Read/print only.
void test_arm_rotation_sensor();
bool test_lift_hold(); // Call ONCE to capture hold; keep updating afterward.
bool test_lift_target(double position_deg);
bool test_arm_target(double position_deg);
bool test_claw(bool open);
bool test_claw_flip(bool reverse);
// Compatibility with previous lift names. Obsolete unused placeholder APIs removed.
void lift_up();
void lift_down();
void lift_stop(); // Release to sensor hold; use stop_all_mechanisms for zero output.
