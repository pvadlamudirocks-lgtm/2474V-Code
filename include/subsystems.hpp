#pragma once
#include "api.h"

// All wiring and calibration choices live here. Validation is documented per item.
namespace mechanism_config {
// ----- Wiring and physical directions -----
// Change assignments here after checking the robot, rather than in route functions.
// Legacy distance_correction.cpp is excluded; its P11/P12 placeholders are obsolete.
constexpr int LEFT_LIFT_PORT = 14; // Left lift motor: red cartridge, 100 RPM.
constexpr int RIGHT_LIFT_PORT = 15; // Right lift motor: red cartridge, 100 RPM.
constexpr int LIFT_ROTATION_PORT = 16;
// Confirmed build specification: red cartridges use 36:1 internal gearing, 100 RPM.
// Confirmed external gearing is 1:1: one output revolution per motor revolution.
// Documentation only: manual control uses voltage, not a requested output RPM.
constexpr double LIFT_MOTOR_RPM = 100.0;
constexpr double LIFT_SPEED_UP_RATIO = 1.0; // Output speed / motor speed; no scaling.
constexpr double LIFT_OUTPUT_RPM = LIFT_MOTOR_RPM; // 100 RPM nominal at 1:1, not measured.
// Keep both PROS lift motors configured as MotorGears::red (36:1, 100 RPM).
constexpr int ARM_MOTOR_PORT = 17;
constexpr int ARM_ROTATION_PORT = 0; // Retired P18 sensor remains unbound.
constexpr int CLAW_DISTANCE_PORT = 0; // Retired P19 sensor remains unbound.
// Downward sensor: builders must supply an unused Smart Port. 0 disables it.
constexpr int LIFT_HEIGHT_DISTANCE_PORT = 0;
// Set true ONLY after verifying the beam stays vertically downward through travel.
constexpr bool LIFT_HEIGHT_SENSOR_MOUNTED_VERTICALLY = false;
// Optional FIXED angle; leave unconfigured for unknown or moving claw geometry.
constexpr bool LIFT_HEIGHT_SENSOR_FIXED_ANGLE_CONFIGURED = false;
constexpr double LIFT_HEIGHT_SENSOR_ANGLE_FROM_VERTICAL_DEG = __builtin_nan("");
// Signed vertical offset: claw reference height MINUS sensor-face height.
constexpr bool LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_CONFIGURED = false;
constexpr double LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_IN = __builtin_nan("");
// Optional measured beam-distance bounds, not guessed lift limits.
constexpr bool LIFT_HEIGHT_DISTANCE_BOUNDS_CONFIGURED = false;
constexpr double LIFT_HEIGHT_DISTANCE_MIN_MM = __builtin_nan("");
constexpr double LIFT_HEIGHT_DISTANCE_MAX_MM = __builtin_nan("");
// New claw-area hardware: 0 is UNBOUND, never passed to a device constructor.
constexpr bool CLAW_PNEUMATIC_CONFIGURED = true;
constexpr int CLAW_PNEUMATIC_PORT = 'A'; // Verified claw solenoid; polarity not yet verified.
constexpr bool FLEX_CONFIGURED = true;
constexpr int FLEX_LEFT_PORT = 11;  // Claw flex motor moved from port 8.
constexpr int FLEX_RIGHT_PORT = 12; // Claw flex motor moved from port 10.
// Mirrored shafts: same semantic command, opposite physical shaft directions.
constexpr bool FLEX_LEFT_REVERSED = false; // Direction test: port 8 not reversed.
constexpr bool FLEX_RIGHT_REVERSED = true; // Preserve the direction used on port 9.
constexpr int FLEX_INTAKE_SIGN = 1; // UNVERIFIED: flip after physical intake test.
constexpr int FLEX_INTAKE_POWER = 0; // Uncommissioned; choose power during testing.
constexpr int FLEX_OUTTAKE_POWER = 0; // Uncommissioned; no tuned speed assumed.
// Current toggle pair is P6/P19; direction settings apply to every toggle command.
constexpr bool TOGGLE_CONFIGURED = true;
constexpr int TOGGLE_PORT = 19;
constexpr int TOGGLE_OTHER_PORT = 6;
constexpr bool TOGGLE_OTHER_REVERSED = false; // Opposite shaft direction to P19.
constexpr bool TOGGLE_REVERSED = true; // Preserve P19's existing direction.
constexpr int TOGGLE_DIRECTION = -1; // Reverse overall direction: requested counterclockwise.
constexpr int TOGGLE_POWER = 127; // Full command; cartridge remains existing green configuration.
constexpr char CLAW_FLIP_ADI_PORT = 'B';  // TODO CONFIRM wiring.
constexpr char LIFT_HOME_SWITCH_ADI_PORT = 'C'; // Teammate-operated command switch.
// TODO assign the REAL bottom switch port ('A'-'H'). 0 means unbound;
// no DigitalIn is constructed and automatic return is rejected while unbound.
constexpr int LIFT_BOTTOM_LIMIT_PORT = 0;
constexpr bool LIFT_BOTTOM_LIMIT_PRESSED_VALUE = true; // Verify actual wiring.
// TODO measure worst-case full descent at the existing 1500 mV, then choose
// a finite timeout with margin. Zero disables return; no guessed travel time.
constexpr unsigned LIFT_RETURN_TIMEOUT_MS = 0;
// TODO bind the Override color mechanism; unrelated to the existing claw flip.
// No device type/port is assumed. Implement color_toggle_set() when selected.
constexpr bool CLAW_OPEN_OUTPUT = false; // User-tested mapping: OFF opens, ON closes; retest after change.
constexpr bool FLIP_REVERSE_OUTPUT = true; // false output should be normal.
// Driver confirmed UP with port14 positive / port15 negative in the raw test.
constexpr bool LEFT_LIFT_REVERSED = false;
constexpr bool RIGHT_LIFT_REVERSED = true;
constexpr bool ARM_MOTOR_REVERSED = false;
constexpr bool LIFT_ROTATION_REVERSED = false;
constexpr bool ARM_ROTATION_REVERSED = false;
// Confirmed: sensor is on the actual lift/output axle. Only centidegrees ->
// degrees conversion applies; no extra motor-to-output scaling at 1:1.
constexpr double LIFT_SENSOR_DEG_PER_MECHANISM_DEG = 1.0;
constexpr double ARM_SENSOR_DEG_PER_MECHANISM_DEG = 1.0; // TODO VERIFY coupling.

// ----- Calibration and enable switches -----
// These gates separate supported manual tests from unqualified automatic movement.
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
constexpr int LIFT_MANUAL_MV = 1500; // Keep low for the lift direction test; external gearing is 1:1.
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

enum class LiftMode { Disabled, Manual, Hold, MoveToTarget, Fault, ReturningHome,
                      DriverFull, AutonFull };
enum class ArmMode { Disabled, Manual, Hold, MoveToTarget, Fault };

// Single-owner API: commands and updates must run from the SAME task.
// No hidden task, delays, controller mappings, or automatic startup homing.
void lift_initialize();
void arm_initialize();
void mechanisms_initialize();
// Physically place/support at known home FIRST. Zeroing leaves motors disabled.
bool lift_zero_at_current_position();
bool arm_zero_at_current_position();
// Refresh every loop while held; manual cannot interrupt ReturningHome.
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
// Nonblocking switch-based return. Requires bound switch, measured timeout,
// and enabled claw hardware. Keep lift_update()/mechanisms_update() serviced.
bool lift_return_home_start();
bool lift_return_home_active();
enum class LiftBottomLimit { Unconfigured, Released, Pressed, Fault };
LiftBottomLimit lift_bottom_limit_read(); // Never substitutes a fake reading.
bool color_toggle_set(bool held); // Compatibility alias for toggle run/stop.
bool arm_set_target(double position_deg);
bool arm_move_home();
bool arm_move_score();
bool arm_move_max_lift_position();
void lift_update();
// Open-loop commissioning commands: latched until explicit stop/change or mode
// shutdown. No guessed travel timeout, target or automatic background task.
bool lift_auton_up_full();
bool lift_auton_down_full();
bool lift_auton_stop(); // Always attempts zero on BOTH motors, even while disabled.
// Sequential, autonomous-only. Caller supplies a MEASURED timeout (no default).
// Uses the existing controller/gates; stops output on success AND failure.
bool lift_auton_move_to_deg(double target_deg, unsigned timeout_ms);
// STOP removes motor output on success; HOLD keeps the same target under sensor control.
// HOLD requires validation and continued mechanism updates after this blocking call.
enum class LiftFinish { STOP, HOLD_CURRENT_TARGET };
bool lift_auton_move_to_deg(double target_deg, unsigned timeout_ms, LiftFinish finish);
bool lift_target_ready();
// Select a configured HOME/CLEARANCE/SCORE/MAX_SAFE entry from lift_calibration.hpp.
bool lift_move_calibrated_position(unsigned preset, unsigned timeout_ms, LiftFinish finish = LiftFinish::STOP);
bool toggle_connected();
bool mechanisms_preflight(bool require_lift, bool require_toggle, bool require_flex);
bool arm_auton_move_to_deg(double target_deg, unsigned timeout_ms);
void lift_print_debug(); // P16 validity, owner/mode, target, reference and faults.
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
// Switch-based descent, STOP, then close claw; arm stays put.
bool mechanism_return_home();
bool stop_all_mechanisms(); // Best-effort motor stop/cancel; no pneumatic changes.
void mechanisms_print_debug(); // Throttled terminal output; preserves odom screen.
// Isolated, nonblocking diagnostics; never run automatically. No button mappings.
// Call manual tests every loop with held inputs, then mechanisms_update().
// Both held commands zero manual voltage; release captures hold once.
void test_lift_manual(bool up_held, bool down_held);
// TEMPORARY: hold L1=14 +12000/15 -12000 electrical mV; hold L2=opposite.
// Compensates existing object reversals without changing motor configuration.
// Repeatable, no time limit while held. Release or both held zeros BOTH motors.
bool test_lift_motor_response(bool up_held, bool down_held);
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

// Nonblocking, single-owner commands. Availability means valid configuration,
// not proof of connected hardware. bool commands report PROS write success.
// Start commands reject disabled mode; stops remain available while disabled.
bool claw_pneumatic_available();
bool flex_available();
bool toggle_available();
// Power is a magnitude [0,127]; invalid values stop and return false.
bool flex_intake(int power = mechanism_config::FLEX_INTAKE_POWER);
bool flex_outtake(int power = mechanism_config::FLEX_OUTTAKE_POWER);
bool flex_stop();
bool toggle_run();
bool toggle_run_reverse(); // Opposite the configured normal direction; shared hardware.
bool toggle_stop();
bool claw_grab(int power = mechanism_config::FLEX_INTAKE_POWER); // Close + intake.
bool claw_score(int power = mechanism_config::FLEX_OUTTAKE_POWER); // Open + outtake.
// Driver: R1 held = intake(127), released = intake(13); R2 = same claw toggle + flex stop.
void claw_driver_control(bool flex_held, bool claw_new_press);
// Combined actions require a chosen power 1..127; uncommissioned default 0
// rejects safely rather than claiming to collect with stationary wheels.
bool pickup_prepare(int power = mechanism_config::FLEX_INTAKE_POWER); // Open + intake.
bool pickup_capture(int power = mechanism_config::FLEX_INTAKE_POWER); // Close + intake.
bool pickup_finish();  // Stop flex; keep pneumatic state.
