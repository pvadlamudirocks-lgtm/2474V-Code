
#include "subsystems.hpp"
#include "lift_calibration.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
using namespace mechanism_config;
// Temporary low-voltage commissioning cap for all open-loop lift commands.
// Restore higher power ONLY after the three-motor lift is physically qualified.

constexpr int LIFT_COMMISSIONING_MV = 12000;
static_assert(THIRD_LIFT_PORT >= 1 && THIRD_LIFT_PORT <= 21 &&
              THIRD_LIFT_PORT != LEFT_LIFT_PORT && THIRD_LIFT_PORT != RIGHT_LIFT_PORT &&
              THIRD_LIFT_PORT != LIFT_ROTATION_PORT && THIRD_LIFT_PORT != ARM_MOTOR_PORT &&
              THIRD_LIFT_PORT != FLEX_LEFT_PORT && THIRD_LIFT_PORT != FLEX_RIGHT_PORT &&
              THIRD_LIFT_PORT != TOGGLE_PORT && THIRD_LIFT_PORT != TOGGLE_OTHER_PORT &&
              THIRD_LIFT_PORT != 1 && THIRD_LIFT_PORT != 2 && THIRD_LIFT_PORT != 3 &&
              THIRD_LIFT_PORT != 4 && THIRD_LIFT_PORT != 5 && THIRD_LIFT_PORT != 7 &&
              THIRD_LIFT_PORT != 20,
              "Third lift port conflicts with an existing hardware assignment");
// Lazy construction: linking this file does not configure hardware during the
// existing odometry-only startup. A device is constructed only when requested.
pros::Motor& left_lift_motor() {  
  static pros::Motor motor(LEFT_LIFT_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}

pros::Motor& right_lift_motor() {
  static pros::Motor motor(RIGHT_LIFT_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}

pros::Motor& third_lift_motor() {
  static pros::Motor motor(THIRD_LIFT_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}

pros::Motor& arm_motor() {  
  static pros::Motor motor(ARM_MOTOR_REVERSED ? -ARM_MOTOR_PORT : ARM_MOTOR_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}
pros::Rotation& lift_sensor() {
  static pros::Rotation sensor(LIFT_ROTATION_REVERSED ? -LIFT_ROTATION_PORT : LIFT_ROTATION_PORT);
  return sensor;
}
pros::Rotation& arm_sensor() {
  static pros::Rotation sensor(ARM_ROTATION_REVERSED ? -ARM_ROTATION_PORT : ARM_ROTATION_PORT);
  return sensor;
}
pros::Distance& claw_distance() {
  static pros::Distance sensor(CLAW_DISTANCE_PORT);
  return sensor;
}
pros::adi::DigitalOut& claw_piston(bool initial_output) {
  static pros::adi::DigitalOut piston(CLAW_PNEUMATIC_PORT, initial_output);
  return piston;
}
pros::adi::DigitalOut& flip_piston() {
  static pros::adi::DigitalOut piston(CLAW_FLIP_ADI_PORT, !FLIP_REVERSE_OUTPUT);
  return piston;
}
pros::adi::DigitalIn& lift_home_switch() {
  // Installed pros/adi.hpp: DigitalIn(uint8_t) accepts 'A'-'H'; get_value()
  // reads the digital input. Construct only when this command is enabled.
  static pros::adi::DigitalIn input(LIFT_HOME_SWITCH_ADI_PORT);
  return input;
}

// ----- Home-command switch -----
// Count a deliberate press sequence; this switch requests home, not a sensor zero.
struct LiftHomeSwitchState {
  bool sampled = false;
  bool raw_pressed = false;
  bool stable_pressed = true; // Require a debounced release before first press.
  unsigned raw_changed_at = 0;
  unsigned sequence_started_at = 0;
  unsigned presses = 0;
};
LiftHomeSwitchState home_switch;

void update_lift_home_switch() {
  if (!ENABLE_LIFT_HOME_SWITCH_COMMAND) return;
  const unsigned now = pros::millis();
  if (home_switch.presses != 0 &&
      now - home_switch.sequence_started_at >= LIFT_HOME_PRESS_SEQUENCE_TIMEOUT_MS) {
    home_switch.presses = 0;
    std::printf("Lift home switch: sequence timed out\n");
  }

  const auto value = lift_home_switch().get_value();
  if (value != 0 && value != 1) {
    // Discard the sequence on an invalid read; require release after recovery.
    home_switch = LiftHomeSwitchState{};
    return;
  }
  const bool pressed = value == 1;
  if (!home_switch.sampled || pressed != home_switch.raw_pressed) {
    home_switch.sampled = true;
    home_switch.raw_pressed = pressed;
    home_switch.raw_changed_at = now;
  }
  if (now - home_switch.raw_changed_at < LIFT_HOME_SWITCH_DEBOUNCE_MS ||
      pressed == home_switch.stable_pressed) return;
  home_switch.stable_pressed = pressed;
  if (!pressed) return; // Only debounced released -> pressed edges count.

  if (home_switch.presses == 0) home_switch.sequence_started_at = now;
  ++home_switch.presses;
  std::printf("Lift home switch: %u/%u\n", home_switch.presses, LIFT_HOME_REQUIRED_PRESSES);
  if (home_switch.presses >= LIFT_HOME_REQUIRED_PRESSES) {
    // Go TO the calibrated home/intake position. Never redefine sensor zero.
    const bool accepted = lift_move_home();
    home_switch.presses = 0;
    std::printf(accepted ? "Lift home requested\n" :
                          "Lift home request rejected: lift not ready/calibrated or manual control active\n");
  }
}

// Lift and arm follow the same simple rules. A shared record prevents their
// safety behavior from drifting apart. Public functions below name each axis.
enum class AxisMode { Disabled, Manual, Hold, MoveToTarget, Fault };
struct AxisState {
  AxisMode mode = AxisMode::Disabled;
  bool initialized = false;
  bool homed = false;
  double target = 0.0;
  double previous_position = 0.0;
  unsigned last_update = 0;
  unsigned manual_refresh = 0;
  unsigned target_started = 0;
  unsigned target_timeout_ms = TARGET_TIMEOUT_MS;
  unsigned settled_since = 0;
  bool settling = false;
  bool settled = false;
  int manual_mv = 0;
};
AxisState lift;
AxisState arm;
bool returning_home = false;
// An explicitly invoked diagnostic owns output only while it is running.
bool lift_response_running = false;
// Only one lift mode may command all three motors at a time.
// Full-voltage driver/autonomous control must not compete with target or hold control.
enum class LiftOutputOwner { None, Driver, Autonomous };
LiftOutputOwner lift_output_owner = LiftOutputOwner::None;
bool lift_output_fault = false; // Write faults latch independently of sensor calibration.
int lift_full_direction = 0;
unsigned lift_full_last_refresh = 0;
bool flex_hold_active = false;
unsigned return_started_at = 0;
bool return_started_in_autonomous = false;
bool claw_is_open = false;
bool claw_raw_state = false; // Electrical output only; open/closed is unverified.
bool flip_is_reverse = false; // Known normal software state; no startup toggle.
bool claw_state_written = false;
bool flip_state_written = false;
bool object_detected = false;
unsigned object_readings = 0;
unsigned last_distance_sample = 0;

// In these small shared helpers, is_lift=true selects lift; false selects arm.
double read_position(bool is_lift) {
  // Never construct the retired arm sensor on an unbound port.
  if (!is_lift && (ARM_ROTATION_PORT < 1 || ARM_ROTATION_PORT > 21)) return NAN;
  const double ratio = is_lift ? LIFT_SENSOR_DEG_PER_MECHANISM_DEG : ARM_SENSOR_DEG_PER_MECHANISM_DEG;
  const auto raw = is_lift ? lift_sensor().get_position() : arm_sensor().get_position();
  if (raw == PROS_ERR || !std::isfinite(ratio) || ratio <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  // Installed PROS returns signed accumulated CENTIDEGREES, not degrees.
  // get_position does not wrap every revolution like get_angle does.
  return static_cast<double>(raw) / 100.0 / ratio;
}

// Shared lift output: positive command = UP. P13 uses P15's motor
// polarity and P15's -up_mv command; confirm actual shaft direction first.
bool write_lift_pair(int up_mv) {
    // Preserve calibrated lift limits when enabled.
  if (up_mv != 0 && lift_calibration::enable_limits) {
    if (!auton_math::lift_direction_allowed(
            lift_calibration::limits,
            read_position(true),
            up_mv > 0 ? 1 : -1))    
      up_mv = 0;

    if (!lift.homed)
      up_mv = 0;
  }
  // Check the physical ports using positive port numbers.
// Workaround for reversed-motor is_installed() bug in PROS 4.1.1.
if (up_mv != 0) {
  static pros::Motor p13_check(THIRD_LIFT_PORT, pros::v5::MotorGears::green);
  static pros::Motor p14_check(LEFT_LIFT_PORT, pros::v5::MotorGears::green);
  static pros::Motor p15_check(RIGHT_LIFT_PORT, pros::v5::MotorGears::green);

  if (!p13_check.is_installed() ||
      !p14_check.is_installed() ||
      !p15_check.is_installed()) {

    left_lift_motor().move_voltage(0);
    right_lift_motor().move_voltage(0);
    third_lift_motor().move_voltage(0);

    lift_output_fault = true;
    return false;
  }
}

  const auto left = left_lift_motor().move_voltage(
      (LEFT_LIFT_REVERSED ? -1 : 1) * up_mv);

  const auto right = right_lift_motor().move_voltage(
      (RIGHT_LIFT_REVERSED ? -1 : 1) * up_mv);

      

const auto third = third_lift_motor().move_voltage(
    (mechanism_config::THIRD_LIFT_REVERSED ? -1 : 1) * up_mv);

// All three motor commands must succeed.
if (left != PROS_ERR &&
    right != PROS_ERR &&
    third != PROS_ERR) {
    return true;
}

// A motor command failed.
// Attempt to stop ALL three lift motors.
left_lift_motor().move_voltage(0);
right_lift_motor().move_voltage(0);
third_lift_motor().move_voltage(0);

lift_output_fault = true;
return false;

}



bool write_voltage(bool is_lift, int mv) {
  if (!is_lift) {
    return arm_motor().move_voltage(mv) != PROS_ERR;
  }

  // Always permit a stop command.
  if (mv == 0) {
    return write_lift_pair(0);
  }

  // Do not move before establishing a valid home reference.
  if (!lift.initialized || !lift.homed) {
    return write_lift_pair(0);
  }

  // P16 must remain valid during powered movement.
  const double position = read_position(true);

  if (!std::isfinite(position)) {
    write_lift_pair(0);
    return false;
  }
  
  

// Unknown maximum allows upward movement ONLY
// during supervised standalone calibration.
// Once a maximum exists, it is enforced.
if (mv > 0) {
    const double max_safe = mechanism_config::LIFT_MAX_DEG;

    const bool max_known =
        std::isfinite(max_safe) && max_safe > 0.0;

    const bool calibration_driver =
        mechanism_config::ENABLE_LIFT_CALIBRATION_MODE &&
        lift_output_owner == LiftOutputOwner::Driver &&
        !pros::competition::is_connected();

    if ((max_known && position >= max_safe) ||
        (!max_known && !calibration_driver)) {
        return write_lift_pair(0);
    }
}


  if (mv < 0) {
    const auto bottom = lift_bottom_limit_read();

    if (bottom == LiftBottomLimit::Fault) {
      write_lift_pair(0);
      return false;
    }

    // Prevent lowering at or below the calibrated zero.
   
    if (bottom == LiftBottomLimit::Pressed ||
        position <= mechanism_config::LIFT_LOWER_STOP_MARGIN_DEG) {
        return write_lift_pair(0);
    }

  }

  return write_lift_pair(mv);
}


// Cancel the current axis command and attempt a stop; a fault also invalidates home.
bool disable_axis(AxisState& axis, bool is_lift, bool fault = false) {

if (is_lift) {
    lift_output_owner = LiftOutputOwner::None;
    lift_full_direction = 0;
    lift_full_last_refresh = 0;
}
  if (is_lift) lift_response_running = false; // Disable/stop must cancel the pulse.
  if (is_lift) returning_home = false;
  const bool stopped = !axis.initialized || write_voltage(is_lift, 0);
  fault = fault || !stopped;
  axis.mode = fault ? AxisMode::Fault : AxisMode::Disabled;
  axis.manual_mv = 0;
  axis.settling = false;
  axis.settled = false;
  // Faults latch. Recovery requires explicit initialize and physical re-zero.
  // Reconnecting a sensor must not resume an abandoned target.
  if (fault) axis.homed = false;
  return stopped;
}

bool feedback_ready(const AxisState& axis, bool is_lift) {
  return axis.initialized && axis.homed && axis.mode != AxisMode::Fault &&
         (is_lift ? LIFT_FEEDBACK_VERIFIED : ARM_FEEDBACK_VERIFIED);
}
bool limits_ready(bool is_lift) {
  const bool calibrated = is_lift ? LIFT_LIMITS_CALIBRATED : ARM_LIMITS_CALIBRATED;
  const double minimum = is_lift ? LIFT_MIN_DEG : ARM_MIN_DEG;
  const double maximum = is_lift ? LIFT_MAX_DEG : ARM_MAX_DEG;
  return calibrated && std::isfinite(minimum) && std::isfinite(maximum) && maximum > minimum;
}

void initialize_axis(AxisState& axis, bool is_lift) {
  axis = AxisState{}; // Discard old targets; startup position is NOT software zero.
  axis.initialized = true;
  bool brakes_ok;
  if (is_lift) {
    const auto left_result = left_lift_motor().set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    const auto right_result = right_lift_motor().set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    const auto third_result = third_lift_motor().set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    brakes_ok = left_result != PROS_ERR && right_result != PROS_ERR &&
                third_result != PROS_ERR;
  } else {
    brakes_ok = arm_motor().set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE) != PROS_ERR;
  }
  const bool stopped = write_voltage(is_lift, 0);
  const double position = read_position(is_lift);
  axis.last_update = pros::millis();
  axis.previous_position = position;
  if (!brakes_ok || !stopped || !std::isfinite(position)) disable_axis(axis, is_lift, true);
}

// ----- Physical home reference -----
// The caller must place the mechanism at home before resetting its sensor position.
bool zero_axis(AxisState& axis, bool is_lift) {
  if (!is_lift && (ARM_ROTATION_PORT < 1 || ARM_ROTATION_PORT > 21)) return false;
  if (!axis.initialized || axis.mode == AxisMode::Fault) return false;
  disable_axis(axis, is_lift);
  axis.homed = false;
  if (!write_voltage(is_lift, 0)) {
    disable_axis(axis, is_lift, true);
    return false;
  }
  // API verified in installed include/pros/rotation.hpp: reset_position(), NOT
  // reset(). Caller has physically placed/supported the mechanism at known home.
  const auto result = is_lift ? lift_sensor().reset_position() : arm_sensor().reset_position();
  const double position = read_position(is_lift);
  if (result == PROS_ERR || !std::isfinite(position)) {
    disable_axis(axis, is_lift, true);
    return false;
  }
  axis.homed = true;
  axis.target = 0.0;
  axis.previous_position = position;
  axis.last_update = pros::millis();
  return true; // Still Disabled; zeroing does not energize the mechanism.
}

void capture_hold(AxisState& axis, bool is_lift) {
  if (is_lift && lift_output_owner != LiftOutputOwner::None) return;
  if (is_lift && returning_home) return;
  axis.manual_mv = 0;
  axis.settling = false;
  axis.settled = false;
  if (!feedback_ready(axis, is_lift)) {
    disable_axis(axis, is_lift, axis.mode == AxisMode::Fault);
    return;
  }
  const double position = read_position(is_lift);
  if (!std::isfinite(position)) {
    disable_axis(axis, is_lift, true);
    return;
  }
  axis.target = position; // Capture ONCE, rather than following a falling lift.
  axis.previous_position = position;
  axis.last_update = pros::millis();
  axis.mode = AxisMode::Hold;
  if (!write_voltage(is_lift, 0)) disable_axis(axis, is_lift, true);
}

int limit_voltage(const AxisState& axis, bool is_lift, double position, int mv) {
  if (!axis.homed) return mv; // Unreferenced commissioning jog needs supervision.
  if (!limits_ready(is_lift)) return mv; // Unknown bounds must not imply a calibrated zero limit.
  const double minimum = is_lift ? LIFT_MIN_DEG : ARM_MIN_DEG;
  const double maximum = is_lift ? LIFT_MAX_DEG : ARM_MAX_DEG;
  if (position <= minimum && mv < 0) return 0;
  if (limits_ready(is_lift) && position >= maximum && mv > 0) return 0;
  return mv;
}

// ----- Low-voltage manual control -----
// Held commands must be refreshed; this is separate from the full-voltage driver test.
void manual_axis(AxisState& axis, bool is_lift, int direction) {
  if (pros::competition::is_disabled()) {
    disable_axis(axis, is_lift, axis.mode == AxisMode::Fault);
    return;
  }
  if (is_lift && lift_output_owner != LiftOutputOwner::None) return;
  if (is_lift && returning_home) return;
  if (!axis.initialized || axis.mode == AxisMode::Fault) return;
  // Cancel automatic movement BEFORE checking the commissioning enable flag.
  // Manual state blocks target requests until explicit release or timeout.
  if (axis.mode == AxisMode::Disabled) {
    axis.last_update = pros::millis();
    axis.previous_position = read_position(is_lift);
  }
  axis.mode = AxisMode::Manual;
  axis.settling = false;
  axis.settled = false;
  axis.manual_refresh = pros::millis();
  const bool enabled = is_lift ? ENABLE_LIFT_MANUAL : ENABLE_ARM_MANUAL;
  const int speed = is_lift ? LIFT_MANUAL_MV : ARM_MANUAL_MV;
  if (speed < 0 || speed > 12000) {
    disable_axis(axis, is_lift, true);
    return;
  }
  axis.manual_mv = enabled ? direction * speed : 0;
  const double position = read_position(is_lift);
  if (!std::isfinite(position) ||
      !write_voltage(is_lift, limit_voltage(axis, is_lift, position, axis.manual_mv)))
    disable_axis(axis, is_lift, true);
}

// ----- Lift and arm target control -----
// Automatic moves need a known home, verified feedback and measured safe limits.
bool target_allowed(const AxisState& axis, bool is_lift, double target) {
 
return !(is_lift &&
         (ENABLE_LIFT_CALIBRATION_MODE ||
          returning_home ||
          lift_output_owner != LiftOutputOwner::None)) &&
       feedback_ready(axis, is_lift) &&
       limits_ready(is_lift) &&
       axis.mode != AxisMode::Manual &&
       std::isfinite(target) &&
       target >= (is_lift ? LIFT_MIN_DEG : ARM_MIN_DEG) &&
       target <= (is_lift ? LIFT_MAX_DEG : ARM_MAX_DEG);

}
bool set_target(AxisState& axis, bool is_lift, double target,
                unsigned timeout_ms = TARGET_TIMEOUT_MS) {
  if (pros::competition::is_disabled() || timeout_ms == 0) return false;
  if (!target_allowed(axis, is_lift, target)) return false;
  const double position = read_position(is_lift);
  if (!std::isfinite(position)) {
    disable_axis(axis, is_lift, true);
    return false;
  }
  axis.target = target;
  axis.mode = AxisMode::MoveToTarget;
  axis.target_started = pros::millis();
  axis.target_timeout_ms = timeout_ms;
  axis.last_update = axis.target_started;
  axis.previous_position = position;
  axis.settling = false;
  axis.settled = false;
  return true; // Actual control runs on next update; never wait here.
}

// One control step: read position, check timeouts, then choose a limited motor voltage.
// The owning task must keep calling this; it is not a background controller.

void update_axis(AxisState& axis, bool is_lift) {
  if (!axis.initialized ||
      axis.mode == AxisMode::Disabled ||
      axis.mode == AxisMode::Fault) return;

  const unsigned now = pros::millis();
  const unsigned elapsed = now - axis.last_update;

  if (elapsed > MAX_UPDATE_GAP_MS) {
    // A delayed automatic lift update is a recoverable abort.
    // Stop motor output without intentionally erasing the home reference.
    if (is_lift &&
        (axis.mode == AxisMode::MoveToTarget ||
         axis.mode == AxisMode::Hold)) {

      const bool stopped = disable_axis(axis, true);

      std::printf(
          "LIFT AUTO ABORT: update gap %u ms; stopped=%d; homed=%d\n",
          elapsed, stopped, lift.homed
      );
    } else {
      disable_axis(axis, is_lift, true);
    }
    return;
  }

  const double position = read_position(is_lift);

  if (!std::isfinite(position)) {
    disable_axis(axis, is_lift, true);

    if (is_lift)
      std::printf("LIFT HARD FAULT: P16 feedback invalid\n");

    return;
  }

  if (axis.mode == AxisMode::Manual &&
      now - axis.manual_refresh >= MANUAL_COMMAND_TIMEOUT_MS) {
    capture_hold(axis, is_lift);
    return;
  }

  if (elapsed < UPDATE_PERIOD_MS) return;

  const double velocity =
      (position - axis.previous_position) * 1000.0 / elapsed;

  axis.last_update = now;
  axis.previous_position = position;

  int mv = axis.manual_mv;

  if (axis.mode != AxisMode::Manual) {
    if (!feedback_ready(axis, is_lift)) {
      disable_axis(axis, is_lift);
      return;
    }

    const double error = axis.target - position;
    const double tolerance = is_lift
        ? LIFT_TARGET_TOLERANCE_DEG
        : ARM_TARGET_TOLERANCE_DEG;

    if (std::fabs(error) <= tolerance) {
      if (!axis.settling)
        axis.settled_since = now;

      axis.settling = true;
      axis.settled =
          now - axis.settled_since >= TARGET_SETTLE_MS;
    } else {
      axis.settling = false;
      axis.settled = false;
    }

    if (axis.mode == AxisMode::MoveToTarget) {
      if (axis.settled) {
        axis.mode = AxisMode::Hold;
      } else if (
          now - axis.target_started >= axis.target_timeout_ms) {

        if (is_lift) {
          const double target = axis.target;
          const bool stopped = disable_axis(axis, true);

          std::printf(
              "LIFT AUTO ABORT: target timeout %.1f->%.1f deg; stopped=%d; homed=%d\n",
              position, target, stopped, lift.homed
          );
        } else {
          disable_axis(axis, false, true);
        }

        return;
      }
    }

    const double kp = is_lift
        ? LIFT_HOLD_KP : ARM_HOLD_KP;

    const double kd = is_lift
        ? LIFT_HOLD_KD : ARM_HOLD_KD;

    const int cap = is_lift
        ? LIFT_CLOSED_LOOP_MAX_MV
        : ARM_CLOSED_LOOP_MAX_MV;

    double output = kp * error - kd * velocity;

    if (!std::isfinite(output) ||
        cap < 0 || cap > 12000) {
      disable_axis(axis, is_lift, true);
      return;
    }

    if (output > cap) output = cap;
    if (output < -cap) output = -cap;

    mv = static_cast<int>(output);
  }

  if (!write_voltage(
          is_lift,
          limit_voltage(axis, is_lift, position, mv))) {

    disable_axis(axis, is_lift, true);

    if (is_lift)
      std::printf(
          "LIFT HARD FAULT: lift motor voltage command failed\n"
      );
  }
}


bool at_target(const AxisState& axis, bool is_lift) {
  if ((axis.mode != AxisMode::Hold && axis.mode != AxisMode::MoveToTarget) ||
      !axis.settled || pros::millis() - axis.last_update > MAX_UPDATE_GAP_MS) return false;
  const double position = read_position(is_lift);
  const double tolerance = is_lift ? LIFT_TARGET_TOLERANCE_DEG : ARM_TARGET_TOLERANCE_DEG;
  return std::isfinite(position) && std::fabs(axis.target - position) <= tolerance;
}
const char* mode_name(AxisMode mode) {
  switch (mode) {
    case AxisMode::Disabled: return "Disabled";
    case AxisMode::Manual: return "Manual";
    case AxisMode::Hold: return "Hold";
    case AxisMode::MoveToTarget: return "MoveToTarget";
    case AxisMode::Fault: return "Fault";
  }
  return "Unknown";
}

// ----- Optional claw-area detection -----
// Require repeated valid readings before requesting an arm action.
void update_object_detection() {
  if (CLAW_DISTANCE_PORT < 1 || CLAW_DISTANCE_PORT > 21 ||
      !ENABLE_CLAW_DISTANCE_AUTO_ARM || !CLAW_DISTANCE_CALIBRATED ||
      CLAW_DETECT_MIN_MM <= 0 || CLAW_DETECT_MAX_MM <= CLAW_DETECT_MIN_MM ||
      CLAW_DETECT_CONSISTENT_READINGS == 0) {
    object_detected = false;
    object_readings = 0;
    return;
  }
  const unsigned now = pros::millis();
  if (now - last_distance_sample < CLAW_DISTANCE_SAMPLE_MS) return;
  // Widely separated readings are not a consistent detection sequence.
  if (now - last_distance_sample > MAX_UPDATE_GAP_MS) {
    object_readings = 0;
    object_detected = false;
  }
  last_distance_sample = now;
  const auto mm = claw_distance().get();
  // Installed PROS documents confidence only above 200 mm. Near the claw,
  // validate range and repeated samples instead of rejecting unavailable confidence.
  bool valid = mm != PROS_ERR && mm != 9999 && mm > 0;
  if (valid && mm > DISTANCE_CONFIDENCE_AVAILABLE_ABOVE_MM) {
    const auto confidence = claw_distance().get_confidence();
    valid = confidence != PROS_ERR && confidence >= CLAW_DETECT_MIN_CONFIDENCE;
  }
  const bool in_range = valid && mm >= CLAW_DETECT_MIN_MM && mm <= CLAW_DETECT_MAX_MM;
  if (!in_range) object_readings = 0;
  else if (object_readings < CLAW_DETECT_CONSISTENT_READINGS) ++object_readings;
  const bool previous_detection = object_detected;
  object_detected = object_readings >= CLAW_DETECT_CONSISTENT_READINGS;
  // One request per detection edge; no queued surprise after manual release.
  // Do not replace a manual command or an existing automatic target.
  if (object_detected && !previous_detection && arm.mode == AxisMode::Hold)
    arm_move_score();
}
} // namespace

void lift_initialize() {
  lift_output_owner = LiftOutputOwner::None;
  lift_output_fault = false;
  returning_home = false;
  home_switch = LiftHomeSwitchState{};
  initialize_axis(lift, true);
}
void arm_initialize() { initialize_axis(arm, false); }
void mechanisms_initialize() {
  flex_stop();
  toggle_stop();
  lift_initialize();
  arm_initialize();
  object_detected = false;
  object_readings = 0;
  last_distance_sample = pros::millis();
  if (claw_pneumatic_available()) {
    claw_raw_state = false;
    claw_state_written = claw_piston(false).set_value(false) != PROS_ERR;
    claw_is_open = claw_raw_state == mechanism_config::CLAW_OPEN_OUTPUT;
  }
  if (mechanism_config::ENABLE_PNEUMATICS) {
    claw_flip_forward(); // Explicit false, never toggle.
  }
}
bool lift_zero_at_current_position() {
  return !returning_home && lift_output_owner == LiftOutputOwner::None && zero_axis(lift, true);
}
bool arm_zero_at_current_position() { return zero_axis(arm, false); }
void lift_manual_up() { manual_axis(lift, true, 1); }
void lift_manual_down() { manual_axis(lift, true, -1); }
void arm_manual_up() { manual_axis(arm, false, 1); }
void arm_manual_down() { manual_axis(arm, false, -1); }
void lift_manual_stop_and_hold() { capture_hold(lift, true); }
void arm_manual_stop_and_hold() { capture_hold(arm, false); }
void lift_cancel_auto() {
  if (lift.mode == AxisMode::MoveToTarget) capture_hold(lift, true);
}
void arm_cancel_auto() {
  if (arm.mode == AxisMode::MoveToTarget) capture_hold(arm, false);
}
bool lift_set_target(double position_deg) { return set_target(lift, true, position_deg); }

static double lift_valid_preset_degrees(
    lift_calibration::Position preset) {

  const unsigned index = static_cast<unsigned>(preset);

  if (index >= lift_calibration::positions.size()) {
    return NAN;
  }

  const auto& p = lift_calibration::positions[index];
  const double max_safe = mechanism_config::LIFT_MAX_DEG;

  if (!p.configured ||
      !std::isfinite(p.degrees) ||
      !std::isfinite(max_safe) ||
      max_safe <= 0.0 ||
      p.degrees <= 0.0 ||
      p.degrees > max_safe) {
    return NAN;
  }

  return p.degrees;
}

// Nonblocking position command, used by Y for Intake.
bool lift_move_preset(lift_calibration::Position preset) {
  const double degrees = lift_valid_preset_degrees(preset);

  if (!std::isfinite(degrees)) {
    return false;
  }

  return lift_set_target(degrees);
}

bool arm_set_target(double position_deg) { return set_target(arm, false, position_deg); }
bool lift_move_home() { return lift_set_target(mechanism_config::LIFT_HOME_DEG); }
bool lift_move_low() {
  return mechanism_config::LIFT_SCORING_POSITIONS_CALIBRATED && lift_set_target(mechanism_config::LIFT_LOW_DEG);
}
bool lift_move_mid() {
  return mechanism_config::LIFT_SCORING_POSITIONS_CALIBRATED && lift_set_target(mechanism_config::LIFT_MID_DEG);
}
bool lift_move_high() {
  return mechanism_config::LIFT_SCORING_POSITIONS_CALIBRATED && lift_set_target(mechanism_config::LIFT_HIGH_DEG);
}
bool lift_move_max() { return lift_set_target(mechanism_config::LIFT_MAX_DEG); }
bool arm_move_home() { return arm_set_target(mechanism_config::ARM_HOME_DEG); }
bool arm_move_score() {
  return mechanism_config::ARM_SCORING_POSITIONS_CALIBRATED && arm_set_target(mechanism_config::ARM_SCORE_DEG);
}
bool arm_move_max_lift_position() {
  return mechanism_config::ARM_SCORING_POSITIONS_CALIBRATED && arm_set_target(mechanism_config::ARM_MAX_LIFT_DEG);
}
LiftBottomLimit lift_bottom_limit_read() {
  if (mechanism_config::LIFT_BOTTOM_LIMIT_PORT == 0)
    return LiftBottomLimit::Unconfigured;
  static_assert(mechanism_config::LIFT_BOTTOM_LIMIT_PORT == 0 ||
                (mechanism_config::LIFT_BOTTOM_LIMIT_PORT >= 'A' &&
                 mechanism_config::LIFT_BOTTOM_LIMIT_PORT <= 'H'),
                "Select a real bottom switch ADI port A-H, or leave 0 unbound");
  static pros::adi::DigitalIn input(mechanism_config::LIFT_BOTTOM_LIMIT_PORT);
  const auto value = input.get_value();
  if (value != 0 && value != 1) return LiftBottomLimit::Fault;
  return (value != 0) == mechanism_config::LIFT_BOTTOM_LIMIT_PRESSED_VALUE
             ? LiftBottomLimit::Pressed : LiftBottomLimit::Released;
}
bool lift_return_home_active() { return returning_home; }
namespace {
// ----- Full-voltage lift control -----
// Use the shared UP/DOWN motor mapping and give this command exclusive lift ownership.

bool lift_full_command(
    int direction,
    LiftOutputOwner owner) {

    const bool enabled =
        !pros::competition::is_disabled() &&
        (pros::competition::is_autonomous() ==
         (owner == LiftOutputOwner::Autonomous));

    const bool calibration_driver =
        mechanism_config::ENABLE_LIFT_CALIBRATION_MODE &&
        owner == LiftOutputOwner::Driver &&
        !pros::competition::is_connected();

    // Normal movement requires qualified limits.
    // Calibration may bypass only the unknown upper limit.
    if (!enabled ||
        (!LIFT_LIMITS_CALIBRATED && !calibration_driver) ||
        !lift.initialized ||
        !lift.homed ||
        lift.mode == AxisMode::Fault ||
        lift_output_fault ||
        returning_home ||
        (direction != 1 && direction != -1)) {

        lift_auton_stop();
        return false;
    }

    const double current_deg = read_position(true);
    const double max_safe = mechanism_config::LIFT_MAX_DEG;

    const bool max_known =
        std::isfinite(max_safe) && max_safe > 0.0;

    if (!std::isfinite(current_deg) ||
        (direction > 0 &&
         ((max_known && current_deg >= max_safe) ||
          (!max_known && !calibration_driver))) ||
        (direction < 0 &&
         current_deg <= mechanism_config::LIFT_LOWER_STOP_MARGIN_DEG)) {

        lift_auton_stop();
        return false;
    }

    if (direction < 0) {
        const auto bottom = lift_bottom_limit_read();

        if (bottom == LiftBottomLimit::Pressed ||
            bottom == LiftBottomLimit::Fault) {
            lift_auton_stop();
            return false;
        }
    }

    if (lift_output_owner != owner ||
        lift_full_direction != direction) {

        if (!disable_axis(
                lift, true,
                lift.mode == AxisMode::Fault)) {
            return false;
        }
    }

    lift_output_owner = owner;
    lift_full_direction = direction;
    lift_full_last_refresh = pros::millis();

    if (!write_voltage(
            true,
            direction * LIFT_COMMISSIONING_MV)) {

        disable_axis(lift, true, true);
        return false;
    }

    return true;
}

// Blocking target wrapper: service mechanisms until success, failure or timeout.
// STOP is the default; powered hold needs explicit validation and continued updates.
bool auton_move_axis(AxisState& axis, bool is_lift, double target, unsigned timeout_ms,
                     LiftFinish finish = LiftFinish::STOP) {
  // Sequential ownership transfer: abandon any previous output before validation.
  if (!disable_axis(axis, is_lift, axis.mode == AxisMode::Fault)) return false;
  if (finish != LiftFinish::STOP && (finish != LiftFinish::HOLD_CURRENT_TARGET ||
      !is_lift || !lift_calibration::hold_validated)) return false;
  if (pros::competition::is_disabled() || !pros::competition::is_autonomous() ||
      timeout_ms == 0 || !set_target(axis, is_lift, target, timeout_ms)) return false;
  const unsigned began = pros::millis();
  while (!pros::competition::is_disabled() && pros::competition::is_autonomous() &&
         pros::millis() - began < timeout_ms) {
    mechanisms_update();
    if (axis.mode == AxisMode::Fault || !std::isfinite(read_position(is_lift))) break;
    if (at_target(axis, is_lift)) {
      if (finish == LiftFinish::HOLD_CURRENT_TARGET) return true;
      // No hidden powered hold while a subsequent legacy navigation call blocks.
      return disable_axis(axis, is_lift);
    }
    pros::delay(mechanism_config::UPDATE_PERIOD_MS);
  }
  disable_axis(axis, is_lift, axis.mode == AxisMode::Fault);
  return false;
}
} // namespace

bool lift_auton_up_full() { return lift_full_command(1, LiftOutputOwner::Autonomous); }
bool lift_auton_down_full() { return lift_full_command(-1, LiftOutputOwner::Autonomous); }
bool lift_auton_stop() {
  const bool cancelled = disable_axis(lift, true, lift.mode == AxisMode::Fault);
  const bool stopped = write_lift_pair(0); // Also covers an uninitialized axis.
  return cancelled && stopped;
}
bool lift_auton_move_to_deg(double target_deg, unsigned timeout_ms) {
  return auton_move_axis(lift, true, target_deg, timeout_ms);
}



bool lift_auton_to_preset(
    lift_calibration::Position preset,
    unsigned timeout_ms,
    LiftFinish finish) {

  const double degrees =
      lift_valid_preset_degrees(preset);

  if (!std::isfinite(degrees) || timeout_ms == 0) {
    return false;
  }

  return lift_auton_move_to_deg(
      degrees,
      timeout_ms,
      finish
  );
}

bool arm_auton_move_to_deg(double target_deg, unsigned timeout_ms) {
  return auton_move_axis(arm, false, target_deg, timeout_ms);
}
void lift_print_debug() {
  const double position = lift_position_deg();
  const bool target_active = lift.mode == AxisMode::MoveToTarget || lift.mode == AxisMode::Hold;
  const char* owner = lift_output_owner == LiftOutputOwner::Autonomous ? "AutonFull" :
      lift_output_owner == LiftOutputOwner::Driver ? "DriverFull" :
      returning_home ? "ReturningHome" : mode_name(lift.mode);
  std::printf("LIFT P16=%.3f deg valid=%d mode=%s target=%.3f target_active=%d "
              "homed=%d feedback_fault=%d output_fault=%d\n",
              position, std::isfinite(position), owner,
              target_active ? lift.target : NAN, target_active, lift.homed,
              lift.mode == AxisMode::Fault, lift_output_fault);
}

bool lift_return_home_start() {
  using namespace mechanism_config;
  if (lift_output_owner != LiftOutputOwner::None) return false;
  if (returning_home) return true; // Repeated requests never restart the timer.
  const auto bottom = lift_bottom_limit_read();
  if (!lift.initialized || lift.mode == AxisMode::Fault ||
      pros::competition::is_disabled() || !ENABLE_LIFT_MANUAL ||
      !claw_pneumatic_available() || LIFT_RETURN_TIMEOUT_MS == 0 ||
      LIFT_MANUAL_MV <= 0 || LIFT_MANUAL_MV > 12000 ||
      bottom == LiftBottomLimit::Unconfigured || bottom == LiftBottomLimit::Fault)
    return false;
  if (!write_voltage(true, 0)) { disable_axis(lift, true, true); return false; }
  returning_home = true;
  lift.mode = AxisMode::Disabled; // Existing target/hold must not compete.
  lift.settled = false;
  lift.manual_mv = 0;
  return_started_at = lift.last_update = pros::millis();
  return_started_in_autonomous = pros::competition::is_autonomous();
  return true;
}
// ----- Lift mode service -----
// Service whichever mode owns the lift: full output, target/hold, or switch-based return.
void lift_update() {
  if (pros::competition::is_disabled()) { disable_axis(lift, true, lift.mode == AxisMode::Fault); return; }

  if (lift_output_owner != LiftOutputOwner::None) {
    const bool expected_auton =
        lift_output_owner == LiftOutputOwner::Autonomous;

    if (pros::competition::is_autonomous() != expected_auton) {
      lift_auton_stop();
      return;
    }

    if (lift.mode == AxisMode::Fault || lift_output_fault) {
      disable_axis(lift, true, true);
      return;
    }

    
    // Stop a manual/open-loop command if its owner interferes(driver)
    // stops refreshing it. Checked cooperatively
    // whenever lift_update() executes.
    if (lift_full_last_refresh == 0 ||
        pros::millis() - lift_full_last_refresh >
            mechanism_config::MANUAL_COMMAND_TIMEOUT_MS) {

      lift_auton_stop();
      return;
    }


    if (!lift.homed) {
      lift_auton_stop();
      return;
    }

    const double position = read_position(true);

    if (!std::isfinite(position)) {
      disable_axis(lift, true, true);
      return;
    }
    
    
    // In calibration, an unknown maximum permits supervised L1 movement.
    // A measured maximum is always enforced.
    if (lift_full_direction > 0) {
      const double max_safe = mechanism_config::LIFT_MAX_DEG;
      const bool max_known = std::isfinite(max_safe) && max_safe > 0.0;
      const bool calibration_driver =
          mechanism_config::ENABLE_LIFT_CALIBRATION_MODE &&
          lift_output_owner == LiftOutputOwner::Driver &&
          !pros::competition::is_connected();
      if ((max_known && position >= max_safe) ||
          (!max_known && !calibration_driver)) {
        lift_auton_stop();
        return;
      }
    }


    if (lift_full_direction < 0) {
      const auto bottom = lift_bottom_limit_read();

      if (bottom == LiftBottomLimit::Fault) {
        disable_axis(lift, true, true);
        return;
      }

      
      if (bottom == LiftBottomLimit::Pressed ||
          position <= mechanism_config::LIFT_LOWER_STOP_MARGIN_DEG) {

          lift_auton_stop();
          return;
      }

    }

    if (lift_calibration::enable_limits &&
        !auton_math::lift_direction_allowed(
            lift_calibration::limits,
            position,
            lift_full_direction)) {
      lift_auton_stop();
      return;
    }

    return;
  }

  if (!returning_home) { update_axis(lift, true); return; }
  if (pros::competition::is_autonomous() != return_started_in_autonomous) {
    disable_axis(lift, true);
    return;
  }
  const unsigned now = pros::millis();
  const auto bottom = lift_bottom_limit_read();
  // Check the physical stop before any further downward output, every loop.
  if (bottom == LiftBottomLimit::Pressed) {
    // zero_axis first stops BOTH motors, then resets the reference ONCE.
    // Never close the claw if stopping/zeroing fails.
    if (zero_axis(lift, true) && !claw_close()) disable_axis(lift, true, true);
    return;
  }
  if (bottom != LiftBottomLimit::Released ||
      now - return_started_at >= mechanism_config::LIFT_RETURN_TIMEOUT_MS ||
      now - lift.last_update > mechanism_config::MAX_UPDATE_GAP_MS ||
      !std::isfinite(read_position(true))) {
    disable_axis(lift, true, true); // Latched fault; no claw closure or auto-retry.
    std::printf("Lift return fault: switch, sensor, timeout or update gap\n");
    return;
  }
  lift.last_update = now;
  if (!write_voltage(true, -mechanism_config::LIFT_MANUAL_MV))
    disable_axis(lift, true, true);
}
void arm_update() { update_axis(arm, false); }
void mechanisms_update() {
  if (pros::competition::is_disabled()) { stop_all_mechanisms(); return; }
  lift_update();
  arm_update();
  update_lift_home_switch();
  update_object_detection();
}
double lift_position_deg() { return read_position(true); }
double arm_position_deg() { return read_position(false); }
bool lift_at_target() { return at_target(lift, true); }
bool arm_at_target() { return at_target(arm, false); }
LiftMode lift_mode() {
  if (lift_output_owner == LiftOutputOwner::Autonomous) return LiftMode::AutonFull;
  if (lift_output_owner == LiftOutputOwner::Driver) return LiftMode::DriverFull;
  if (returning_home) return LiftMode::ReturningHome;
  switch (lift.mode) {
    case AxisMode::Disabled: return LiftMode::Disabled;
    case AxisMode::Manual: return LiftMode::Manual;
    case AxisMode::Hold: return LiftMode::Hold;
    case AxisMode::MoveToTarget: return LiftMode::MoveToTarget;
    case AxisMode::Fault: return LiftMode::Fault;
  }
  return LiftMode::Fault;
}
ArmMode arm_mode() {
  switch (arm.mode) {
    case AxisMode::Disabled: return ArmMode::Disabled;
    case AxisMode::Manual: return ArmMode::Manual;
    case AxisMode::Hold: return ArmMode::Hold;
    case AxisMode::MoveToTarget: return ArmMode::MoveToTarget;
    case AxisMode::Fault: return ArmMode::Fault;
  }
  return ArmMode::Fault;
}
// ----- Pneumatic commands -----
// Routes request open/closed states here instead of writing raw ADI outputs.
bool claw_set(bool open) {
  if (!open && returning_home) return false; // R1 cannot close during descent.
  if (!claw_pneumatic_available() || pros::competition::is_disabled()) return false;
  const bool output = open ? mechanism_config::CLAW_OPEN_OUTPUT : !mechanism_config::CLAW_OPEN_OUTPUT;
  if (claw_piston(output).set_value(output) == PROS_ERR) {
    claw_state_written = false;
    return false;
  }
  claw_raw_state = output;
  claw_is_open = open;
  claw_state_written = true;
  return true;
}
bool claw_open() { return claw_set(true); }
bool claw_close() { return claw_set(false); }
bool claw_toggle() { return claw_set(!claw_is_open); }
bool claw_flip_set(bool reverse) {
  if (!mechanism_config::ENABLE_PNEUMATICS || pros::competition::is_disabled()) return false;
  const bool output = reverse ? mechanism_config::FLIP_REVERSE_OUTPUT : !mechanism_config::FLIP_REVERSE_OUTPUT;
  if (flip_piston().set_value(output) == PROS_ERR) {
    flip_state_written = false;
    return false;
  }
  flip_is_reverse = reverse;
  flip_state_written = true;
  return true;
}
bool claw_flip_forward() { return claw_flip_set(false); }
bool claw_flip_reverse() { return claw_flip_set(true); }
bool claw_flip_toggle() { return claw_flip_set(!flip_is_reverse); }
bool claw_object_detected() {
  return object_detected && pros::millis() - last_distance_sample <= mechanism_config::MAX_UPDATE_GAP_MS;
}
bool mechanism_return_home() {
  return lift_return_home_start();
}
bool color_toggle_set(bool held) {
  return held ? toggle_run() : toggle_stop(); // Compatibility; no latch.
}
// Shared motor cleanup for faults and mode changes; leave pneumatic positions alone.
bool stop_all_mechanisms() {
  flex_hold_active = false;
  const bool flex_ok = !flex_available() || flex_stop();
  const bool toggle_ok = !toggle_available() || toggle_stop();
  home_switch = LiftHomeSwitchState{};
  const bool lift_ok = disable_axis(lift, true, lift.mode == AxisMode::Fault);
  const bool arm_ok = disable_axis(arm, false, arm.mode == AxisMode::Fault);
  object_detected = false;
  object_readings = 0;
  return flex_ok && toggle_ok && lift_ok && arm_ok;
}
void mechanisms_print_debug() {
  static unsigned last_print = 0;
  static bool printed = false;
  const unsigned now = pros::millis();
  if (printed && now - last_print < mechanism_config::DEBUG_PERIOD_MS) return;
  printed = true;
  last_print = now;
  lift_print_debug();
  std::printf("Arm pos=%.2f target=%.2f mode=%s home=%d\n",
              arm_position_deg(), arm.target, mode_name(arm.mode), arm.homed);
  std::printf("Claw open=%d written=%d | Flip reverse=%d written=%d | Distance enabled=%d detected=%d\n",
              claw_is_open, claw_state_written, flip_is_reverse, flip_state_written,
              mechanism_config::ENABLE_CLAW_DISTANCE_AUTO_ARM, claw_object_detected());
}
void test_lift_manual(bool up_held, bool down_held) {
  if (up_held && down_held) manual_axis(lift, true, 0);
  else if (up_held) lift_manual_up();
  else if (down_held && !up_held) lift_manual_down();
  else if (lift.mode == AxisMode::Manual) lift_manual_stop_and_hold();
}
// Three-motor lift commissioning: continuous held buttons, startup reference, guarded lower stop.





bool test_lift_motor_response(
    bool up_held,
    bool down_held) {

    if (mechanism_config::ENABLE_LIFT_CALIBRATION_MODE) {

        // Continuous hold-to-move calibration.
        static int previous_direction = 0;
        static unsigned held_since = 0;
        static double started_deg = 0.0;
        static bool timed_out = false;

        const bool driver_allowed =
            !pros::competition::is_disabled() &&
            !pros::competition::is_autonomous() &&
            !pros::competition::is_connected();

        // Release, both buttons, or disabled = STOP.
        if (!driver_allowed || up_held == down_held) {
            previous_direction = 0;
            held_since = 0;
            timed_out = false;
            return lift_auton_stop();
        }

        const int direction = up_held ? 1 : -1;

        if (direction != previous_direction) {
            previous_direction = direction;
            held_since = pros::millis();
            started_deg = read_position(true);
            timed_out = false;
        }

        const double current_deg = read_position(true);

        if (!std::isfinite(current_deg) ||
            !std::isfinite(started_deg)) {

            disable_axis(lift, true, true);
            return false;
        }

        // Reaching a limit is a normal stop, NOT a stall fault.
        // L2 remains blocked here while held; lifting away is allowed.
        const double max_safe = mechanism_config::LIFT_MAX_DEG;
        const bool at_lower = direction < 0 &&
            current_deg <= mechanism_config::LIFT_LOWER_STOP_MARGIN_DEG;
        const bool at_upper = direction > 0 &&
            std::isfinite(max_safe) && max_safe > 0.0 &&
            current_deg >= max_safe;
        if (at_lower || at_upper) {
            lift_auton_stop();
            return true;
        }

        // Fault if P16 moves opposite to the command.
        if ((current_deg - started_deg) * direction < -2.0) {
            std::printf(
                "LIFT CAL FAULT: wrong P16 direction\n"
            );
            disable_axis(lift, true, true);
            return false;
        }

        // Fault if the lift fails to start moving.
        const unsigned held_ms =
            pros::millis() - held_since;

        if (held_ms >=
                mechanism_config::LIFT_CALIBRATION_MOTION_CHECK_MS &&
            (current_deg - started_deg) * direction <
                mechanism_config::LIFT_CALIBRATION_MIN_MOTION_DEG) {

            std::printf(
                "LIFT CAL FAULT: no P16 movement\n"
            );
            disable_axis(lift, true, true);
            return false;
        }

        // One continuous hold has a safety timeout.
        if (timed_out ||
            pros::millis() - held_since >=
                mechanism_config::LIFT_CALIBRATION_MAX_HOLD_MS) {

            if (!timed_out) {
                std::printf(
                    "LIFT CAL: hold timeout, release button\n"
                );
            }

            timed_out = true;
            lift_auton_stop();
            return false;
        }

        const bool success =
            lift_full_command(
                direction,
                LiftOutputOwner::Driver
            );

        if (!success) lift_auton_stop();
        return success;
    }

    // Normal match control with verified limits.
    const bool enabled =
        !pros::competition::is_disabled() &&
        !pros::competition::is_autonomous();

    if (!enabled || (up_held && down_held)) {
        return lift_auton_stop();
    }

    if (!up_held && !down_held) {
        if (lift_output_owner == LiftOutputOwner::Driver) {
            return lift_auton_stop();
        }

        // Preserve any active Y Intake preset.
        return true;
    }

    const int direction = up_held ? 1 : -1;

    const bool success =
        lift_full_command(
            direction,
            LiftOutputOwner::Driver
        );

    if (!success) lift_auton_stop();
    return success;
}


void test_arm_manual(bool up_held, bool down_held) {
  if (up_held && down_held) manual_axis(arm, false, 0);
  else if (up_held) arm_manual_up();
  else if (down_held && !up_held) arm_manual_down();
  else if (arm.mode == AxisMode::Manual) arm_manual_stop_and_hold();
}
void test_lift_rotation_sensor() {
  static unsigned last = 0;
  if (pros::millis() - last < mechanism_config::DEBUG_PERIOD_MS) return;
  last = pros::millis();
  std::printf("Lift sensor only: %.2f deg (NAN = invalid)\n", lift_position_deg());
}
void test_arm_rotation_sensor() {
  static unsigned last = 0;
  if (pros::millis() - last < mechanism_config::DEBUG_PERIOD_MS) return;
  last = pros::millis();
  std::printf("Arm sensor only: %.2f deg (NAN = invalid)\n", arm_position_deg());
}
bool test_lift_hold() {
  lift_manual_stop_and_hold();
  return lift.mode == AxisMode::Hold;
}
bool test_lift_target(double position_deg) { return lift_set_target(position_deg); }
bool test_arm_target(double position_deg) { return arm_set_target(position_deg); }
bool test_claw(bool open) { return claw_set(open); }
bool test_claw_flip(bool reverse) { return claw_flip_set(reverse); }
void lift_up() { lift_manual_up(); }
void lift_down() { lift_manual_down(); }
void lift_stop() { lift_manual_stop_and_hold(); }

namespace {
constexpr bool smart_port_valid(int port) { return port >= 1 && port <= 21; }
// Reject active assignments and the remaining P13 historical reservation.
// Legacy P11/P12 distance placeholders are inactive; that module remains excluded.
constexpr bool new_motor_port_free(int port) {
  return smart_port_valid(port) && port != 1 && port != 2 && port != 3 &&
         port != 4 && port != 5 && port != 7 && port != 20 &&
         port != 13 &&
         port != LEFT_LIFT_PORT && port != RIGHT_LIFT_PORT &&
         port != THIRD_LIFT_PORT &&
         port != LIFT_ROTATION_PORT && port != ARM_MOTOR_PORT &&
         port != ARM_ROTATION_PORT && port != CLAW_DISTANCE_PORT;
}
// All accessors are private and called ONLY after the corresponding availability guard.
pros::Motor& flex_left_motor() {
  static pros::Motor motor(FLEX_LEFT_REVERSED ? -FLEX_LEFT_PORT : FLEX_LEFT_PORT);
  return motor; // 5.5 W: open-loop move(), no assumed 11 W cartridge.
}
pros::Motor& flex_right_motor() {
  static pros::Motor motor(FLEX_RIGHT_REVERSED ? -FLEX_RIGHT_PORT : FLEX_RIGHT_PORT);
  return motor;
}
pros::Motor& toggle_motor() {
  static pros::Motor motor(TOGGLE_REVERSED ? -TOGGLE_PORT : TOGGLE_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}
pros::Motor& toggle_other_motor() {
  static pros::Motor motor(TOGGLE_OTHER_REVERSED ? -TOGGLE_OTHER_PORT : TOGGLE_OTHER_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}
// ----- Flex intake -----
// Apply the configured intake direction to both mirrored motors; stop both if one write fails.
bool flex_command(int power, int direction) {
  if (!flex_available()) return false;
  if (pros::competition::is_disabled() || power < 0 || power > 127) {
    flex_stop();
    return false;
  }
  const int command = power * FLEX_INTAKE_SIGN * direction;
  const auto left = flex_left_motor().move(command);
  const auto right = flex_right_motor().move(command);
  if (left == PROS_ERR || right == PROS_ERR) {
    flex_stop(); // A failed half-pair must not leave the other wheel running.
    return false;
  }
  return true;
}
bool claw_and_flex(bool open, bool intake, int power) {
  if (power <= 0 || power > 127 || !flex_available()) { flex_stop(); return false; }
  const bool piston_ok = open ? claw_open() : claw_close();
  if (!piston_ok) { flex_stop(); return false; }
  return intake ? flex_intake(power) : flex_outtake(power);
}
}
bool claw_pneumatic_available() {
  using namespace mechanism_config;
  return CLAW_PNEUMATIC_CONFIGURED && CLAW_PNEUMATIC_PORT >= 'A' &&
         CLAW_PNEUMATIC_PORT <= 'H' && CLAW_PNEUMATIC_PORT != LIFT_HOME_SWITCH_ADI_PORT &&
         CLAW_PNEUMATIC_PORT != LIFT_BOTTOM_LIMIT_PORT &&
         (!ENABLE_PNEUMATICS || CLAW_PNEUMATIC_PORT != CLAW_FLIP_ADI_PORT);
}
bool flex_available() {
  using namespace mechanism_config;
  return FLEX_CONFIGURED && new_motor_port_free(FLEX_LEFT_PORT) &&
         new_motor_port_free(FLEX_RIGHT_PORT) && FLEX_LEFT_PORT != FLEX_RIGHT_PORT &&
         FLEX_LEFT_REVERSED != FLEX_RIGHT_REVERSED &&
         (FLEX_INTAKE_SIGN == 1 || FLEX_INTAKE_SIGN == -1) &&
         (!TOGGLE_CONFIGURED || (FLEX_LEFT_PORT != TOGGLE_PORT && FLEX_RIGHT_PORT != TOGGLE_PORT &&
                                FLEX_LEFT_PORT != TOGGLE_OTHER_PORT && FLEX_RIGHT_PORT != TOGGLE_OTHER_PORT));
}
bool toggle_available() {
  using namespace mechanism_config;
  return TOGGLE_CONFIGURED && new_motor_port_free(TOGGLE_PORT) &&
         new_motor_port_free(TOGGLE_OTHER_PORT) && TOGGLE_PORT != TOGGLE_OTHER_PORT &&
         (!FLEX_CONFIGURED || (TOGGLE_PORT != FLEX_LEFT_PORT && TOGGLE_PORT != FLEX_RIGHT_PORT &&
                              TOGGLE_OTHER_PORT != FLEX_LEFT_PORT && TOGGLE_OTHER_PORT != FLEX_RIGHT_PORT));
}
bool flex_intake(int power) { return flex_command(power, 1); }
bool flex_outtake(int power) { return flex_command(power, -1); }
bool flex_stop() {
  if (!flex_available()) return false;
  const auto left = flex_left_motor().move_voltage(0);
  const auto right = flex_right_motor().move_voltage(0);
  return left != PROS_ERR && right != PROS_ERR;
}
namespace {
// ----- Toggle pair -----
// Single-motor test commands only P19; otherwise the mirrored P6/P19 pair shares a command.
bool toggle_command(bool reverse) {
  if (!toggle_available()) return false;
  if (pros::competition::is_disabled() || TOGGLE_POWER < 1 || TOGGLE_POWER > 127 ||
      (TOGGLE_DIRECTION != 1 && TOGGLE_DIRECTION != -1) ||
      (!TOGGLE_SINGLE_MOTOR_TEST && TOGGLE_REVERSED == TOGGLE_OTHER_REVERSED)) { toggle_stop(); return false; }
  const int command = TOGGLE_POWER * TOGGLE_DIRECTION * (reverse ? -1 : 1);
  const auto first = toggle_motor().move(command);
  if (TOGGLE_SINGLE_MOTOR_TEST) {
    if (first != PROS_ERR) return true;
  } else {
    const auto second = toggle_other_motor().move(command);
    if (first != PROS_ERR && second != PROS_ERR) return true;
  }
  toggle_stop();
  return false;
}
} // namespace
bool toggle_run() { return toggle_command(false); }
bool toggle_run_reverse() { return toggle_command(true); }
bool toggle_stop() {
  if (!toggle_available()) return false;
  const auto first = toggle_motor().move_voltage(0);
  if (TOGGLE_SINGLE_MOTOR_TEST) return first != PROS_ERR;
  const auto second = toggle_other_motor().move_voltage(0);
  return first != PROS_ERR && second != PROS_ERR;
}
bool claw_grab(int power) { return claw_and_flex(false, true, power); }
bool claw_score(int power) { return claw_and_flex(true, false, power); }
void claw_driver_control(bool flex_held, bool claw_new_press) {

  if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
    flex_hold_active = false;
    flex_stop();
    return;
  }

  // Any R1 press stops the flex and keeps it stopped
  // until R2 is pressed again.
  if (claw_new_press) {
    flex_hold_active = false;
    flex_stop();

    if (claw_pneumatic_available()) {
      if (claw_toggle()) {
        std::printf("CLAW RAW STATE: %s\n",
                    claw_raw_state ? "ON" : "OFF");
      } else {
        claw_state_written = false;
        std::printf("CLAW RAW WRITE FAILED\n");
      }
    }

    return;
  }

  // R2 held = 100%.
  if (flex_held && flex_available()) {
    flex_hold_active = true;

    if (!flex_intake(127)) {
      flex_hold_active = false;
      flex_stop();
    }
  }

  // R2 released after intake = about 10%.
  else if (flex_hold_active && flex_available()) {
    if (!flex_intake(13)) {
      flex_hold_active = false;
      flex_stop();
    }
  }

  // Before the first R1 press, or after R2, stay stopped.
  else {
    flex_stop();
  }
}
bool pickup_prepare(int power) { return claw_and_flex(true, true, power); }
bool pickup_capture(int power) { return claw_grab(power); }
bool pickup_finish() { return flex_stop(); }

bool lift_target_ready() {
  return feedback_ready(lift,true) && limits_ready(true) &&
      std::isfinite(read_position(true)) && !lift_output_fault;
}
bool lift_auton_move_to_deg(double target,unsigned timeout,LiftFinish finish) {
  return auton_move_axis(lift,true,target,timeout,finish);
}
bool lift_move_calibrated_position(unsigned preset,unsigned timeout,LiftFinish finish) {
  if (preset >= lift_calibration::positions.size()) return false;
  const auto& p=lift_calibration::positions[preset];
  return p.configured && std::isfinite(p.degrees) && lift_auton_move_to_deg(p.degrees,timeout,finish);
}
bool toggle_connected() {
  return toggle_available() && toggle_motor().is_installed() &&
         (TOGGLE_SINGLE_MOTOR_TEST || toggle_other_motor().is_installed());
}
// ----- Route hardware snapshot -----
// Report every mechanism, but only required devices can fail this route's preflight.
bool mechanisms_preflight(bool require_lift,bool require_toggle,bool require_flex) {
  const bool lift_sensor_ok=std::isfinite(lift_position_deg());
  const bool lift_motors=left_lift_motor().is_installed() &&
      right_lift_motor().is_installed() && third_lift_motor().is_installed();

  // Diagnose each lift motor individually.
  std::printf("LIFT DEVICE STATUS: P13=%d P14=%d P15=%d\n",
              third_lift_motor().is_installed(),
              left_lift_motor().is_installed(),
              right_lift_motor().is_installed());

  const bool toggle_ok=toggle_connected();
  const bool flex_ok=flex_available() && flex_left_motor().is_installed() && flex_right_motor().is_installed();
  std::printf("PREFLIGHT lift P%d/P%d/P%d motors=%d rotation P%d=%d target_ready=%d %s\n",
      LEFT_LIFT_PORT,RIGHT_LIFT_PORT,THIRD_LIFT_PORT,lift_motors,
      LIFT_ROTATION_PORT,lift_sensor_ok,lift_target_ready(),
      require_lift ? "REQUIRED FOR ROUTE" : "OPTIONAL");
  std::printf("PREFLIGHT toggle P%d secondary=%d connected=%d %s\n",TOGGLE_PORT,
      TOGGLE_SINGLE_MOTOR_TEST ? 0 : TOGGLE_OTHER_PORT,toggle_ok,
      require_toggle ? "REQUIRED FOR ROUTE" : "OPTIONAL");
  std::printf("PREFLIGHT flex P%d/P%d connected=%d %s\n",FLEX_LEFT_PORT,FLEX_RIGHT_PORT,flex_ok,
      require_flex ? "REQUIRED FOR ROUTE" : "OPTIONAL");
  std::printf("PREFLIGHT claw configured=%d (no physical position feedback); arm sensor port=%d OPTIONAL / UNCONFIGURED\n",
      claw_pneumatic_available(),ARM_ROTATION_PORT);
  return (!require_lift || (lift_sensor_ok && lift_motors && lift_target_ready())) &&
      (!require_toggle || toggle_ok) && (!require_flex || flex_ok);
}
