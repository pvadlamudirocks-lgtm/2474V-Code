#include "subsystems.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

namespace {
using namespace mechanism_config;
// Lazy construction: linking this file does not configure hardware during the
// existing odometry-only startup. A device is constructed only when requested.
pros::Motor& left_lift_motor() {
  static pros::Motor motor(LEFT_LIFT_REVERSED ? -LEFT_LIFT_PORT : LEFT_LIFT_PORT,
                           pros::v5::MotorGears::green);
  return motor;
}
pros::Motor& right_lift_motor() {
  static pros::Motor motor(RIGHT_LIFT_REVERSED ? -RIGHT_LIFT_PORT : RIGHT_LIFT_PORT,
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
pros::adi::DigitalOut& claw_piston() {
  static pros::adi::DigitalOut piston(CLAW_ADI_PORT, !CLAW_OPEN_OUTPUT);
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
  unsigned settled_since = 0;
  bool settling = false;
  bool settled = false;
  int manual_mv = 0;
};
AxisState lift;
AxisState arm;
bool returning_home = false;
unsigned return_started_at = 0;
bool return_started_in_autonomous = false;
bool claw_is_open = false;
bool flip_is_reverse = false; // Known normal software state; no startup toggle.
bool claw_state_written = false;
bool flip_state_written = false;
bool object_detected = false;
unsigned object_readings = 0;
unsigned last_distance_sample = 0;

// In these small shared helpers, is_lift=true selects lift; false selects arm.
double read_position(bool is_lift) {
  const double ratio = is_lift ? LIFT_SENSOR_DEG_PER_MECHANISM_DEG : ARM_SENSOR_DEG_PER_MECHANISM_DEG;
  const auto raw = is_lift ? lift_sensor().get_position() : arm_sensor().get_position();
  if (raw == PROS_ERR || !std::isfinite(ratio) || ratio <= 0.0)
    return std::numeric_limits<double>::quiet_NaN();
  // Installed PROS returns signed accumulated CENTIDEGREES, not degrees.
  // get_position does not wrap every revolution like get_angle does.
  return static_cast<double>(raw) / 100.0 / ratio;
}

bool write_voltage(bool is_lift, int mv) {
  if (is_lift) {
    if (mv < 0) {
      const auto bottom = lift_bottom_limit_read();
      if (bottom == LiftBottomLimit::Pressed || bottom == LiftBottomLimit::Fault) {
        left_lift_motor().move_voltage(0);
        right_lift_motor().move_voltage(0);
        return bottom != LiftBottomLimit::Fault;
      }
    }
    // Issue BOTH writes even if one motor reports failure.
    const auto left_result = left_lift_motor().move_voltage(mv);
    const auto right_result = right_lift_motor().move_voltage(mv);
    return left_result != PROS_ERR && right_result != PROS_ERR;
  }
  return arm_motor().move_voltage(mv) != PROS_ERR;
}

void disable_axis(AxisState& axis, bool is_lift, bool fault = false) {
  if (is_lift) returning_home = false;
  if (axis.initialized) write_voltage(is_lift, 0);
  axis.mode = fault ? AxisMode::Fault : AxisMode::Disabled;
  axis.manual_mv = 0;
  axis.settling = false;
  axis.settled = false;
  // Faults latch. Recovery requires explicit initialize and physical re-zero.
  // Reconnecting a sensor must not resume an abandoned target.
  if (fault) axis.homed = false;
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
    brakes_ok = left_result != PROS_ERR && right_result != PROS_ERR;
  } else {
    brakes_ok = arm_motor().set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE) != PROS_ERR;
  }
  const bool stopped = write_voltage(is_lift, 0);
  const double position = read_position(is_lift);
  axis.last_update = pros::millis();
  axis.previous_position = position;
  if (!brakes_ok || !stopped || !std::isfinite(position)) disable_axis(axis, is_lift, true);
}

bool zero_axis(AxisState& axis, bool is_lift) {
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
  const double minimum = is_lift ? LIFT_MIN_DEG : ARM_MIN_DEG;
  const double maximum = is_lift ? LIFT_MAX_DEG : ARM_MAX_DEG;
  if (position <= minimum && mv < 0) return 0;
  if (limits_ready(is_lift) && position >= maximum && mv > 0) return 0;
  return mv;
}

void manual_axis(AxisState& axis, bool is_lift, int direction) {
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

bool target_allowed(const AxisState& axis, bool is_lift, double target) {
  return !(is_lift && returning_home) && feedback_ready(axis, is_lift) && limits_ready(is_lift) &&
         axis.mode != AxisMode::Manual && std::isfinite(target) &&
         target >= (is_lift ? LIFT_MIN_DEG : ARM_MIN_DEG) &&
         target <= (is_lift ? LIFT_MAX_DEG : ARM_MAX_DEG);
}
bool set_target(AxisState& axis, bool is_lift, double target) {
  if (!target_allowed(axis, is_lift, target)) return false;
  const double position = read_position(is_lift);
  if (!std::isfinite(position)) {
    disable_axis(axis, is_lift, true);
    return false;
  }
  axis.target = target;
  axis.mode = AxisMode::MoveToTarget;
  axis.target_started = pros::millis();
  axis.last_update = axis.target_started;
  axis.previous_position = position;
  axis.settling = false;
  axis.settled = false;
  return true; // Actual control runs on next update; never wait here.
}

void update_axis(AxisState& axis, bool is_lift) {
  if (!axis.initialized || axis.mode == AxisMode::Disabled || axis.mode == AxisMode::Fault) return;
  const unsigned now = pros::millis();
  const unsigned elapsed = now - axis.last_update;
  if (elapsed > MAX_UPDATE_GAP_MS) {
    // Cooperative stop on resumption, NOT an independent watchdog.
    disable_axis(axis, is_lift, true);
    return;
  }
  const double position = read_position(is_lift);
  if (!std::isfinite(position)) {
    disable_axis(axis, is_lift, true);
    return;
  }
  if (axis.mode == AxisMode::Manual && now - axis.manual_refresh >= MANUAL_COMMAND_TIMEOUT_MS) {
    capture_hold(axis, is_lift);
    return;
  }
  if (elapsed < UPDATE_PERIOD_MS) return;
  const double velocity = (position - axis.previous_position) * 1000.0 / elapsed;
  axis.last_update = now;
  axis.previous_position = position;
  int mv = axis.manual_mv;
  if (axis.mode != AxisMode::Manual) {
    if (!feedback_ready(axis, is_lift)) {
      disable_axis(axis, is_lift);
      return;
    }
    const double error = axis.target - position;
    const double tolerance = is_lift ? LIFT_TARGET_TOLERANCE_DEG : ARM_TARGET_TOLERANCE_DEG;
    if (std::fabs(error) <= tolerance) {
      if (!axis.settling) axis.settled_since = now;
      axis.settling = true;
      axis.settled = now - axis.settled_since >= TARGET_SETTLE_MS;
    } else {
      axis.settling = false;
      axis.settled = false;
    }
    if (axis.mode == AxisMode::MoveToTarget) {
      if (axis.settled) axis.mode = AxisMode::Hold; // Hold the SAME requested target.
      else if (now - axis.target_started >= TARGET_TIMEOUT_MS) {
        disable_axis(axis, is_lift, true); // No unlimited push into a blocked target.
        return;
      }
    }
    const double kp = is_lift ? LIFT_HOLD_KP : ARM_HOLD_KP;
    const double kd = is_lift ? LIFT_HOLD_KD : ARM_HOLD_KD;
    const int cap = is_lift ? LIFT_CLOSED_LOOP_MAX_MV : ARM_CLOSED_LOOP_MAX_MV;
    double output = kp * error - kd * velocity;
    if (!std::isfinite(output) || cap < 0 || cap > 12000) {
      disable_axis(axis, is_lift, true);
      return;
    }
    if (output > cap) output = cap;
    if (output < -cap) output = -cap;
    mv = static_cast<int>(output);
  }
  if (!write_voltage(is_lift, limit_voltage(axis, is_lift, position, mv)))
    disable_axis(axis, is_lift, true);
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

void update_object_detection() {
  if (!ENABLE_CLAW_DISTANCE_AUTO_ARM || !CLAW_DISTANCE_CALIBRATED ||
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
  returning_home = false;
  home_switch = LiftHomeSwitchState{};
  initialize_axis(lift, true);
}
void arm_initialize() { initialize_axis(arm, false); }
void mechanisms_initialize() {
  lift_initialize();
  arm_initialize();
  object_detected = false;
  object_readings = 0;
  last_distance_sample = pros::millis();
  if (mechanism_config::ENABLE_PNEUMATICS) {
    claw_close();
    claw_flip_forward(); // Explicit false, never toggle.
  }
}
bool lift_zero_at_current_position() { return !returning_home && zero_axis(lift, true); }
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
bool lift_return_home_start() {
  using namespace mechanism_config;
  if (returning_home) return true; // Repeated requests never restart the timer.
  const auto bottom = lift_bottom_limit_read();
  if (!lift.initialized || lift.mode == AxisMode::Fault ||
      pros::competition::is_disabled() || !ENABLE_LIFT_MANUAL ||
      !ENABLE_PNEUMATICS || LIFT_RETURN_TIMEOUT_MS == 0 ||
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
void lift_update() {
  if (pros::competition::is_disabled()) { disable_axis(lift, true, lift.mode == AxisMode::Fault); return; }
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
bool claw_set(bool open) {
  if (!open && returning_home) return false; // R1 cannot close during descent.
  if (!mechanism_config::ENABLE_PNEUMATICS) return false;
  const bool output = open ? mechanism_config::CLAW_OPEN_OUTPUT : !mechanism_config::CLAW_OPEN_OUTPUT;
  if (claw_piston().set_value(output) == PROS_ERR) {
    claw_state_written = false;
    return false;
  }
  claw_is_open = open;
  claw_state_written = true;
  return true;
}
bool claw_open() { return claw_set(true); }
bool claw_close() { return claw_set(false); }
bool claw_toggle() { return claw_set(!claw_is_open); }
bool claw_flip_set(bool reverse) {
  if (!mechanism_config::ENABLE_PNEUMATICS) return false;
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
  // TODO COLOR_TOGGLE_HARDWARE: bind the selected device here. Existing claw
  // flip on ADI B has no evidence of being this mechanism; do not reuse its port.
  (void)held;
  return false; // Explicitly unavailable, never claim a hardware action succeeded.
}
void stop_all_mechanisms() {
  home_switch = LiftHomeSwitchState{};
  disable_axis(lift, true, lift.mode == AxisMode::Fault);
  disable_axis(arm, false, arm.mode == AxisMode::Fault);
  object_detected = false;
  object_readings = 0;
}
void mechanisms_print_debug() {
  static unsigned last_print = 0;
  static bool printed = false;
  const unsigned now = pros::millis();
  if (printed && now - last_print < mechanism_config::DEBUG_PERIOD_MS) return;
  printed = true;
  last_print = now;
  std::printf("Lift pos=%.2f target=%.2f mode=%s home=%d | Arm pos=%.2f target=%.2f mode=%s home=%d\n",
              lift_position_deg(), lift.target, mode_name(lift.mode), lift.homed,
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
