#include "dev_tests.hpp"
#include "telemetry.hpp"
#include "auton_actions.hpp"
#include "autons.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "full_system_diag.hpp"
#include "motion_exit_trace.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <atomic>

namespace {
constexpr bool TWO_MOVE_CHAINED = true;
}
// ----- Chaining qualification -----
// Compare handoff and final-settle behavior using the existing two-move test.
void auton_two_move_chain_test() {
  nav_set_start(0, 0, 0);
  const double imu = chassis.drive_imu_get();
  if (!nav_validate_pose(0, 0, 0, 0.5, 0.5) || !std::isfinite(imu) ||
      std::fabs(std::remainder(imu - nav_game_to_internal_heading(0), 360.0)) > 0.5) {
    if (NAV_DIAGNOSTICS) {
      std::printf("TWO-MOVE TEST ABORTED: start pose/IMU did not settle.\n");
      nav_show_pose();
      nav_print_pose();
    }
    return;
  }

  NavSequence sequence(NAV_DIAGNOSTICS); // Buffered only; dump after finish/rest.
  const auto first_end = TWO_MOVE_CHAINED ? NavEndMode::CHAIN : NavEndMode::SETTLE;
  if (NAV_DIAGNOSTICS) {
    std::printf("\n========== TWO-MOVE START: %s ==========\n"
                "PUBLIC route: (0,0) -> (0,24) -> (24,24), speed=127, FRONT-first\n"
                "Existing EZ forward chain extension: %.3f in; CHAIN: native release; FINAL SETTLE: <=1 in\n",
                TWO_MOVE_CHAINED ? "CHAINED" : "FULL SETTLE",
                chassis.pid_drive_chain_forward_constant_get());
    nav_print_pose();
  }
  const auto start_pose = nav_pose_get();
  const auto began = pros::millis();
  const bool first_completed = nav_go_to(0, 24, 127, first_end);
  const auto first_elapsed = pros::millis() - began;
  const auto second_began = pros::millis();
  const bool completed = first_completed &&
                         nav_go_to(24, 24, 127, NavEndMode::SETTLE);
  const auto second_elapsed = pros::millis() - second_began;
  const auto elapsed_ms = pros::millis() - began; // Through move 2's full settle/rest.
  const bool finished = sequence.finish();       // Existing final stop/rest cleanup.

  if (NAV_DIAGNOSTICS) {
    std::printf("CHAIN leg 1 release: %lu ms; leg 2 settle: %lu ms (dispatched: %s)\n",
                static_cast<unsigned long>(first_elapsed), static_cast<unsigned long>(second_elapsed),
                first_completed ? "YES" : "NO");
    if (completed && finished)
      nav_print_test_result("CHAIN_TWO_MOVE", start_pose, {24, 24, ez::ANGLE_NOT_SET}, elapsed_ms);
    nav_trace_dump(); // No serial output occurred during either movement.
    if (!first_completed) std::printf("MOVE 2 NOT DISPATCHED: movement 1 failed.\n");
    if (completed && finished) {
      std::printf("\n========== TWO-MOVE CHAIN RESULT (%s) ==========\n"
                  "Elapsed through final settle: %lu ms\n",
                  TWO_MOVE_CHAINED ? "CHAINED" : "FULL SETTLE",
                  static_cast<unsigned long>(elapsed_ms));
      if (nav_debug_result(24, 24)) {
        pros::lcd::set_text(0, "TWO-MOVE CHAIN RESULT");
        pros::lcd::print(6, "ELAPSED: %lu ms", static_cast<unsigned long>(elapsed_ms));
        pros::lcd::set_text(7, TWO_MOVE_CHAINED ? "ROBOT | CHAINED" : "ROBOT | FULL SETTLE");
      }
    } else {
      std::printf("\nTWO-MOVE TEST FAILED: %s ; elapsed to failure=%lu ms.\n"
                  "Current snapshot follows; NOT a successful final result.\n",
                  nav_fault(), static_cast<unsigned long>(elapsed_ms));
      nav_show_pose();
      nav_print_pose();
      pros::lcd::set_text(0, "TWO-MOVE TEST FAILED");
      pros::lcd::set_text(5, nav_fault());
    }
    std::fflush(stdout);
  }
}

namespace {
const char* test_name(DevTest test) {
  switch (test) {
#define NAME(value) case DevTest::value: return #value
    NAME(SENSOR_HEALTH); NAME(START_POSE); NAME(MANUAL_ODOM);
    NAME(DRIVE_FORWARD_12); NAME(TURN_90); NAME(GOTO_Y_12); NAME(GOTO_X_12);
    NAME(REVERSE_12); NAME(RETURN_TO_ORIGIN); NAME(TWO_POINT); NAME(SQUARE);
    NAME(GOTO_POSE); NAME(CHAIN_TWO_MOVE); NAME(PURE_PURSUIT_BASIC); NAME(MIRROR_MATH);
    NAME(STABILITY_MONITOR); NAME(FULL_SYSTEM_DIAG); NAME(REVERSE_RIGHT_DIAG);
#undef NAME
  }
  return "UNKNOWN";
}

// Only TURN_90 opts into these checks. No logging, resets, or motor writes in
// this callback: the existing motor-owner task invokes it under its own lock.
// Capture only the first failed gate; the motor owner never prints or waits.
// Require live readiness; EZ calibration timing history remains informational.
enum class TurnSensorGate { None, OdomDisabled, LiveSensors };
std::atomic<TurnSensorGate> turn_sensor_gate{TurnSensorGate::None};
bool turn_90_sensors_ok() {
  const auto failed = !chassis.odom_enabled() ? TurnSensorGate::OdomDisabled :
                     !nav_sensors_ready() ? TurnSensorGate::LiveSensors : TurnSensorGate::None;
  if (failed != TurnSensorGate::None) {
    auto expected = TurnSensorGate::None;
    turn_sensor_gate.compare_exchange_strong(expected, failed);
  }
  return failed == TurnSensorGate::None;
}

const char* turn_sensor_gate_name() {
  switch (turn_sensor_gate.load()) {
    case TurnSensorGate::OdomDisabled: return "ODOMETRY_DISABLED";
    case TurnSensorGate::LiveSensors: return "LIVE_SENSOR_OR_POSE_CHECK_FAILED";
    default: return "NONE";
  }
}

// Caller must already have stopped the drivetrain. These are fresh diagnostic
// samples, not substitutes for the captured gate or a reason to retry movement.
void turn_90_log_checks(const char* phase, const char* reason) {
  const auto pose = nav_pose_get();
  const double imu = chassis.drive_imu_get();
  std::printf("TURN_90 CHECK phase=%s reason=%s first_failed_guard=%s "
              "ez_imu_calibrated=%d odom_enabled=%d historical_sensor_fault=%d "
              "competition_raw=%u autonomous=%d disabled=%d connected=%d deadline_state=%d\n",
              phase, reason, turn_sensor_gate_name(), chassis.drive_imu_calibrated(),
              chassis.odom_enabled(), nav_sensor_fault_latched(),
              static_cast<unsigned>(pros::competition::get_status()),
              pros::competition::is_autonomous(), pros::competition::is_disabled(),
              pros::competition::is_connected(),
              static_cast<int>(ez::geometry_turn_deadline_result()));
  std::printf("TURN_90 READBACK X=%.6f Y=%.6f H=%.6f native_imu=%.6f "
              "abs_start_x=%.6f abs_start_y=%.6f abs_start_h=%.6f abs_imu_h=%.6f tolerance=0.5\n",
              pose.x, pose.y, pose.theta, imu, std::fabs(pose.x), std::fabs(pose.y),
              std::fabs(std::remainder(pose.theta,360.0)),
              std::fabs(std::remainder(imu,360.0)));
  std::printf("TURN_90 CHECK following device readings are post-stop rechecks; "
              "they may differ from the original failed sample.\n");
  nav_print_sensor_health();
  std::fflush(stdout);
}

const char* turn_90_sensor_rejection(const char* phase, const char* reason) {
  // All callers are either pre-motion or after the existing explicit stop.
  turn_90_log_checks(phase, reason);
  return "SENSOR_FAULT";
}

unsigned turn_output_started = 0, turn_output_last_print = 0;
void print_turn_output() {
  const auto t = ez::turn_output_trace_get(); // Lock released before serial IO.
  std::printf("TURN_90 OUTPUT ms=%u sample_ms=%u requested_max=30 pid=%.3f slew_limit=%.3f "
              "after_slew=%.3f before_cap=%.3f turn_L=%d turn_R=%d command_L=%d command_R=%d "
              "drive_enabled=%d mode=%d zeros=%u zero_reason=%s heading=%.5f exit=%s\n",
              pros::millis()-turn_output_started,t.sampled_ms,t.pid,t.slew_limit,
              t.after_slew,t.before_cap,t.turn_left,t.turn_right,t.command_left,t.command_right,
              t.drive_enabled,t.mode,t.zero_count,t.zero_reason,chassis.drive_imu_get(),t.exit_branch);
  for (const auto* motors : {&chassis.left_motors,&chassis.right_motors})
    for (const auto& motor : *motors)
      std::printf("TURN_90 MOTOR port=%d rpm=%.3f voltage_mV=%d current_mA=%d\n",
                  motor.get_port(),motor.get_actual_velocity(),motor.get_voltage(),motor.get_current_draw());
}
bool turn_output_wait_service(ez::Drive*) {
  const auto now = pros::millis();
  if (now-turn_output_last_print >= 100) {
    turn_output_last_print = now;
    print_turn_output();
  }
  return true; // Observation only; existing wait cancellation runs before this.
}
void run_turn_90_safe() {
  ez::drive_stop(&chassis); // Cancel any previous PID before preflight/reporting.
  turn_sensor_gate.store(TurnSensorGate::None);
  turn_90_log_checks("ENTRY", "READ_ONLY_SNAPSHOT");
  auto start = nav_pose_get();
  unsigned elapsed = 0;
  const bool printing = chassis.pid_print_toggle_get();
  const char* status = [&]() -> const char* {
    if (ez::autonomous_cancelled()) return "CANCELLED";
    if (!nav_print_sensor_health())
      return turn_90_sensor_rejection("PREFLIGHT", "PRINTED_SENSOR_HEALTH_FAILED");
    if (!turn_90_sensors_ok())
      return turn_90_sensor_rejection("PREFLIGHT", turn_sensor_gate_name());

    nav_set_start(0, 0, 0);
    start = nav_pose_get();
    // nav_set_start has no return value: require healthy post-reset readback,
    // including both the odometry heading and the IMU's own heading.
    if (!turn_90_sensors_ok())
      return turn_90_sensor_rejection("AFTER_NAV_SET_START", turn_sensor_gate_name());
    if (!nav_validate_pose(0, 0, 0, 0.5, 0.5))
      return turn_90_sensor_rejection("AFTER_NAV_SET_START", "POSE_READBACK_OUTSIDE_0.5");
    if (std::fabs(std::remainder(chassis.drive_imu_get(), 360.0)) > 0.5)
      return turn_90_sensor_rejection("AFTER_NAV_SET_START", "IMU_READBACK_OUTSIDE_0.5");
    if (ez::autonomous_cancelled()) return "CANCELLED";
    std::printf("TURN_90 START X=%.3f Y=%.3f H=%.3f TARGET_H=90.000 MAX_POWER=30/127 DEADLINE_MS=5000\n",
                start.x, start.y, start.theta);
    std::fflush(stdout); // Finish serial work before arming or powering.
    chassis.pid_print_toggle(false);
    turn_velocity_trace_begin();
    const unsigned began = pros::millis();
    if (!ez::geometry_turn_deadline_arm(&chassis, turn_90_sensors_ok, 30))
      return ez::autonomous_cancelled() ? "CANCELLED" : "INTERFERED";

    turn_output_started = turn_output_last_print = pros::millis();
    ez::turn_output_trace_begin(&chassis);
    ez::wait_service_set(turn_output_wait_service);
    chassis.pid_turn_set(nav_game_to_internal_heading(90), 30);
    chassis.pid_wait();
    // FIRST action after the wait: serialized zero disables PID and then
    // disarms the deadline. No sensor read, rest wait, or report precedes it.
    ez::drive_stop(&chassis);
    elapsed = pros::millis() - began;
    ez::wait_service_set(nullptr);
    print_turn_output(); // Includes the explicit zero, always after stopping.
    const auto output_trace = ez::turn_output_trace_get();
    ez::turn_output_trace_end();
    std::printf("TURN_90 EXIT native=%s branch=%s error=%.6f derivative=%.7f secondary=%.7f "
                "k_ms=%d m_ms=%d current_ms=%d deadline_state=%d interfered=%d\n",
                ez::exit_to_string(turn_velocity_trace_result()).c_str(),output_trace.exit_branch,
                output_trace.exit_error,output_trace.exit_derivative,output_trace.exit_secondary,
                output_trace.exit_k,output_trace.exit_m,output_trace.exit_l,
                static_cast<int>(ez::geometry_turn_deadline_result()),chassis.interfered);
    const auto result = ez::geometry_turn_deadline_result();
    if (result == geometry_turn::Result::Timeout) return "TIMEOUT";
    if (result == geometry_turn::Result::Interrupted || ez::autonomous_cancelled())
      return "CANCELLED";
    if (result == geometry_turn::Result::SensorFault)
      return turn_90_sensor_rejection("AFTER_STOP", "MOTOR_OWNER_SENSOR_GUARD");
    if (!turn_90_sensors_ok())
      return turn_90_sensor_rejection("AFTER_STOP", turn_sensor_gate_name());

if (chassis.interfered)
  return "INTERFERED";

if (output_trace.exit_branch != nullptr &&
    std::strcmp(output_trace.exit_branch, "SMALL_ERROR") == 0)
  return "SUCCESS";

return "UNVERIFIED_EXIT";

  }();

  // Also covers every preflight/rejected-arm exit; do not clear the B latch.
  ez::drive_stop(&chassis);
  chassis.pid_print_toggle(printing);
  std::printf("TURN_90 STOP_COMMANDED: PID disabled, drivetrain zero requested; elapsed_motion_ms=%u\n", elapsed);
  // Rest observation is now strictly after the explicit stop.
  const bool rest_verified = nav_print_test_result("TURN_90", start, {0, 0, 90}, elapsed);
  const auto final_pose = nav_pose_get();
  if (std::strcmp(status, "SUCCESS") == 0) {
    if (ez::autonomous_cancelled()) status = "CANCELLED";
    else if (!turn_90_sensors_ok())
      status = turn_90_sensor_rejection("AFTER_REST", turn_sensor_gate_name());
    else if (!rest_verified || std::fabs(std::remainder(90 - final_pose.theta, 360.0)) > 1.0)
      status = "INTERFERED"; // A native return alone is not target qualification.
  }
  std::printf("TURN_90 STATUS=%s START=(%.3f,%.3f,%.3f) TARGET_H=90.000 "
              "FINAL=(%.3f,%.3f,%.3f) elapsed_motion_ms=%u STOP_COMMANDED=YES\n",
              status, start.x, start.y, start.theta,
              final_pose.x, final_pose.y, final_pose.theta, elapsed);
  std::fflush(stdout);
}

// Common before/after reporting keeps primitive tests easy to compare.
template <typename Motion>
void measured_move(const char* label, ez::pose target, Motion motion) {
  if (ez::autonomous_cancelled()) return;
  const auto start = nav_pose_get();
  nav_print_snapshot("BEFORE MOVE");
  const auto began = pros::millis();
  motion();
  const auto elapsed = pros::millis() - began;
  if (ez::autonomous_cancelled() || chassis.interfered) {
    std::printf("DEV MOVE FAILED: %s cancelled=%d interfered=%d\n",label,ez::autonomous_cancelled(),chassis.interfered);
    return;
  }
  nav_print_test_result(label, start, target, elapsed);
}


// GOTO_POSE uses the existing motor-owner cutoff; no extra motor writer.
int pose_xy_exit=-1, pose_angular_exit=-1;
unsigned pose_exit_evaluations=0;
void observe_pose_exit(ez::Drive* drive,int xy,int angular) {
  // Invoked on the native waiting task, not under the motor-owner lock.
  if(drive!=&chassis) return;
  pose_xy_exit=xy; pose_angular_exit=angular; ++pose_exit_evaluations;
}
const char* pose_exit_name(int value) {
  if(value<0) return "UNOBSERVED";
  switch(static_cast<ez::exit_output>(value)) {
    case ez::RUNNING:return "RUNNING";
    case ez::SMALL_EXIT:return "SMALL";
    case ez::BIG_EXIT:return "BIG";
    case ez::VELOCITY_EXIT:return "VELOCITY";
    case ez::mA_EXIT:return "OVER_CURRENT";
    case ez::ERROR_NO_CONSTANTS:return "NO_CONSTANTS";
  }
  return "UNKNOWN";
}
bool pose_exit_success(int value) {
  return value==static_cast<int>(ez::SMALL_EXIT) || value==static_cast<int>(ez::BIG_EXIT);
}
bool pose_sensors_ready() { return chassis.odom_enabled() && nav_sensors_ready(); }
void run_goto_pose_safe() {
  ez::drive_stop(&chassis);
  if(ez::autonomous_cancelled() || !pose_sensors_ready()) {
    std::printf("GOTO_POSE NOT STARTED: cancelled/sensor fault; STOP_COMMANDED=YES\n");
    return;
  }
  // Do not modify either navigation lookahead. Reject unexpected configuration.
  if(!std::isfinite(chassis.odom_look_ahead_get()) ||
     std::fabs(chassis.odom_look_ahead_get()-7.0)>0.000001) {
    std::printf("GOTO_POSE NOT STARTED: expected unchanged 7-inch lookahead; STOP_COMMANDED=YES\n");
    return;
  }
  nav_set_start(0,0,0);
  if(!pose_sensors_ready() || !nav_validate_pose(0,0,0,0.5,0.5)) {
    std::printf("GOTO_POSE NOT STARTED: start readback failed; STOP_COMMANDED=YES\n");
    return;
  }
  std::printf("GOTO_POSE requested=(-12,0,270) speed=40 lookahead=7 in cutoff=5000 ms\n");
  std::fflush(stdout);
  NavSequence sequence(true,5000);
  if(!sequence.ok()) return;
  pose_xy_exit=pose_angular_exit=-1; pose_exit_evaluations=0;
  ez::native_odom_exit_observer_set(observe_pose_exit);
  const bool armed=ez::geometry_turn_deadline_arm(&chassis,pose_sensors_ready);
  const unsigned began=pros::millis();
  const bool returned=armed && nav_go_to_pose(-12,0,270,40,NavEndMode::SETTLE);
  ez::drive_stop(&chassis); // Disable native PID before rest checks/reporting.
  const unsigned motion_ms=pros::millis()-began;
  const auto deadline=ez::geometry_turn_deadline_result();
  ez::native_odom_exit_observer_set(nullptr);
  ez::navigation_lock();
  const double internal_error=chassis.current_a_odomPID.error;
  ez::navigation_unlock();
  const bool finished=sequence.finish(); // Existing bounded 2 s rest check.
  const auto final=nav_pose_get();
  const double xy=std::hypot(final.x+12.0,final.y);
  const double heading=std::remainder(270.0-final.theta,360.0);
  const bool cancelled=ez::autonomous_cancelled() || deadline==geometry_turn::Result::Interrupted;
  const bool timeout=deadline==geometry_turn::Result::Timeout || std::strcmp(nav_fault(),"movement timeout")==0;
  const bool sensors=pose_sensors_ready() && deadline!=geometry_turn::Result::SensorFault;
  const bool rested=nav_sequence_rest_verified();
  const bool accepted=armed && returned && finished && rested && sensors && !cancelled && !timeout &&
      !chassis.interfered && pose_exit_success(pose_xy_exit) && pose_exit_success(pose_angular_exit) &&
      std::isfinite(xy) && std::isfinite(heading) && xy<=0.75 && std::fabs(heading)<=3.0;
  std::printf("GOTO_POSE status=%s requested=(-12,0,270) final=(%.5f,%.5f,%.5f) "
              "xy_error_in=%.5f final_heading_error_deg=%.5f internal_angular_pid_error_deg=%.5f\n"
              "native_xy=%s native_angular=%s exit_evaluations=%u motion_ms=%u "
              "armed=%d timeout=%d cancelled=%d interfered=%d sensors_ok=%d rest_verified=%d "
              "fault=%s STOP_COMMANDED=YES\n",
              accepted?"PASS":"FAIL",final.x,final.y,final.theta,xy,heading,internal_error,
              pose_exit_name(pose_xy_exit),pose_exit_name(pose_angular_exit),pose_exit_evaluations,motion_ms,
              armed,timeout,cancelled,chassis.interfered,sensors,rested,nav_fault());
  
std::printf(
    "STAGE1_GATE armed=%d returned=%d finished=%d rested=%d "
    "sensors=%d cancelled=%d timeout=%d interfered=%d "
    "native_xy=%s native_angular=%s fault=%s\n",
    armed, returned, finished, rested,
    sensors, cancelled, timeout, chassis.interfered,
    pose_exit_name(pose_xy_exit),
    pose_exit_name(pose_angular_exit),
    nav_fault());
std::fflush(stdout);

// nav_trace_dump(); // Temporarily disabled to keep serial output readable.
  // Diagnostic-only second stage: never retry or correct a failed first movement.
  const bool stage1_safe=armed && returned && finished && rested && sensors && !cancelled && !timeout &&
      !chassis.interfered && pose_exit_success(pose_xy_exit) && pose_exit_success(pose_angular_exit);
const bool correction_needed=stage1_safe && std::isfinite(xy) && std::isfinite(heading) && xy<=1.0 && std::fabs(heading)>3.0;  bool correction_pass=false;
  ez::pose combined=final;
  if(correction_needed && !ez::autonomous_cancelled() && pose_sensors_ready()) {
    // Stage 1 finish released the hooks and disarmed its watchdog. No pose reset.
    NavSequence correction(true,5000);
    const bool turn_armed=correction.ok() &&
        ez::geometry_turn_deadline_arm(&chassis,pose_sensors_ready,30);
    ez::turn_output_trace_begin(&chassis);
    const unsigned turn_began=pros::millis();
    const bool turn_returned=turn_armed && nav_turn_to(270,30,NavEndMode::SETTLE);
    ez::drive_stop(&chassis);
    const unsigned turn_ms=pros::millis()-turn_began;
    const auto turn_deadline=ez::geometry_turn_deadline_result();
    const auto turn_trace=ez::turn_output_trace_get();
    ez::turn_output_trace_end();
    const bool turn_finished=correction.finish();
    combined=nav_pose_get();
    const double turn_heading=std::remainder(270.0-combined.theta,360.0);
    const bool turn_cancelled=ez::autonomous_cancelled() ||
        turn_deadline==geometry_turn::Result::Interrupted;
    const bool turn_timeout=turn_deadline==geometry_turn::Result::Timeout ||
        std::strcmp(nav_fault(),"movement timeout")==0;
    const bool turn_sensors=pose_sensors_ready() &&
        turn_deadline!=geometry_turn::Result::SensorFault;
    const bool turn_rested=nav_sequence_rest_verified();
    const bool turn_native_ok=std::strcmp(turn_trace.exit_branch,"SMALL_ERROR")==0 ||
        std::strcmp(turn_trace.exit_branch,"BIG_ERROR")==0;
    correction_pass=turn_armed && turn_returned && turn_finished && turn_rested &&
        turn_sensors && !turn_cancelled && !turn_timeout && !chassis.interfered &&
        turn_native_ok && std::isfinite(turn_heading) && std::fabs(turn_heading)<=3.0;
    std::printf("GOTO_POSE HEADING_CORRECTION status=%s target_deg=270 speed=30 cutoff_ms=5000 "
                "motion_ms=%u armed=%d native_branch=%s native_error_deg=%.5f "
                "final_heading_error_deg=%.5f timeout=%d cancelled=%d interfered=%d "
                "sensors_ok=%d rest_verified=%d fault=%s STOP_COMMANDED=YES\n",
                correction_pass?"PASS":"FAIL",turn_ms,turn_armed,turn_trace.exit_branch,
                turn_trace.exit_error,turn_heading,turn_timeout,turn_cancelled,chassis.interfered,
                turn_sensors,turn_rested,nav_fault());
// nav_trace_dump(); // Temporarily disabled.  } else {
    std::printf("GOTO_POSE HEADING_CORRECTION SKIPPED: %s\n",
                accepted?"already within final-pose tolerances":
                correction_needed?"live safety gate rejected":"stage 1 failed safety/completion/XY gate");
  }
  const double combined_xy=std::hypot(combined.x+12.0,combined.y);
  const double combined_heading=std::remainder(270.0-combined.theta,360.0);
  const double drift_x=combined.x-final.x, drift_y=combined.y-final.y;
  const bool combined_pass=(accepted || correction_pass) && !ez::autonomous_cancelled() &&
      pose_sensors_ready() && !chassis.interfered && std::isfinite(combined_xy) &&
      std::isfinite(combined_heading) && combined_xy<=0.75 && std::fabs(combined_heading)<=3.0;
  std::printf("GOTO_POSE COMBINED status=%s boomerang_only=%s final=(%.5f,%.5f,%.5f) "
              "xy_error_in=%.5f heading_error_deg=%.5f turn_xy_drift_in=%.5f "
              "turn_dx_in=%.5f turn_dy_in=%.5f\n",
              combined_pass?"PASS":"FAIL",accepted?"PASS":"FAIL",combined.x,combined.y,combined.theta,
              combined_xy,combined_heading,std::hypot(drift_x,drift_y),drift_x,drift_y);

}


void run_boomerang_only_test() {



constexpr double TARGET_X = 24.0;
constexpr double TARGET_Y = 24.0;

// The heading we want the robot to reach.
constexpr double DESIRED_HEADING = 45;

// Temporary compensation for right-diagonal Boomerang.
constexpr double HEADING_COMPENSATION = 4;

// Heading sent to the Boomerang controller.
constexpr double TARGET_H =
    DESIRED_HEADING + HEADING_COMPENSATION;

  std::printf(  
      "TEST B FIRMWARE VERIFIED: TARGET=(%.1f, %.1f, %.1f)\n",
      TARGET_X, TARGET_Y, TARGET_H);
  std::fflush(stdout);

  constexpr int SPEED = 127;
  constexpr unsigned TIMEOUT_MS = 5000;

  ez::drive_stop(&chassis);

  if (ez::autonomous_cancelled() || !pose_sensors_ready() ||
      !std::isfinite(chassis.odom_look_ahead_get()) ||
      std::fabs(chassis.odom_look_ahead_get() - 7.0) > 0.000001) {
    std::printf("BOOMERANG TEST REJECTED: preflight failed\n");
    return;
  }

  nav_set_start(0, 0, 0);
  if (!pose_sensors_ready() ||
      !nav_validate_pose(0, 0, 0, 0.5, 0.5)) {
    std::printf("BOOMERANG TEST REJECTED: start pose invalid\n");
    return;
  }

  NavSequence sequence(true, TIMEOUT_MS);
  if (!sequence.ok()) return;

  pose_xy_exit = pose_angular_exit = -1;
  pose_exit_evaluations = 0;
  ez::native_odom_exit_observer_set(observe_pose_exit);

  const bool armed =
      ez::geometry_turn_deadline_arm(&chassis, pose_sensors_ready);

  const unsigned start_ms = pros::millis();

  const bool returned = armed &&
      nav_go_to_pose(TARGET_X, TARGET_Y, TARGET_H,
                     SPEED, NavEndMode::SETTLE);

  ez::drive_stop(&chassis);

  const unsigned elapsed = pros::millis() - start_ms;
  const auto deadline = ez::geometry_turn_deadline_result();

  ez::native_odom_exit_observer_set(nullptr);

  const bool finished = sequence.finish();
  const bool rested = nav_sequence_rest_verified();
  const auto final = nav_pose_get();

  const double xy_error =
      std::hypot(TARGET_X - final.x, TARGET_Y - final.y);

  const double heading_error =
      std::remainder(TARGET_H - final.theta, 360.0);

  const bool safe =
      armed && returned && finished && rested &&
      !ez::autonomous_cancelled() && !chassis.interfered &&
      pose_sensors_ready() &&
      deadline != geometry_turn::Result::Timeout &&
      deadline != geometry_turn::Result::Interrupted &&
      deadline != geometry_turn::Result::SensorFault &&
      pose_exit_success(pose_xy_exit) &&
      pose_exit_success(pose_angular_exit);

  const bool qualified =
      safe && std::isfinite(xy_error) &&
      std::isfinite(heading_error) &&
      xy_error <= 0.75 &&
      std::fabs(heading_error) <= 3.0;

  std::printf(
      "\n===== BOOMERANG ONLY RESULT =====\n"
      "Target: (%.3f, %.3f, %.3f)\n"
      "Final: (%.3f, %.3f, %.3f)\n"
      "XY error: %.4f in\n"
      "Heading error: %.4f deg\n"
      "Elapsed: %u ms\n"
      "Native XY exit: %s\n"
      "Native angular exit: %s\n"
      "Exit evaluations: %u\n"
      "Armed: %d Returned: %d Finished: %d\n"
      "Rest verified: %d Interfered: %d\n"
      "Deadline state: %d\n"
      "Safe completion: %s\n"
      "Odometry qualification: %s\n"
      "No terminal-heading correction commanded\n"
      "================================\n",
      TARGET_X, TARGET_Y, TARGET_H,
      final.x, final.y, final.theta,
      xy_error, heading_error, elapsed,
      pose_exit_name(pose_xy_exit),
      pose_exit_name(pose_angular_exit),
      pose_exit_evaluations,
      armed, returned, finished,
      rested, chassis.interfered,
      static_cast<int>(deadline),
      safe ? "YES" : "NO",
      qualified ? "PASS" : "FAIL"
  );

  std::fflush(stdout);
  nav_trace_dump();
}


void point(const char* label, double x, double y) {
  measured_move(label, {x, y, ez::ANGLE_NOT_SET},
                [=] { nav_go_to(x, y, 40); });
}

}

// ----- Primitive test dispatch -----
// Run the selected foundation test; real route stages use the separate segment harness.
void run_dev_test(DevTest test) {
  if (test == DevTest::FULL_SYSTEM_DIAG) { run_full_system_diag(); return; }
  if (test == DevTest::STABILITY_MONITOR) {
    telemetry_monitor_test();
    return;
  }
  const char* name = test_name(test);
  std::printf("\n=== DEV TEST START: %s ===\n", name);
  // Scope marker also covers all safe early returns.
  struct EndMarker {
    const char* name;
    ~EndMarker() {
      std::printf("=== DEV TEST END: %s ===\n", name);
      std::fflush(stdout);
    }
  } end{name};
  if (test == DevTest::SENSOR_HEALTH || test == DevTest::MANUAL_ODOM) {
    nav_print_sensor_health();
    std::printf("30-second read-only window; NO pose/sensor reset. NaN means unavailable.\n");
    nav_manual_odom_sample(false);
    const auto began = pros::millis();
    while (pros::millis() - began < 30000 && !pros::competition::is_disabled()) {
      nav_manual_odom_sample(true);
      pros::delay(50);
    }
    nav_manual_odom_sample(false);
    return;
  }
  if (std::strcmp(name, "UNKNOWN") == 0) {
    std::printf("UNKNOWN DEV TEST: no movement dispatched.\n");
    return;
  }
  if (test == DevTest::MIRROR_MATH) {
    auton_actions::run_helper_math_test();
    return;
  }
  if (test == DevTest::TURN_90) { run_turn_90_safe(); return; }
  if (test == DevTest::REVERSE_RIGHT_DIAG) { run_reverse_right_diag(); return; }
if (test == DevTest::GOTO_POSE) {
  run_boomerang_only_test();
  return;
}  nav_print_sensor_health(); // Diagnostic only.
  nav_set_start(0, 0, 0);
  switch (test) {
    case DevTest::START_POSE:
      nav_print_snapshot("START POSE");
      return;
    case DevTest::DRIVE_FORWARD_12:
      drive_diagnostic_arm();
      measured_move(name, {0, 12, ez::ANGLE_NOT_SET}, [] {  nav_turn_to(90, 30); });
      break;
    case DevTest::GOTO_Y_12: point(name, 0, 12); break; 
    case DevTest::GOTO_X_12:
      point(name, -24, 0);
      break;

    case DevTest::REVERSE_12:
      std::printf("Expected: BACK leads toward Y=-12; FRONT remains facing approximately +Y.\n");
      measured_move(name, {0, -12, ez::ANGLE_NOT_SET}, [] { nav_go_to_reverse(0, -12, 40); });
      break;
    case DevTest::RETURN_TO_ORIGIN:
      point("RETURN_TO_ORIGIN outbound", 0, 12);
      point("RETURN_TO_ORIGIN return FRONT-first", 0, 0);
      break;
    case DevTest::TWO_POINT:
      point("TWO_POINT leg 1", 0, 12);
      point("TWO_POINT leg 2", 12, 12);
      break;
    case DevTest::SQUARE:
      // Later foundation test: ordinary blocking point-to-point wrappers, no chaining.
      point("SQUARE leg 1", 0, 12);
      point("SQUARE leg 2", 12, 12);
      point("SQUARE leg 3", 12, 0);
      point("SQUARE leg 4", 0, 0);
      break;

      


    case DevTest::GOTO_POSE: break; // Handled by bounded harness above.

    case DevTest::CHAIN_TWO_MOVE:
      auton_two_move_chain_test(); // Existing geometry, speed, tracing and chain settings.
      break;
    case DevTest::PURE_PURSUIT_BASIC:
      measured_move(name, {12, 24, ez::ANGLE_NOT_SET}, [] {
        if (!nav_follow_path({{0, 12}, {12, 24}}, 40))
          std::printf("PURE PURSUIT REQUEST REJECTED\n");
      });                                       
      break;
    default: break;
  }
}
