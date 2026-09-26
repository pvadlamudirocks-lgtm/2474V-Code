#include "motion_exit_trace.hpp"
#include <cstdio>
#include <atomic>
#include <cmath>

namespace {
struct HeadingSample {
  double game = 0, ez = 0, imu = 0;
};
struct FinalTurnTrace {
  HeadingSample before, exit, wait, returned;
  double requested = 0, target = 0;
  ez::exit_output reason = ez::RUNNING;
  bool exit_seen = false;
};
FinalTurnTrace trace;
bool active = false;
std::atomic<bool> low_error_sampling{false};
std::atomic<unsigned> sampling_generation{0};

void low_error_samples(unsigned generation, std::uint32_t began) {
  double previous_heading = chassis.drive_imu_get();
  while (low_error_sampling.load() && sampling_generation.load() == generation) {
    pros::delay(100);
    if (!low_error_sampling.load() || sampling_generation.load() != generation) return;
    if (chassis.drive_mode_get() != ez::TURN) continue;
    const auto elapsed = pros::millis() - began;
    const double heading = chassis.drive_imu_get();
    const double ez_heading = chassis.odom_theta_get();
    const double target = chassis.turnPID.target;
    const double error = chassis.turnPID.error;
    const double integral = chassis.turnPID.integral;
    const double derivative = chassis.turnPID.derivative;
    const double output = chassis.turnPID.output;
    const auto gains = chassis.turnPID.constants;
    const double slew_limit = chassis.slew_turn.output();
    const int turn_min = chassis.pid_turn_min_get();
    const bool enabled = chassis.pid_drive_toggle_get();
    const auto left_voltage = chassis.left_motors[0].get_voltage();
    const auto right_voltage = chassis.right_motors[0].get_voltage();
    const double change = heading - previous_heading;
    previous_heading = heading;
    if (std::fabs(target - heading) >= 10.0) continue;

    // Reconstruct local turn_pid_task() requests without compute/iterate or writes.
    double request = ez::util::clamp(output, slew_limit, -slew_limit);
    if (gains.ki != 0 && std::fabs(target) > gains.start_i &&
        std::fabs(error) < gains.start_i && turn_min != 0)
      request = ez::util::clamp(request, turn_min, -turn_min);
    std::printf("\n===== LOW-ERROR TURN PID TRACE =====\n\n"
                "elapsed turn time = %lu ms\nPUBLIC heading = %.6f\nEZ heading = %.6f\n"
                "target EZ heading = %.6f\nangular error = %.6f\n\n"
                "TURN PID error = %.7f\n"
                "TURN PID P contribution = UNAVAILABLE (not stored by EZ)\n"
                "TURN PID I contribution = UNAVAILABLE (not stored by EZ)\n"
                "TURN PID D contribution = UNAVAILABLE (not stored by EZ)\n"
                "TURN PID integral state = %.7f\nTURN PID derivative state = %.7f\n"
                "TURN PID gains P/I/D = %.6f %.6f %.6f\nfinal TURN PID output = %.7f\n\n"
                "left request (reconstructed, before integer conversion) = %.7f\n"
                "right request (reconstructed, before integer conversion) = %.7f\n"
                "exact last commanded output = UNAVAILABLE (EZ local variable)\n"
                "PID output enabled = %s\nslew limit = %.5f\nturn minimum limit = %d\n\n"
                "actual left motor voltage (first left motor, mV) = %ld\n"
                "actual right motor voltage (first right motor, mV) = %ld\n\n"
                "heading change since previous 100 ms sample = %.7f\n"
                "===================================\n",
                static_cast<unsigned long>(elapsed), nav_internal_to_game_heading(ez_heading), ez_heading,
                target, target - heading, error, integral, derivative, gains.kp, gains.ki, gains.kd,
                output, request, -request, enabled ? "true" : "false", slew_limit, turn_min,
                static_cast<long>(left_voltage), static_cast<long>(right_voltage), change);
    std::fflush(stdout);
  }
}

HeadingSample sample() {
  const double heading = chassis.odom_theta_get();
  return {nav_internal_to_game_heading(heading), heading, chassis.drive_imu_get()};
}
}  // namespace

void final_turn_trace_arm(double public_target) {
  trace = {};
  trace.requested = public_target;
  trace.before = sample();
  active = true;
  const unsigned generation = ++sampling_generation;
  const auto began = pros::millis();
  low_error_sampling.store(true);
  pros::Task sampler([generation, began]() { low_error_samples(generation, began); }, "Low error turn trace");
}
void final_turn_trace_target(double ez_target) {
  if (active) trace.target = ez_target;
}
void final_turn_trace_exit(ez::exit_output result) {
  if (!active || trace.exit_seen || result == ez::RUNNING) return;
  // Existing hook: first native exit evaluation returning a non-RUNNING result.
  trace.exit = sample();
  trace.reason = result;
  trace.exit_seen = true;
}
void final_turn_trace_wait_returned() {
  low_error_sampling.store(false);
  if (active) trace.wait = sample();
}
void final_turn_trace_nav_returned() {
  if (!active) return;
  const auto returned_ms = pros::millis();
  trace.returned = sample();
  const auto frozen = trace;
  active = false;
  // Independent read-only task: no added wait or motor command in autonomous.
  pros::Task reporter([frozen, returned_ms]() {
    HeadingSample later[3];
    std::uint32_t actual_ms[3];
    const std::uint32_t requested_ms[] = {100, 250, 500};
    for (int i = 0; i < 3; ++i) {
      const auto elapsed = pros::millis() - returned_ms;
      if (elapsed < requested_ms[i]) pros::delay(requested_ms[i] - elapsed);
      actual_ms[i] = pros::millis() - returned_ms;
      later[i] = sample();
    }
    std::printf("\n===== FINAL TURN TRACE =====\n\n"
                "PUBLIC heading before nav_turn_to = %.5f\nEZ heading before nav_turn_to = %.5f\n\n"
                "requested PUBLIC target = %.3f\nconverted EZ target passed to pid_turn_set = %.5f\n\n",
                frozen.before.game, frozen.before.ez, frozen.requested, frozen.target);
    if (frozen.exit_seen)
      std::printf("PUBLIC heading when TURN exit first triggers = %.5f\n"
                  "EZ heading when TURN exit first triggers = %.5f\n"
                  "IMU rotation when TURN exit first triggers = %.5f\n\n",
                  frozen.exit.game, frozen.exit.ez, frozen.exit.imu);
    else
      std::printf("NO TURN EXIT EVENT CAPTURED\n\n");
    std::printf("PUBLIC heading when pid_wait returns = %.5f\nEZ heading when pid_wait returns = %.5f\n\n"
                "PUBLIC heading immediately when nav_turn_to returns = %.5f\n\n",
                frozen.wait.game, frozen.wait.ez, frozen.returned.game);
    for (int i = 0; i < 3; ++i)
      std::printf("PUBLIC heading +%lu ms later = %.5f\nIMU rotation +%lu ms later = %.5f\n"
                  "actual sample elapsed = %lu ms\n",
                  static_cast<unsigned long>(requested_ms[i]), later[i].game,
                  static_cast<unsigned long>(requested_ms[i]), later[i].imu,
                  static_cast<unsigned long>(actual_ms[i]));
    std::printf("\nturn exit reason = %s\n================================\n",
                frozen.exit_seen ? ez::exit_to_string(frozen.reason).c_str() : "NO EXIT EVENT CAPTURED");
    std::fflush(stdout);
  }, "Final turn trace");
}
