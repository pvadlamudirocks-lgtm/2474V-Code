/*
 * Native exit functions copied from EZ-Template v3.2.2 src/EZ-Template/PID.cpp.
 * Mozilla Public License v2.0: https://mozilla.org/MPL/2.0/
 * Only names and observation hooks differ; decision logic remains native.
 */
#include "motion_exit_trace.hpp"
#include <algorithm>
#include <cstdio>

using namespace ez;
namespace {
std::uint32_t trace_began = 0;
int trace_max_k = 0, trace_max_m = 0;
bool trace_printed = false;
ez::exit_output last_turn_exit = ez::RUNNING;

void trace_capture(const PID& pid, int k, int m, double used_accel, bool primary) {
  if (trace_printed) return;
  trace_printed = true;
  const auto elapsed = pros::millis() - trace_began;
  const double current = pid.cur;
  const double change = pid.derivative;
  const double target = pid.target;
  const double error = pid.error;
  const auto accel = chassis.imu.get_accel();
  std::printf("\n===== TURN VELOCITY EXIT TRACE =====\n"
              "branch = %s\nelapsed turn time = %lu ms\n"
              "current PUBLIC heading = %.5f\ncurrent EZ heading = %.5f\n"
              "target EZ heading = %.5f\nremaining angular error = %.5f\n\n"
              "current heading = %.5f\nprevious heading = %.5f (current - derivative)\n"
              "derivative/change per update = %.7f\nprimary counter k = %d ms\n\n"
              "imu accel.x = %.7f\nimu accel.y = %.7f\naccel.x + accel.y = %.7f\n"
              "secondary input actually tested = %.7f\nsecondary counter m = %d ms\n"
              "maximum k = %d ms\nmaximum m = %d ms\n"
              "Heading values are the PID sample; accel axes are read at exit.\n"
              "================================\n",
              primary ? "PRIMARY HEADING-CHANGE VELOCITY EXIT" : "SECONDARY IMU-ACCEL VELOCITY EXIT",
              static_cast<unsigned long>(elapsed), nav_internal_to_game_heading(current), current,
              target, error, current, current - change, change, k,
              accel.x, accel.y, accel.x + accel.y, used_accel, m, trace_max_k, trace_max_m);
  std::fflush(stdout);
}
}  // namespace

void turn_velocity_trace_begin() {
  last_turn_exit = ez::RUNNING;
  trace_began = pros::millis();
  trace_max_k = trace_max_m = 0;
  trace_printed = false;
}

void turn_velocity_trace_record_result(ez::exit_output result) {
  last_turn_exit = result;
  final_turn_trace_exit(result);
}
ez::exit_output turn_velocity_trace_result() { return last_turn_exit; }

exit_output PID::exit_condition_turn_trace(bool print) {
  // If this function is called while all exit constants are 0, print an error
  if (exit.small_error == 0 && exit.small_exit_time == 0 && exit.big_error == 0 && exit.big_exit_time == 0 && exit.velocity_exit_time == 0 && exit.mA_timeout == 0) {
    exit_condition_print(ERROR_NO_CONSTANTS);
    return ERROR_NO_CONSTANTS;
  }

  // If the robot gets within the target, make sure it's there for small_timeout amount of time
  if (exit.small_error != 0) {
    if (abs(error) < exit.small_error) {
      j += util::DELAY_TIME;
      i = 0;  // While this is running, don't run big thresh
      if (j > exit.small_exit_time) {
        timers_reset();
        if (print) exit_condition_print(SMALL_EXIT);
        return SMALL_EXIT;
      }
    } else {
      j = 0;
    }
  }

  // If the robot is close to the target, start a timer.  If the robot doesn't get closer within
  // a certain amount of time, exit and continue.  This does not run while small_timeout is running
  else if (exit.big_error != 0 && exit.big_exit_time != 0) {  // Check if this condition is enabled
    if (abs(error) < exit.big_error) {
      i += util::DELAY_TIME;
      if (i > exit.big_exit_time) {
        timers_reset();
        if (print) exit_condition_print(BIG_EXIT);
        return BIG_EXIT;
      }
    } else {
      i = 0;
    }
  }

  // If the motor velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (abs(derivative) <= velocity_zero_main) {
      k += util::DELAY_TIME;
      trace_max_k = std::max(trace_max_k, k);
      if (k > exit.velocity_exit_time) {
        trace_capture(*this, k, m, second_sensor, true);
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      k = 0;
    }
  }

  if (!use_second_sensor)
    return RUNNING;

  // If the secondary sensors velocity is 0, the code will timeout and set interfered to true.
  if (exit.velocity_exit_time != 0) {  // Check if this condition is enabled
    if (abs(second_sensor) <= velocity_zero_secondary) {
      m += util::DELAY_TIME;
      trace_max_m = std::max(trace_max_m, m);
      if (m > exit.velocity_exit_time) {
        trace_capture(*this, k, m, second_sensor, false);
        timers_reset();
        if (print) exit_condition_print(VELOCITY_EXIT);
        return VELOCITY_EXIT;
      }
    } else {
      m = 0;
    }
  }

  // printf("j: %i   i: %i   k: %i   m: %i\n", j, i, k, m);

  return RUNNING;
}

exit_output PID::exit_condition_turn_trace(std::vector<pros::Motor> sensor, bool print) {
  // If the motors are pulling too many mA, the code will timeout and set interfered to true.
  if (exit.mA_timeout != 0) {  // Check if this condition is enabled
    for (auto i : sensor) {
      // Check if 1 motor is pulling too many mA
      if (i.is_over_current()) {
        is_mA = true;
        break;
      }
      // If all of the motors aren't drawing too many mA, keep bool false
      else {
        is_mA = false;
      }
    }
    if (is_mA) {
      l += util::DELAY_TIME;
      if (l > exit.mA_timeout) {
        timers_reset();
        if (print) exit_condition_print(mA_EXIT);
        return mA_EXIT;
      }
    } else {
      l = 0;
    }
  }

  return exit_condition_turn_trace(print);
}

