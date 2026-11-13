#pragma once
#include "geometry_turn_deadline.hpp"
namespace ez {
class Drive;
using DriveTelemetryObserver = void (*)(Drive*, int left, int right);
void drive_telemetry_observer_set(DriveTelemetryObserver observer);
using WaitService = bool (*)(Drive*);
void wait_service_set(WaitService service);
// Invoked by wait loops on their caller's task. False cancels that wait.
bool wait_service_tick(Drive* drive);
using CommandObserver = void (*)(Drive*, int, int);
void command_observer_set(CommandObserver observer);
}

namespace ez {
// Serializes controller target installation with the existing single control task.
// Does not write zero or alter the brake mode.
void navigation_lock();
void navigation_unlock();
}

namespace ez {
// Basic lifecycle only; no sensor preflight, deadline, ownership gate or ACK.
void drive_stop(Drive* drive); // Serialized native drive_set(0,0), disables PID.
void autonomous_begin();      // Clear the B-button emergency latch for a new run.
void autonomous_emergency_stop(); // Atomic request; controller performs stop.
bool autonomous_cancelled();  // Competition mode/disable or autonomous-only B.
// Explicit turn diagnostics only. Enforced by the motor owner independently of waits.
// Optional health callback must be read-only, silent, and must not take navigation_lock.
using TurnSensorCheck = bool (*)();
bool geometry_turn_deadline_arm(Drive* drive, TurnSensorCheck sensors_ok = nullptr, int max_power = 127);
bool geometry_turn_deadline_active(Drive* drive);
geometry_turn::Result geometry_turn_deadline_result();
bool geometry_turn_deadline_stopped(Drive* drive);
}

namespace ez {
class PID;
// Opt-in TURN_90 observation only. No motor commands or terminal IO.
struct TurnOutputTrace {
  unsigned sampled_ms = 0, zero_count = 0;
  double pid = 0, slew_limit = 0, after_slew = 0, before_cap = 0;
  int turn_left = 0, turn_right = 0, command_left = 0, command_right = 0;
  bool drive_enabled = false;
  int mode = 0;
  const char* zero_reason = "NONE";
  const char* exit_branch = "RUNNING";
  double exit_error = 0, exit_derivative = 0, exit_secondary = 0;
  int exit_k = 0, exit_m = 0, exit_l = 0;
};
void turn_output_trace_begin(Drive* drive);
void turn_output_trace_end();
TurnOutputTrace turn_output_trace_get();
void turn_output_trace_exit(const PID* pid, const char* branch, double secondary, int k, int m, int l);
}

namespace ez {
// Observation only: native PTP wait publishes its already-evaluated/latching
// results. Callback must not print, drive, or reevaluate an exit condition.
using NativeOdomExitObserver = void (*)(Drive*, int xy, int angular);
void native_odom_exit_observer_set(NativeOdomExitObserver observer);
// Last native pid_wait result for this drive, independent of optional callbacks.
// Reset at wait entry; completed is published only at the end of the odometry branch.
struct NativeOdomWaitResult { bool completed = false; int xy = -1, angular = -1; };
NativeOdomWaitResult native_odom_wait_result(Drive* drive);
}
