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
// Geometry diagnostic only. Existing controller enforces this independently of waits.
bool geometry_turn_deadline_arm(Drive* drive);
geometry_turn::Result geometry_turn_deadline_result();
bool geometry_turn_deadline_stopped(Drive* drive);
}
