#pragma once
namespace ez {
class Drive;
enum class DiagnosticEvent { BEFORE_OUTPUT, AFTER_OUTPUT, STOP };
// Invoked only by the existing drivetrain owner under its navigation lock.
using DiagnosticService = bool (*)(Drive*, DiagnosticEvent);
bool diagnostic_service_set(DiagnosticService service);
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
bool navigation_try_lock(unsigned timeout_ms);
}

namespace ez {
// Basic execution protection in the existing controller task. No per-test policy.
using MovementHealth = bool (*)();
void movement_health_set(MovementHealth);
bool movement_begin_autonomous();
void movement_cancel(const char* reason); // static-lifetime reason; no mutex
bool movement_stop(); // bounded acknowledgement; controller owns the zero writes
bool movement_ready();
const char* movement_fault();
}
