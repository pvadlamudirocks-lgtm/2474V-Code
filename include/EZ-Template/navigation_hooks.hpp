#pragma once
namespace ez {
class Drive;
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
