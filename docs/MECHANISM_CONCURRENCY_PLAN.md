# Mechanism concurrency plan

All commands/updates belong to one controlling task. opcontrol calls mechanisms_update
every 10 ms; blocking lift/arm target wrappers service it while waiting. NavSequence
already installs the same-task EZ wait-service callback and updates mechanisms during
motion/rest waits. Ordinary trusted nav_go_to/nav_turn_to wrappers block without
mechanism service. Session's ordinary motion watchdog adds sensor/deadline checks
only; it deliberately does not change mechanism ownership.

Claw pneumatic output persists without update. Flex and toggle motor commands latch
until explicit stop/change; they can run through ordinary blocking nav. Their write
success does not provide completion feedback. Full-voltage lift output also latches,
but bottom/soft-limit checks need updates, so it must NOT be started then abandoned
during a blocking ordinary move. Session rejects active lift/arm target/hold/manual/
full-output states before ordinary navigation. No independent watchdog stops a lift
while its owner is blocked. Cooperative stale-update detection acts only on resumption.

For future lift overlap, qualify existing NavSequence/NavEvents first: construct the
sequence, arm a short mechanism event, begin a calibrated target in that event, and
keep its same-task wait service running. Require an explicit feedback/target result
before scoring. Monitor faults; an event's return true means accepted, not later
target completion. No callback may delay, run nested navigation, or call another
blocking axis wrapper. Finish stops all mechanisms including hold; plan stage boundaries.
Session.sequence uses this path behind the existing disabled advanced/mechanism gates.

Existing lift_set_target is nonblocking. Its controller transitions to Hold at the
same requested target. New LiftFinish::HOLD_CURRENT_TARGET on the blocking wrapper
is rejected until hold_validated=true; afterward it still requires continuous
same-task update service. Default STOP retains the current zero-output behavior.
No extra gain or voltage was chosen. New full-command soft limits are checked on
write and subsequent updates only after explicit calibration/enablement.

Do not add an independent mechanism task now. If a future redesign needs one,
transfer ALL axis writes, homing, target updates, stops and sensor state to that
task; pass commands through a bounded mailbox, use generation IDs to discard stale
requests, acknowledge stop, and make disable an immediate owner override. Eliminate
driver/autonomous direct writers before activation. Mixing a task with current
writers would allow PID/full-voltage/stop commands to overwrite each other.

Field tests: missed update, blocked target, sensor disconnect, rejected event,
disable during rise, mode transition, sequence finish while holding, loaded travel,
and stop-write fault. Validate one axis alone before overlap. No concurrent lift
behavior was physically qualified in this task.
