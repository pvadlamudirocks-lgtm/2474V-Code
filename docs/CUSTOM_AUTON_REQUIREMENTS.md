# Six-command custom autonomous — 2026-11-09

The six calls are directly in src/main.cpp; there is no route runner or executor.
The existing autonomous entry sets the drivetrain reference to (0,0,0).
This does not reset P16. Physically align the robot to that reference before running.

pulse_toggle() lives in subsystems.cpp and runs one cancellable 395 ms pulse,
checking configured motor connectivity and explicitly stopping afterward.
Lift functions retain calibration, homing, limits, feedback and timeout checks.
Safely stopped timeout/update-gap outcomes do not cancel later commands.
Hard lift failures and Toggle failures latch the existing autonomous cancellation.
Claw calls honor that cancellation during autonomous; driver mapping is unchanged.

nav_go_to() keeps its 3-inch lookahead and existing controller. It uses the existing
5-second motor-owner deadline, preserving an already armed diagnostic deadline.
It stops before reporting and checks measured rest for at most 2 seconds.
Failure to verify rest cancels subsequent autonomous actuation.
The watchdog depends on the motor-owner task continuing to execute; it is not a
hardware watchdog. The Toggle/lift loops also depend on task scheduling.

There is no longer a route-level requirement to reach XY before lowering/opening.
The final claw call can run after a safely stopped incomplete lift movement.
This sequence does not establish scoring success or physical load retention.
No powered holding during navigation was introduced. No PID, geometry, motor
configuration, driver control, or P16-zeroing changes were made.