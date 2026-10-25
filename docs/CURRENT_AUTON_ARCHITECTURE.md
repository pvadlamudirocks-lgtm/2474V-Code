# Current-tree audit before implementation

Inspected the working files, not historical commits. The tree arrived dirty, including
navigation, mechanisms, main, the EZ archive, and untracked autonomous helpers.
No local changes were discarded. No AGENTS.md was found in the workspace.

- PROS / EZ-Template with local PID, wait-service, exit-observation and path tracing
  patches. Makefile excludes the legacy distance_correction.cpp entirely.
- FRONT is +Y at 0 degrees; +X at 90. Public conversion preserves XY and wraps
  headings, retaining ANGLE_NOT_SET for XY-only requests.
- Drive {-3,2}/{-1,4}; IMU 20; BACK horizontal tracker 5 and RIGHT vertical tracker
  7. Diameter, offsets, reversals, PID, slew and exit settings will be preserved.
- Three-argument go-to/reverse/turn calls use the trusted native blocking commands.
  Explicit-end-mode point calls instead use injected Pure Pursuit inside NavSequence.
  They are not interchangeable physical implementations.
- NavSequence has same-task mechanism service, per-motion deadlines, events,
  CHAIN handoffs, final acceptance/rest checks, cancellation and buffered tracing.
- Existing action helpers validate inputs but ordinary completion is explicitly
  unverified. Advanced paths/mechanism actions/correction gates default off.
- main currently selects RUN_CUSTOM_AUTON_TEST=true: start reset then toggle for
  395 ms. RUN_DEV_TEST=false. Competition entries left/right/safe/skills are inert.
- Driver: left Y/right X arcade; UP coast; L1/L2 full-voltage lift; R1 flex;
  R2 claw and flex stop; B normal toggle; X reverse toggle. A, Y, LEFT, RIGHT,
  DOWN are unused. Pose recording will use A capture, LEFT+RIGHT count reset,
  DOWN+Y explicit configured start initialization. No existing binding is stolen.
- Current toggle hardware config is P19 reversed and P6 not reversed, not P18/19
  from the request's background. Preserve current tree; physically resolve mismatch.
  Distance configuration must reserve both P18 and P19 as well as active ports.
- Flex P8/P10; lift P14/P15 and rotation P16; claw ADI A. Arm P17 exists but
  feedback port is unbound; flip ADI B disabled. Bottom limit is unbound.
- Lift target controller already has gated P/D hold with brake BRAKE, not brake
  HOLD. Blocking lift target wrapper explicitly zeros output on success. Full
  output commissioning bypasses target control. Limits/feedback are unqualified.
- No background mechanism task: opcontrol, blocking mechanism wrappers, or
  NavSequence own updates. Legacy blocking nav does not service them.
- Live Brain XYH display is gated by NAV_DIAGNOSTICS and screen ownership;
  controller bottom line shows lift at 100 ms. Selector owns disabled screen.
- Legacy distance module constructs conflicting placeholder sensors and uses
  guessed geometry/gates. It must remain excluded; new correction uses explicit
  unconfigured records and pure geometry rather than enabling that module.

Boomerang: forward wrapper calls pid_odom_boomerang_set with unchanged inch/degree
target and FRONT heading; reverse native pid_odom_set supports heading-bearing
targets. Local boomerang_task generates a lead point and switches to the real
target near the endpoint. motion_exit_trace wraps waits to defer PP settling until
the final waypoint. No clear static bug proves the reported short stop. PHYSICAL
DIAGNOSTIC REQUIRED; inspect requested/final targets and actual exit reasons.
