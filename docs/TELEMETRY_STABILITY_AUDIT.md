# 2474V telemetry and stability foundation — 2026-10-15

The current working tree was audited before editing. Existing uncommitted work
was preserved. This change does not claim that the robot is anti-tip protected.

## Drivetrain ownership audit

| Mode/path | Current output owner |
|---|---|
| Driver | `opcontrol()` -> unchanged `driver_arcade_control()` -> EZ `opcontrol_joystick_threshold_iterate()`, including existing active braking |
| EZ autonomous | `nav_go_to`, `nav_turn_to`, and related wrappers dispatch targets; EZ `ez_auto_task()` runs the selected PID and `private_drive_set()` |
| NavSequence/Session | Supervises and waits on EZ; the existing `navigation_lock` serializes cancellation/target transitions against the patched EZ PID loop |
| Custom autonomous | Current selected custom function runs the existing toggle test; any uncommented navigation calls would delegate to EZ. Direct `drive_set()` calls in legacy tests are manual output/stop paths |
| Disabled/mode transition | PROS terminates the prior competition task; `auton::lifecycle_stop()` calls `nav_cancel_sequence()`, which clears hooks and calls `drive_set(0,0)` under `navigation_lock`, stopping/disabling EZ motion |

The patched EZ task locks around the autonomous control switch, but arbitrary
legacy/custom motion calls are not all governed by a universal takeover protocol.
Basic wrappers can block in `pid_wait`; sequence and Session paths have their own
lifetimes. An asynchronous takeover must also prevent those callers from dispatching
their next motion. A mutex alone is not enough. No existing route/ownership code
was changed to force a recovery path into this task.

Anti-tip is therefore a **read-only monitor** serviced by the existing Brain
display task every approximately 50 ms, independently of the selected Brain page.
It creates no task, writes no motor, and does not cancel routes. It cannot compete
with the driver or EZ for motor output. No new recovery motor owner exists.

Future active driver integration should apply `stability::propose_driver_command`
once to the deadzoned forward/turn request, before the existing mixing/output path.
It adds the bounded signed pitch correction to forward and preserves turn except
for the proposed severe-roll stop. Its output is only data today; nothing applies it.
No steering correction for lateral roll is invented.

Future autonomous takeover must be implemented in the navigation supervisor:
latch route abort, prevent subsequent dispatch, cancel under the navigation mutex,
confirm EZ mode DISABLE and zero output, then transfer ownership to a recovery
controller with a bounded timeout and explicit stop. Recovery must not auto-resume
an interrupted route. This integration and physical validation remain future work.

## Dashboard and diagnostics

| Physical row / API index | Example | Meaning |
|---|---|---|
| First / 0 | `H301 X-16.6` | Canonical heading degrees, X inches |
| Second / 1 | `Y14.3 L842` | Canonical Y inches, lift Rotation Sensor degrees |
| Third / 2 | `D18.42in` | Configured sensor-face vertical clearance |

All five values fit. Every row is exactly 15 padded characters plus a terminator;
printing starts at column 0. No full-screen clears are used. Number overflow uses
`---`, never a truncated numeric value. Normal controller display rounds H/L to
whole degrees, X/Y to tenths, and D to hundredths. Full measurements are retained.
Before vertical/fixed-angle geometry is configured, row 2 adds `b`, for example
`D18.42in b`, meaning beam distance only. Invalid distance is `D---`.

Heading/X/Y come directly from `nav_pose_get()`, the same FRONT-based public frame
as route coordinates (+Y FRONT, +X RIGHT, clockwise heading). There is no new
coordinate transform. `nav_sensors_ready()` prevents cached odometry from being
displayed as valid after a sensor failure. Lift uses existing `lift_position_deg()`.
D uses the filtered height snapshot, divided by exactly 25.4 for beam inches, or
its explicitly configured vertical projection. No claw offset is applied to D.

Capture interval is 100 ms. Actual writes are one row per 50 ms (150 ms per row),
because installed PROS `misc.h` documents the wireless limit. Failed writes retry
on the next slot; disconnected controllers may refresh more slowly. There are no
new driver-loop delays. The pose recorder retains all button actions and terminal
feedback; it no longer prints controller text. EZ selector/Brain behavior is unchanged.

`print_robot_telemetry()` is an on-demand terminal snapshot: validated pose,
pitch/roll, lift degrees, raw/filtered distance, beam/vertical/reference inches,
confidence, validity, rates, state, proposed correction and configuration/fault reason.
Call it from the same task that owns lift-height sampling; it is not a background
logger. `anti_tip::snapshot()` is mutex-protected and returns invalid after 200 ms.

## IMU frame and signal processing

P20 remains the sole IMU through existing `chassis.imu`. New `robot_heading_deg()`,
`robot_pitch_deg()`, `robot_roll_deg()` and `robot_orientation()` centralize reads.
Heading remains canonical navigation heading, with no added 180-degree adjustment.
Moving the IMU toward the right side requires no XY translation offset for this
heading-based odometry. It can affect dynamic accelerations; validate readings
under actual motion after relocation.

There is no established pitch/roll mounting calibration in this tree. Default
`IMU_PITCH_SIGN=+1`, `IMU_ROLL_SIGN=+1` preserve sensor-local signs as provisional
values; `AXES_VERIFIED=false`. The intended calibrated robot convention is positive
pitch = front up, positive roll = right side up. A planned horizontal 180-degree
rotation has NOT been assumed or applied. Measure signs after the actual mount is
installed. A different mounting involving axis swaps needs an expanded transform.

Angular rates are derivatives of angle-filtered samples followed by a rate filter,
using actual elapsed time and wrap-safe angle differences. Both filters have an
80 ms time constant (signal smoothing, not tuned physical tipping thresholds).
First sample, invalid data, and gaps over 200 ms invalidate the rate and rewarm.
IMU missing/calibrating/error, PROS_ERR_F, nonfinite readings or invalid signs produce
disabled state and zero proposed correction, with a diagnostic reason.

## States and calibration gates

`anti_tip::config::mode` defaults to MONITOR_ONLY. Possible states are DISABLED,
STABLE, WARNING, RECOVERY, SEVERE. With current configuration the state is DISABLED
because axes/thresholds are unverified, while readings and rates remain observable.
It does not invent a STABLE classification without calibrated thresholds.

`config::settings` uses `stability::Settings`: `warning_pitch`, `recovery_pitch`,
`severe_pitch`, `exit_pitch`, `warning_roll`, `severe_roll`, `exit_roll`,
`rate_horizon_s`, signed `k_angle`, signed `k_rate`, and `max_correction`.
All default to NAN; `calibrated=false`. Enter/exit ordering is validated. Recovery
holds until the lower exit threshold; warning/severe transitions also have lower
release thresholds. Actual and rate-projected angles influence entry, so an angle
moving toward upright differs from one moving away. No final values are supplied.

Proposed pitch correction is `k_angle*pitch + k_rate*pitch_rate`, limited by the
configured maximum (must be positive and <=127). Invalid input/configuration yields
zero proposal. Roll contributes warnings/severe state and a future stop proposal;
no lateral correction or random steering is generated. Proposed output is never applied.

Future height-dependent sensitivity can use the combined snapshot's Rotation Sensor
degrees and optional Distance Sensor height. No height thresholds, acceleration
limits, fitted mapping, or sensitivity policy is enabled. Rotation remains primary;
monitor validity never depends on the Distance Sensor or a clear floor beam.

`ENABLE_LOW_POWER_TEST=false`, `LOW_POWER_OUTPUT_LIMIT=NAN`, and
`DRIVER_RECOVERY_VALIDATED=false` document future stage gates. They are deliberately
not wired to a motor writer. Changing a flag alone cannot enable recovery.
`AUTON_TAKEOVER_IMPLEMENTED=false` explicitly records the missing active integration.

## Physical commissioning sequence

1. Complete the distance checklist in [LIFT_HEIGHT_CALIBRATION.md](LIFT_HEIGHT_CALIBRATION.md):
   assign the real unused port, confirm detection, verify downward geometry through
   travel, clear the beam, raise/lower, test repeatability and invalid/no-target
   behavior, set an optional known signed reference offset, collect real pairs.
2. Select ONLY `RUN_DEV_TEST=true`, `ACTIVE_DEV_TEST=DevTest::STABILITY_MONITOR`,
   `RUN_CUSTOM_AUTON_TEST=false`, `RUN_ROUTE_SEGMENT_TEST=false` in main.cpp for
   Stage 1. Defaults were not changed. Rebuild before running that selected test.
   It cancels prior navigation, stops mechanisms, and reports every 500 ms for
   30 seconds without commanding movement. It also reports an unavailable IMU.
3. Support the robot securely. Record pitch/roll and rates flat, front up, front
   down, right side up, left side up. Return upright between tests. Rotate clockwise
   on a flat surface and confirm increasing canonical heading (wrap at 360).
   Fill the table below; set signs only from these observations, then recheck and
   set AXES_VERIFIED. Repeat after any mounting rotation/relocation.
4. Still MONITOR_ONLY, determine physical warning/recovery/severe/exit angles,
   rate contribution and signed gains under controlled testing. Populate settings,
   validate hysteresis and proposed correction direction. Test sensor loss and
   startup: proposal must remain zero on invalid data and warmup.
5. Stage 2 is gated: implement/review a sole-owner low-power test application first.
   Use an explicit enable, a measured conservative output limit, supported robot,
   an operator stop, and a bounded timeout. Validate actual correction direction.
   Do not enable driver/autonomous intervention merely because monitor math passes.
6. Stage 3: only after Stage 2 passes, integrate once into the driver command path
   and verify turning, startup, disable and sensor-fault behavior.
7. Stage 4: implement and test autonomous abort/takeover ownership separately.
   Verify no later route command resumes after abort; no automatic resume.

| Physical direction | Observed pitch sign | Observed roll sign | Expected calibrated result |
|---|---|---|---|
| Front upward | | | Positive pitch |
| Front downward | | | Negative pitch |
| Right upward | | | Positive roll |
| Left upward | | | Negative roll |
| Clockwise flat rotation | | | Increasing canonical heading |

## Software validation

`tests/telemetry_stability_test.cpp` contains compile-time assertions using synthetic
fixtures, not robot tuning values. It is host-testable with any C++17 compiler
(`c++ -std=c++17 -Iinclude tests/telemetry_stability_test.cpp`) and can also be
validated without target hardware using the installed ARM compiler's `-fsyntax-only`.
Tests exercise signs, invalid values, state transitions/hysteresis, angular-rate
direction, correction saturation/turn preservation, rate warmup/gaps/wrap, median
spike rejection/reset, sensor range/sentinels, and exact bounded dashboard strings.
No test claims to validate physical tipping or distance accuracy.

Final validation: `make quick -j4` compiled and linked successfully with the
installed PROS toolchain. The C++17 calculation tests passed compile-time evaluation
using `arm-none-eabi-g++ -Wall -Wextra -Werror -Iinclude -fsyntax-only`.
This is compile-time execution of the assertions, not a hardware test or a claim
that a native host executable was run. `git diff --check` passed; existing Git
LF/CRLF conversion notices are not whitespace errors. Diff and status were reviewed.

## Files changed by this task

- Added `include/anti_tip.hpp`, `src/anti_tip.cpp`: read-only stability monitoring.
- Added `include/robot_imu.hpp`, `src/robot_imu.cpp`: centralized IMU frame/validity.
- Added `include/stability_math.hpp`: pure state, rate and command-proposal math.
- Added `include/telemetry.hpp`, `src/telemetry.cpp`, `include/telemetry_format.hpp`:
  dashboard, diagnostics and no-motion monitor test.
- Added `include/lift_height_math.hpp`: testable distance gate and fixed median.
- Added `tests/telemetry_stability_test.cpp`: compile-time calculation tests.
- Updated `include/lift_height.hpp`, `src/lift_height.cpp`: confidence diagnostic
  and use of the tested median/range helpers.
- Updated `src/main.cpp`: dashboard hookup, read-only monitor servicing and
  allowance for the diagnostic test to report IMU calibration failure.
- Updated `src/pose_recorder.cpp`: terminal feedback instead of controller writes.
- Updated `include/dev_tests.hpp`, `src/dev_tests.cpp`: STABILITY_MONITOR selection.
- Updated `src/wall_correction.cpp`: reject collision with lift-height sensor port.
- Updated `docs/LIFT_HEIGHT_CALIBRATION.md`; added this audit report.

No edits to subsystem motor/control implementation, mechanism configuration,
navigation implementation, routes, tuning, or existing driver mappings were made
by this task. Earlier working-tree edits remain intact. No commit, push or upload.

Distance status: **SOFTWARE READY FOR HARDWARE TESTING**. Active anti-tip status:
**MONITOR ONLY; takeover integration and physical validation still required**.
