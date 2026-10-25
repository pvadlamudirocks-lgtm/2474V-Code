# Development auton tests

Change only `ACTIVE_DEV_TEST` in `src/main.cpp` to select a test. The default is
`SENSOR_HEALTH`. The installed Brain selector remains available for future finalized
competition routes; development dispatch ignores its selection.

All coordinates are inches: +Y out of FRONT, +X RIGHT; heading 0 faces +Y,
heading 90 faces +X. No test automatically runs mechanisms.

| DevTest | Motion / expected request |
| --- | --- |
| SENSOR_HEALTH | No motor commands; P5 BACK, P7 RIGHT, P20 IMU status and readings |
| START_POSE | No motor commands; set (0,0,0), print pose |
| MANUAL_ODOM | No motor commands; set (0,0,0), manual-push telemetry for 30 seconds while enabled |
| DRIVE_FORWARD_12 | +12 inches FRONT-first, speed 40 |
| TURN_90 | FRONT heading 90, speed 30 |
| GOTO_Y_12 | (0,12), speed 40 |
| GOTO_X_12 | (12,0), speed 40 |
| REVERSE_12 | (0,-12), speed 40; BACK leads, FRONT approximately faces +Y |
| RETURN_TO_ORIGIN | (0,12), then (0,0), FRONT-first, separate settles |
| TWO_POINT | (0,12), then (12,12), separate settles |
| SQUARE | (0,12), (12,12), (12,0), (0,0), separate settles; later foundation test |
| GOTO_POSE | (0,12,0), speed 40, existing Boomerang wrapper |
| CHAIN_TWO_MOVE | Locked; existing (0,24), (24,24), speed 127 chain test |
| PURE_PURSUIT_BASIC | Locked; existing path API through (0,12), (12,24), speed 40 |

`ENABLE_ADVANCED_DEV_TESTS` in `include/dev_tests.hpp` defaults to false.
After physically verifying foundation tests, set it true, select the desired
advanced test, and rebuild. When false, either advanced selection prints a warning
and returns before sensor resets or motor commands.

Each run prints START/END markers to the PROS terminal. Sensor preflight rejects
missing/unreadable trackers and unavailable/calibrating IMU before pose resets or
movement. Manual telemetry includes raw centidegrees and sample deltas. It runs
in the autonomous task, so disable ends the sampling window (PROS may terminate
the task before the END marker can print).

`nav_print_snapshot` prints the current pose. `nav_print_test_result` reuses the
existing measured-rest capture and result formatter: start, target, final pose,
target-minus-final dX/dY, Euclidean position error, and optional shortest heading
error. No new accuracy PASS/FAIL thresholds exist. Rest timeout is reported as
unavailable, not successful arrival. Existing interference status remains visible.
Powered calls use `pros::millis()`; ordinary per-leg timing excludes report/rest
observation. Chain release/settle timings are buffered until sequence finish;
chain legs are not separate stops. Existing diagnostic output remains intact.

Future measured positions belong in `include/field_points.hpp`. All optionals
are unset and `FIELD_POINTS_MEASURED` is false. No competition route consumes them.
Future routes belong in `src/autons.cpp`: `auton_left`, `auton_right`, `auton_safe`,
and `skills` currently only print NOT CONFIGURED. Register finalized routes with
the existing selector and explicitly restore selector dispatch when ready.

Physical verification is still required: sensor identity/installation, raw signs,
manual +X/+Y/clockwise heading, start reset, actual 12-inch travel, turn direction,
coordinate and BACK-first movement, settling and measured error, multi-point and
pose behavior, then chaining and Pure Pursuit. Compilation is not physical testing.
