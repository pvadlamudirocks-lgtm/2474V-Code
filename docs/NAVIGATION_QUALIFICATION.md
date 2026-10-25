# Navigation qualification

Current user evidence reports successful basic straight/turn/XY/reverse/return/
multi-point/square testing. This task performed no robot testing. Requalify after
hardware changes; leave all new results blank. Set measured acceptance thresholds
before qualification, not after looking at results.

For every row record commanded target/speed, start and end XYH, independent physical
endpoint, elapsed time, exit reason, battery/load, config version and repetition count.
Systematic error means consistent signed bias under controlled conditions; random
error means run-to-run spread without a stable sign. Both may coexist.

| Test | What it proves | Additional data | Systematic signature | Random signature |
|---|---|---|---|---|
| SENSOR_HEALTH | Valid IMU/trackers before dispatch | ports, raw values, calibration state | repeated wrong assignment | intermittent missing/error reads |
| MANUAL_ODOM | Frame, scale, tracker direction without powered motion | surveyed pushes/turns, raw centidegrees | wrong sign or distance-proportional bias | contact/slip-dependent variation |
| Short straight | initial slew and short travel | stopping distance | repeated overshoot | variable stop |
| Long straight | distance scaling and heading stability | cross-track versus travel | growing signed scale error | wandering spread |
| Reverse straight | BACK-first semantics | FRONT heading retained | fixed reverse bias | slip variation |
| 90 right | clockwise turn | raw IMU and final heading | consistent angular bias | inconsistent settle |
| 90 left | counterclockwise turn | same as right | direction asymmetry | variable friction |
| 180 turn | large-angle behavior | chosen turn direction | consistent undershoot | changing direction/endpoint |
| X-only go-to | +X convention | physical displacement | mirrored/swapped axis | varying approach |
| Y-only go-to | +Y convention | physical displacement | fixed forward bias | changing stop |
| Diagonal go-to | combined steering/travel | cross-track and endpoint | corner-dependent bias | varying curve |
| Reverse go-to | reverse point dispatch | target and FRONT heading | reverse geometry bias | spread under load |
| Return to origin | accumulated error | outbound and return physical endpoint | reproducible drift | dispersed return |
| Two point | independent motion transitions | each leg separately | second-leg inherited bias | variable transition |
| Square | frame and accumulating turns | all four corners | loop closure bias | unrepeatable closure |
| CHAIN_TWO_MOVE | native handoff without unwanted stop | buffered release errors and RPM | fixed transition miss | variable speed dips |
| Small chained heading | drive-turn-drive behavior | motion_chaining_test A/B | repeated turn shortcut | unstable turn/drive handoff |
| Pure Pursuit basic, if used | injected intermediate and final target behavior | pp index and final exit | stable final bias | inconsistent waypoint progress |
| GOTO_POSE diagnostic only | explains known short stop | requested/native/lead/final target and XY/angular exits | repeatable premature exit | inconsistent lead behavior |

Existing DevTests cover SENSOR_HEALTH, START_POSE, MANUAL_ODOM, DRIVE_FORWARD_12,
TURN_90, GOTO_Y_12, GOTO_X_12, REVERSE_12, RETURN_TO_ORIGIN, TWO_POINT, SQUARE,
GOTO_POSE, CHAIN_TWO_MOVE, PURE_PURSUIT_BASIC and MIRROR_MATH. Other distances and
turns must use explicitly chosen clear-space diagnostic commands; no new route
coordinates were invented. Advanced DevTests require ENABLE_ADVANCED_DEV_TESTS.

## Chaining audit

NavSequence remains the owner. A -> CHAIN, B -> CHAIN, C -> SETTLE is supported
through explicit end-mode calls. Session.sequence uses the existing SequenceStep
type, validates every step before dispatch and refuses all heading-bearing pose
steps. It is disabled by ENABLE_ADVANCED_PATHS until qualification. Timeout is
caller supplied. Direct NavSequence remains available for diagnostic turn chains.

CHAIN returns at native quick-chain release, with residual error and potentially
nonzero velocity. Native extension constants remain 3 inches drive / 3 degrees
turn (existing settings). These are not guaranteed endpoint errors. Log measured
release error. SETTLE uses native waits then existing acceptance <=1 inch and,
when requested, <=3 degrees, with 100 ms rest below 3 RPM and 2 s rest timeout.
These existing software thresholds were not changed or declared physically valid.

Encoder drives, XY point moves, reverse XY moves, turns and XY paths have explicit
end modes. Explicit point end-mode wrappers dispatch injected PP, unlike the trusted
ordinary point wrapper. Pose and final-heading path CHAIN are rejected. Small-turn
chaining is implemented but physically unqualified; use the existing drive-turn-
drive A/B diagnostic before considering it in routes.

Do not print, delay, allocate, run blocking mechanism calls or arm new events between
CHAIN handoffs. Mechanism output already latched can continue. Short same-task
NavEvents are the supported overlap mechanism; callbacks must not recurse into nav.
The action/Session sequence interfaces reject before/during hooks immediately after
a CHAIN and arrival hooks on a CHAIN. A sequence constructor and finish stop all
mechanisms, so start desired outputs only after construction. No chain tuning changed.

## Pure Pursuit and Boomerang audit

nav_waypoint is XY inches; a vector of points and explicit speed 1..127 supplies
native injected PP, in forward or reverse direction. Validation rejects empty paths,
nonfinite coordinates, invalid direction/speed; path-pose also checks final heading.
Lookahead is the existing global 7 inches in default_constants. One final pid_wait
uses the current wrapped final-waypoint wait fix; basic return is not verified
arrival. Session.sequence can use its existing XY injected paths; legacy follow_path
also remains available after explicit advanced opt-in. No path is forced on a route.
PURE_PURSUIT_BASIC retains {(0,12),(12,24)} at 40 from the existing diagnostic.
Qualify tracker scale/contact, turn behavior, lookahead, speed, deadline, final error
and interference under load before use. The native wait without Session/NavSequence
has no added route deadline; use guarded route interfaces for actual routes.

Forward pose wrapper uses pid_odom_boomerang_set, inches/degrees and unchanged FRONT
conversion. Local lead point is internal to boomerang_task; the wait wrapper avoids
intermediate PP settling. No source-proven cause of the major short-stop failure
was found. PHYSICAL DIAGNOSTIC REQUIRED. No PID/lead/offset edits; new competition
architecture has no pose-Boomerang dependency. Old optional pose APIs remain diagnostic.
