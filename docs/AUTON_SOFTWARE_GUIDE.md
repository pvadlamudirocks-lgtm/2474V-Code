# Autonomous software guide

Read CURRENT_AUTON_ARCHITECTURE.md for the pre-edit audit and AUTON_READINESS.md
for current gates. All coordinates are inches; FRONT heading is degrees clockwise
from +Y. Nothing here proves physical qualification.

## Selecting a mode

main.cpp keeps RUN_DEV_TEST, RUN_CUSTOM_AUTON_TEST and RUN_ROUTE_SEGMENT_TEST
mutually exclusive. The existing custom test remains selected: it resets start
then runs the toggle for the existing 395 ms. Set all three false to use the
Brain selector. A route-segment branch always returns without calling the selector.
NAV_DIAGNOSTICS disables the live Brain display. Pose recording has its own
disabled-by-default switch in pose_recorder.hpp. Controller lift telemetry remains
the existing driver feature. Brain live display adds lift and mode without taking
the disabled selector's screen. Detailed DevTest names print at test entry;
segment names are in the event log.

## Pose recording

Set pose_recorder::enabled=true. At the documented physical reference, initialize
the frame with the existing START_POSE DevTest or configure start_configured and
start in pose_recorder.hpp. The DOWN+Y rising chord initializes that configured
pose only with idle sticks, stopped motors and healthy navigation sensors. It
does not infer physical alignment. Verify the resulting pose before driving.
No start reset occurs merely by entering opcontrol.

Drive normally, position the robot, release motion controls, then press A once.
Terminal gives numbered X/Y/H and `{x, y, h}`. Controller rows 0/1 confirm capture;
row 2 stays available for lift. LEFT+RIGHT resets the counter only. Held inputs
do not recapture. Motor/drive mappings remain unchanged. Disable recorder for
competition. Existing UP selects coast (useful for placement); normal mode entry
restores brakes. Capture uses odometry, not independent surveyed ground truth.

## Route data and explicit callbacks

route_config.hpp is the central place for left/right/safe/skills starts, chosen
object count, prefix qualification flags and named Segment configurations. Each
stage has a configured flag and a function pointer; optional poses have no value,
and speed/timeout zero is invalid. No real stage currently has an execute callback.
SNAPSHOT is the only executable sample: it records pose and does no movement.

Add small named callback functions with signature
`auton::Result callback(auton::Session&, const auton::Segment&)`, then bind them to
their RouteSegmentTest entry in route_config.hpp. Callback code calls semantic
methods/APIs, checks each result and returns immediately on failure. For example,
an approach callback can validate `segment.point`, then return
`session.point("approach", point.x, point.y, segment.drive_speed,
segment.timeout_ms, segment.motion_class)`. Alignment uses session.turn separately.
Scoring uses session.mechanism with a named function calling commissioned lift,
claw, flex or toggle semantics. No controller reads or hardware ports in callbacks.

All labels retained by the logger must have static lifetime. Ordinary callback
code and NavEvents must not hide long delays, nest sessions or directly dispatch
raw chassis commands. Session rejects further calls after an abort, but cannot
stop arbitrary code that bypasses its API. Optional requirements should be checked
before movement, e.g. `preflight(true, true, false, true)` for navigation/lift/flex.
lift requirements mean calibrated target control; manual commissioning is separate.

run_route_prefix validates all stages in the selected prefix before resetting
the configured start, verifies readback against a caller-supplied tolerance, and
runs stages in order. Objects=1 runs B1 only. Objects=2 includes the same B1 then
B2, and requires qualified_prefix[0]. Objects=3/4 similarly require every earlier
prefix. The currently tested newest block need not already be marked qualified.
Skills uses the same route type/entry and can reuse callbacks; extend explicit
skills route code later if it needs more than the current four-object prefix.

## Independent segment tests and checkpoints

Set RUN_ROUTE_SEGMENT_TEST=true, other development flags false, select the route
and ACTIVE_ROUTE_SEGMENT. This executes only that callback. It does not require
the whole route enabled or all previous blocks qualified; that is how individual
stages are commissioned. It never resets odometry to the expected start: physically
place the robot, initialize the common frame, and compare actual versus expected.
SEGMENT_START records expected start in target columns and actual in start/end.
SEGMENT_END records final pose/target/errors. No expected pose means NaN columns.

Checkpoint metadata optionally validates Euclidean XY and wrapped heading against
explicit tolerances. A checkpoint does not automatically stop. settle_required
adds the existing-style rest observation (100 ms below 3 RPM, 2 s timeout); this
is software policy inherited from diagnostics, not a physically validated arrival
threshold. final_tolerance provides explicit arrival checks. A mechanism expectation
string is descriptive; verify_mechanisms is required for an actual feedback check.
PASS means configured software checks passed, not independently measured success.
ABORT_ROUTE and STOP_PREFIX both terminate safely; neither skips a failed stage.

Correction policies NONE/ALLOWED/REQUIRED are explicit. Required means an accepted
correction must be recorded before the segment ends. Call session.stop_motion()
before the stationary wall helper, then provide its measured stationary gates.
Optional rejection leaves odometry unchanged. A required rejection stops the route
when checked/end_segment executes. Do not ignore failure of a required correction.

TRANSIT, APPROACH and PRECISION express intent only; every motion requires a
chosen speed and deadline. There are no class speed defaults. Trusted session
point/turn calls preserve the three-argument wrappers, add a wait-service deadline
and sensor/mode checks, and log return as RETURNED_UNVERIFIED. Set checkpoint
targets/tolerances to validate final arrival. Do not assume return means success.
No pose-Boomerang method is offered by Session.

## Logging and PC analysis

Session buffers up to 256 major events with no heap allocation or serial output
at handoffs. At cleanup, CSV lines have AUTONCSV prefix, quoted labels, run number,
timestamp, event, route/stage/motion, result, start/target/end XYH, speed, errors and
mechanism position. Overflow prints AUTON_LOG_DROPPED; such logs are incomplete.
Timestamp is Brain milliseconds; run IDs restart after a reboot. Lifecycle aborts
flush after stopping. Board power loss cannot guarantee delivery; no Brain file is
written. Existing custom tests/old helper calls are not automatically Session routes.
To obtain these CSV events, use Session/segment harness or explicit log_event calls.

From project root:

```text
python tools/analyze_auton_runs.py run1.txt run2.txt --group segment
python tools/analyze_auton_runs.py run1.txt --group motion
python tools/analyze_auton_runs.py run1.txt --group route
python tools/analyze_auton_runs.py - --group segment
python -m unittest discover -s tools -p "test_*.py" -v
g++ -std=c++17 -Iinclude tests/auton_math_test.cpp -o auton_math_test
./auton_math_test
```

The analyzer ignores unrelated serial lines, warns on malformed/missing headers,
and outputs JSON with counts, means, ranges, sample deviations, circular heading
mean/deviation, smallest covering heading arc, Euclidean error and signed XY bias.
No external Python packages. `--event` selects AUTON_END/SEGMENT_END/MOTION_END.
Default chooses the matching endpoint for the grouping, avoiding duplicate events
per movement. Quoted route names are supported. Repeated log headers distinguish
captures, so do not duplicate the same capture in input. Grouping motion includes
route and segment to avoid mixing identical motion names from different stages.

Compare mean error vector to spread to identify bias versus variability. Preserve
conditions (battery, load, config version, start placement) externally. Never pool
different geometry under the same name. Odometry reports cannot reveal odometry
scale bias on their own: survey physical endpoints too.

## Safe abort and ownership

Session abort latches a result, calls the existing nav_cancel_sequence cleanup,
zeros motor mechanisms and preserves pneumatic states. Destruction uses that
same cleanup; main's original RAII remains. disabled/opcontrol use lifecycle_stop
because PROS can terminate autonomous without C++ stack unwinding. A stopped or
failed Session cannot dispatch another command. Deadline, invalid sensor/config,
interference, rejected mechanism and mode changes all lead to stop, never retry.
Motor write success is not physical proof; unplug/failure-injection tests remain.
No hidden background task or independent hardware watchdog was added.
