# 2474V autonomous layers

EZ-Template supplies the motion engine. `navigation` wraps that engine in the
canonical FRONT coordinates. `auton_actions` adds named actions and optional
sequence ownership. Competition route functions compose those actions; the Brain
selector selects route functions. Direct `nav_go_to`, `nav_go_to_pose`,
`nav_turn_to`, and `nav_drive_distance` remain available as escape hatches.

Current startup, driver code, active `SENSOR_HEALTH` test and selector are unchanged.
The selector still contains its inert entry; it does not dispatch the route stubs
in development mode. When measured competition routes are ready, register those
functions in `register_autons()` and deliberately restore selector dispatch in
`autonomous()`. Coordinate edits then require no selector changes.

## Writing routes

Competition entry points remain `auton_left`, `auton_right`, `auton_safe`, and
`skills` in `src/autons.cpp`. Put named measured points in a route-local namespace
or `include/field_points.hpp`. `RoutePoint` stores X, Y, speed and optional FRONT
heading. No heading (or the EZ sentinel) means XY only. No competition coordinates
have been supplied. `src/auton_example.cpp` is an inert, unregistered example with
unset points and its own false enable guard.

Use `drive_to_point`, `drive_to_point_reverse`, `drive_to_pose`,
`turn_to_heading`, and `approach_goal`. The last is only a point move: it does not
approach a wall or operate a mechanism. Valid basic moves call exactly the original
non-sequence navigation overload. Calls are for the enabled autonomous task only.
Do not nest them inside another NavSequence or invoke them from event callbacks.

`collect_object`, `score_object`, `score_high`, and `score_low` are explicitly
NOT CONFIGURED scaffolds. They issue no movement even if feature flags change.
The lift and pickup/scoring policy need physical commissioning first.

## Explicit advanced operations

Four flags in `include/auton_actions.hpp` default false:

- `ENABLE_AUTON_TELEMETRY`: optional action reports; also selectable per Context.
- `ENABLE_ADVANCED_PATHS`: Pure Pursuit wrapper and `run_sequence`.
- `ENABLE_MECHANISM_ACTIONS`: explicit mechanism callbacks and sequence hooks.
- `ENABLE_DISTANCE_CORRECTION`: explicit stationary correction callback.

`follow_path` calls the existing path/pose-path wrappers. `run_sequence` owns one
existing NavSequence and uses existing NavEndMode values; every step defaults to
SETTLE. It does not replace normal point movement. The existing explicit-end-mode
XY wrapper uses injected path infrastructure, so it is only used in this opt-in
sequence API. Sequence deadlines, exit checks, servicing and cleanup are unchanged.
Final steps must SETTLE. Pose CHAIN requests are rejected before any motion.

Before/during/arrival hooks are short calls to existing mechanism APIs. During
hooks reuse NavEvent and the existing wait service, with no background tasks.
No arrival action runs on a CHAIN release. Before/during hooks on the move following
a CHAIN release are rejected, matching existing event-arm restrictions. Hooks and
mechanism helpers return command acceptance, not physical completion. The route
must handle completion explicitly using the subsystem's existing update model.
NavSequence stops mechanisms on entry/finish; do not expect them to persist past it.

## Reports, failure and correction

Each action returns status plus elapsed milliseconds. Basic void navigation APIs
can only report RETURNED UNVERIFIED, not guaranteed arrival or a diagnosed timeout.
Sequence completion uses the existing sequence acceptance checks and fault reason.
Invalid/disabled requests return before dispatch. Check `can_continue()` at every
route boundary. Failed/cancelled dispatched actions use existing navigation stop
and mechanism-stop APIs; `cancel_route()` is also explicit. No automatic retries,
retreats, new accuracy thresholds, or replacement motion controller are installed.
PROS may terminate the autonomous task on disable before a return/report occurs.

Timing uses `pros::millis()` around the existing blocking movement. Optional
telemetry in `nav_diagnostics` prints route/step, target, start/final snapshots,
errors and duration, without extra rest waits. Chain snapshots are release samples,
not arrivals; all sequence reports are buffered until finish. Sequence total timing
includes hooks and existing cleanup; per-move timing excludes hooks. Printing can
add between-step latency when explicitly enabled. Existing lower-level logging is
not suppressed by this new flag.

`correct_pose_from_wall(callback)` is a disabled explicit hook; it never runs as
part of movement. After commissioning, a route can bind an existing `dist_reset_*`
function with its measured wall coordinate/offset. Distance correction remains
excluded in Makefile: its placeholder ports conflict with installed mechanisms.
Binding that implementation requires resolving hardware configuration and opting
its source into the build separately. This task changes none of its math/settings.

## Mathematical checks and development

X mirroring requires an explicit axis: x' = 2*axis_x - x; Y stays fixed. Because
heading uses (sin h, cos h), reflection maps h to -h modulo 360. Optional headings
and the EZ sentinel are preserved. RoutePoint mirroring rejects nonfinite inputs.
No route automatically mirrors. Static assertions check nonzero-axis reflection,
double reflection and cardinal headings during compilation. `DevTest::MIRROR_MATH`
adds no-motion runtime checks, including wrapped headings and invalid inputs;
it bypasses sensor reads and pose resets. It is not selected by default.

Existing DevTests and advanced locks remain unchanged. No physical motion or
mechanism behavior has been verified by these mathematical checks or the build.
