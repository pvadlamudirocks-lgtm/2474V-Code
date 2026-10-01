# Controlled Pure Pursuit diagnostic

The current diagnostic autonomous runs exactly one selected path. The latest
request explicitly authorizes this temporary harness. No controller tuning,
tracker configuration or coordinate conversion was changed. No robot run was
performed by the software build.

## Next physical test: Mode A only

Leave `PATH_DIAGNOSTIC_MODE` at **0** in `src/main.cpp`. Start the robot at public
`(0,0,0)` with the claw physically pointing +Y. Run autonomous once with enough
clearance for the supplied path `(0,10) -> (8,20) -> (20,20)`. The harness calls
`nav_follow_path(path, 127)` and resets odometry only before dispatch.

This is the highest-information first run: it removes Boomerang's moving target
from the experiment while retaining native XY PID, turn bias, angular steering,
lookahead and the existing exit/wait logic. We can establish whether translation
reversal already occurs in XY-only following before attributing it to Boomerang.

Send the entire terminal text from `CONTROLLED PATH TEST` through
`PURE PURSUIT BUFFERED TRACE END`, including CSV rows, event markers and summary.
Also say whether you physically observed the hesitation and approximately when.
Do not send only the final X/Y/H or only the summary. No tuning change is needed
before this run.

Mode **1** selects `nav_follow_path_pose(path, 90.0, 127)` instead. It is available
for a subsequent comparison; the harness never executes both modes in one run.
The public wrapper supports one speed only. No per-point-speed interface was
added, so all three requested points use 127 in both modes.

## What the actual code did before instrumentation

`src/navigation.cpp` has one shared `follow_path` implementation:

1. Reject empty paths, invalid speed/direction and nonfinite coordinates/headings.
2. Convert each public XY point to an `ez::odom` with the selected travel direction
   and the common speed. XY is unchanged; intermediate theta is `ANGLE_NOT_SET`.
3. Only for the pose variant, replace the **last** target's theta with
   `nav_game_to_internal_heading(final_public_heading)`.
4. Call `chassis.pid_odom_injected_pp_set(movements)` once, then `chassis.pid_wait()`
   once. The instrumentation adds only opt-in begin/wait-return observation here.

`nav_follow_path` passes `use_final_heading=false`; `nav_follow_path_pose` passes
`true`. Neither adds a separate movement, a stop between stages or a final turn.
These are thin native **injected** Pure Pursuit wrappers, not the smoothed
`pid_odom_set(vector)` overload. Linear point injection plus lookahead steering
gives continuous following; the wrapper does not generate a spline.

`nav_go_to` builds one XY `ez::odom`, uses EZ `rev`, calls the single-movement
`pid_odom_set`, and waits. That native XY overload selects injected PP. Existing
one-shot/debug output hooks in `nav_go_to` remain unchanged and are not invoked
by this test. `nav_go_to_pose` converts the pose, calls
`pid_odom_boomerang_set(movement)` with EZ `rev`, waits, and prints its existing
pose report. It is also not called by this harness.

Conversion remains `EZ H = wrap(public H + 180)`. XY axes remain fixed. Public
claw-first maps to EZ `rev`; public claw-reverse maps to EZ `fwd`. XY-only pose
conversion preserves `ANGLE_NOT_SET`. Both path directions apply to the whole
path; there is no added reversal waypoint.

The pre-task `autonomous()` was using a raw **smoothed** vector `pid_odom_set`
with an explicit slew argument of `true` and speeds 127/127/70, and then printed
"after" results **without calling `pid_wait()`**. Those readings were not final
arrival measurements. This new test uses the requested wrappers, uniform speed
127, and their existing global-slew selection. No slew setting was edited; this
is not a reproduction of that raw call's explicit per-motion slew override.
The old unrelated 12-inch trace arm/show calls were removed from this harness,
not deleted from the diagnostic library.

## Exact native execution paths

Version inspected: EZ-Template **3.2.2**, using installed headers and the versioned
source under [the v3.2.2 drive directory](https://github.com/EZ-Robotics/EZ-Template/tree/v3.2.2/src/EZ-Template/drive).

Both wrappers: `pid_odom_injected_pp_set(vector<odom>)` chooses global slew from
the first direction, then its boolean overload resets exit timers, calls
`set_odoms_direction` and `inject_points`, enables native turn bias, and invokes
`raw_pid_odom_pp_set`. That installs the vector and selects `PURE_PURSUIT`.
`ez_auto_task` updates odometry, then executes `pp_task` on its normal ~10 ms loop.

For Mode A, `pp_task -> ptp_task` throughout. For Mode B, intermediate points use
`ptp_task`; the final heading-bearing point uses `boomerang_task -> ptp_task` in
the same controller loop. No new drive command is dispatched by the wrapper at
that transition. The requested public 90 becomes EZ 270.

`inject_points` prepends the current pose and injects XY points at the existing
spacing (default 0.5 inches). It avoids injecting the segment leading into a
heading-bearing target. Its extra point after a heading-bearing intermediate
parent is not used for this wrapper, because only the last point has theta.

### Answers from the source

1. **Waypoint advance:** `pp_task` advances by one index per tick when Euclidean
   distance to the current waypoint is strictly less than lookahead. It calls
   `raw_pid_odom_ptp_set` for the new target. It does not stop or wait at each point.
   Trace indices include native injected points, not just the three user points.

2. **Last waypoint:** there is no further index advance. The same PTP/Boomerang
   calculations keep running while final exit conditions are evaluated. An XY
   waypoint supplies no explicit final heading constraint.

3. **Seven-inch lookahead:** it is an advance threshold, not a seven-inch stopping
   tolerance. It also determines the offset points used by `find_point_to_face`.
   Final XY settling continues after the last waypoint is selected. In Boomerang,
   lookahead/2 = **3.5 inches** is a separate threshold where the moving carrot
   becomes the final target.

4. **Heading/Boomerang switch:** a non-sentinel theta on the active waypoint
   selects `boomerang_task`. Carrot magnitude is
   `min(distance_to_target * dlead, max_boomerang_distance)` with direction applied.
   The carrot lies along the endpoint heading. When the carrot changes XY,
   `raw_pid_odom_ptp_set` updates the target, points-to-face, speed and crossing
   reference. It selects the Boomerang angular gains. Below 3.5 inches the carrot
   snaps to the endpoint. Theta shapes the approach; this is **not** an independent
   final-heading PID or an exact final-heading guarantee. Angular exits measure
   the point-to-face error, not simply requested endpoint heading minus actual H.

5. **Translation can reverse:** `is_past_target` returns a signed projected
   distance in the native target/point-to-face frame. PTP compares its sign with
   the sign recorded when that target was installed. The XY PID error is
   `abs(signed_distance) * direction_sign * flipped`, where `flipped` becomes -1
   when those signs differ. Crossing and recrossing can reverse that error.

6. **Other sources of reversed output:** the patched PID still computes
   `P*error + I*integral - D*derivative_on_measurement`. Its braking term can oppose
   travel even before the error crosses zero. PTP then clamps translation and
   multiplies it by `1 - (1 - cos(angular_error))/turn_bias`. With bias **0.9**, this
   scale is negative when `cos(error) < 0.1` (about 84.26 degrees in the principal
   range). Finally, angular mixing can reverse one wheel while average translation
   remains in the requested direction. The trace captures all these stages.

7. **Wait and repeated crossings:** the existing project wait fix first waits for
   the final injected waypoint with current protection, then resets settling
   timers while preserving current protection. It calls the unchanged native wait.
   Native wait independently latches XY and angular exit results and waits until
   both are non-RUNNING. Repeated crossings are possible during this period;
   entering an exit tolerance is not an immediate motor stop. Native `pid_wait`
   does not disable the controller or stop motors when it returns. The harness
   marks that return and explicitly stops afterward for a stationary dump.

8. **Final-point speed:** `raw_pid_odom_ptp_set` calls
   `pid_speed_max_set(imovement.max_xy_speed)`, which updates the slew maximums.
   PTP uses the actual `max_slew_out` to clamp XY and scale angular/XY mixing and
   both wheel commands. Thus a final-point cap is meaningful, subject to the
   current slew state/output. It starts when that target becomes active, not
   necessarily early enough to remove momentum. This trace records the actual
   slew limit and final commands, rather than assuming the requested cap applied.

9. **What a lower final speed would fix:** it can reduce approach energy, but it
   cannot remove signed-error reversal, PID braking, negative bias scale or the
   moving-carrot transition. Source alone cannot select the cause of this robot's
   symptom. No gains, lookahead, bias, dlead, Boomerang distance, slew or exit
   conditions were changed. The known measured offsets remain -2.290/+0.152.

10. **First test:** Mode A at 127, as described above. It determines whether the
    final reversal exists without Boomerang. Do not change multiple variables.

Additional source finding left untouched: upstream `raw_pid_odom_pp_set` reads
`pp_movements.end()->target` when computing the initial slew target. This is a
past-the-end access in that version's source. It is not corrected by this task
and is not evidence that it caused the reported endpoint symptom. The trace's
actual slew-limit column helps avoid assuming what the slew initialization did.

## Instrumentation and interpreting the log

The only new native controller patch is an optional callback after PTP's native
motor write. It passes already-computed locals, without rerunning PID, slew,
crossing calculations or motor mixing. The callback lives in the cold library
and is registered by the hot application, so it does not depend on interposing a
hot symbol on an already-bound cold call. `Drive` gains only a friend declaration;
its object layout is unchanged. Existing PID and wait/turn corrections are intact.

Capture uses a static 800-row ring: about 16 seconds at 20 ms, at least 8 seconds
if every nominal 10 ms tick emits an event. Oldest rows are overwritten if full;
the retained row window, overwritten count, skipped captures and maximum observed
tick gap are reported. Whole-run extrema/counters persist beyond ring wrap.
The callback never waits on a diagnostic lock; a contended capture is counted as
skipped. Motor RPM reads occur on stored rows. No allocation or terminal printing
is performed by the callback. Exit results are observed from existing calls,
never obtained by evaluating exit conditions a second time.

Routine rows capture public/EZ poses; active/final waypoint poses and indices;
distances; actual moving control target; signed target distance and crossing flip;
raw XY error/output/derivative; angular error/output; bias scale; actual slew
limit; scaled XY/angular output; final integer left/right motor commands; average
left/right actual RPM; direction/mode; output enable/interference; and observed
exit states. Waypoint advances, Boomerang activation/carrot lock, sign reversals,
negative bias and final-segment-plane crossings produce event rows and markers.

`L_cmd`/`R_cmd` are the exact integer arguments passed to native
`private_drive_set`, whose motor voltage request is `command * 12000/127` mV.
They are not velocity setpoints and not measured applied voltage. An output-enable
flag is included; EZ also suppresses motor writes in its first 1500 ms and skips
PTO motors. This calibrated test runs after startup and uses the normal drivetrain.

`travel_cmd = (L_cmd + R_cmd)/2 * direction_sign`; positive means the requested
travel direction, negative means commanded reversal. For claw-first, native EZ
translation is negative, so do not interpret EZ negative alone as reversal.
Sign markers ignore values within one command unit of zero and remember polarity
across zero; rows retain the actual small commands. Final-plane overshoot is the
projection past the final **user path segment**, while the final waypoint is active,
not a claim that the robot hit the exact endpoint. XY PID error flips can also
occur at waypoint/carrot changes, so they are labeled separately from plane crossings.

For Mode A, heading error uses the natural final-segment heading, 90 degrees for
this test, explicitly labeled **not enforced**. Mode B uses the requested heading.
The summary includes pose/error both at wait return and after harness stop/rest.

Interpretation:

| Evidence near the endpoint | Explanation to investigate |
| --- | --- |
| XY error flips with final-plane crossing, then raw XY and travel command reverse | Target crossing/correction |
| Raw XY opposes travel before plane crossing; large derivative | PID braking response |
| Raw XY retains its sign but bias becomes negative and travel command reverses | Angular-error turn bias |
| Travel command stays positive but one wheel reverses | Angular steering dominates one wheel |
| Abrupt changes coincide with Boomerang activation or carrot lock | Moving-target/controller transition |
| XY exits early but commands continue until angular exit | Independent exit latching and continued control |

After the wrapper returns, the **test harness only** stops the controller and
checks every drive motor for under 3 RPM for 300 ms (up to 3 seconds). If rest
cannot be verified, it shows an LCD message and retains the buffer without a
terminal dump. Otherwise CSV is emitted in ASCII at one row per 20 ms. The
navigation APIs themselves still neither stop residual PID nor print a trace.

## Files for this task

- `src/main.cpp`: authorized single-mode test; stale diagnostic calls removed.
- `src/navigation.cpp`: opt-in begin/wait-return capture calls only.
- `src/motion_exit_trace.cpp`: forwards existing exit results to the new observer.
- `src/path_motion_trace.cpp`, `include/path_motion_trace.hpp`: buffered capture,
  event markers, stop/rest gate and summary.
- `include/EZ-Template/path_trace_hook.hpp`: read-only native callback contract.
- `include/EZ-Template/drive/drive.hpp`: friend access, no new fields.
- `patches/EZ-Template/pid_tasks.cpp`: v3.2.2 source plus the observation hook.
- `Makefile`, `firmware/EZ-Template.a`: build/install the patched controller member
  alongside the existing unchanged PID patch.
- `NAVIGATION_PATHS.md`, this document: usage and source findings.

The build regenerates both cold and hot packages. Upload the rebuilt project
normally so the updated cold library is included; do not pair a new hot binary
with an old cold package. Nothing has been uploaded or committed by this task.

Validation completed: Mode A and Mode B both compiled and linked with the installed
PROS ARM toolchain. The final build is Mode A and generated `bin/hot.package.bin`
and the updated cold package. Format checks passed for the new trace; extra warning
checks reported only existing unused parameters in PROS LCD headers. Compile-time
sign-event checks cover initial negative EZ-reverse commands, deadband noise,
zero crossings and reversal in both directions. Symbol inspection confirmed the
cold hook setter and hot capture callback are linked, and the archive has one
`pid_tasks.cpp.o`. A source diff against v3.2.2 confirmed that the native patch
adds only the callback support and call, with no controller calculations removed
or changed. Baseline comparisons confirmed unchanged default constants, tracker
setup, driver control, existing PID source patch and existing wait implementation.
Physical motion and terminal transport remain unverified until the robot run.
