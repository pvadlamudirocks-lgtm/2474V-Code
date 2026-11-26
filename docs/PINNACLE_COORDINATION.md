# Pinnacle coordination helpers — inactive qualification stage

These helpers are NOT called by main.cpp. The current custom test and all route
selectors are unchanged. A successful build is not physical qualification.

## Reused infrastructure

- Normal lift set_target/update_axis/at_target: +/-2 P16 degrees continuously for
  the existing 200 ms settle interval. No preload directional-crossing shortcut.
- NavSequence wait service calls mechanisms_update on the same autonomous caller
  during native pid_wait. Native waits normally service at the existing 10 ms cadence.
- An optional NavEvent checks lift status after those updates and aborts the existing
  sequence on failure. No new task, motor owner, PID, follower or wait-hook replacement.
- Existing geometry_turn_deadline is enforced by the drivetrain motor-owner task at
  5000 ms. The explicitly supplied shorter drive deadline is cooperative, NOT an
  independent watchdog. Both deadlines include settling within native navigation.
- Existing drive_stop disables PID under its mutex before any post-motion lift wait.
- Existing NavSequence finish observes drive rest (<3 RPM for 100 ms, at most 2000 ms),
  then stops mechanisms. This extra bounded rest observation is not driving time.
- Existing normal lift STOP controller is used for intake and scoring. Original lift
  timeout, update-gap handling, P16 validity, home/limits and output faults remain intact.
- Existing pulse_toggle implements configurable duration via an overload; its no-arg
  form still delegates with 395 ms. Electrical polarity and toggle power are unchanged.

## Qualification blockers and scope

1. lift_calibration::hold_validated is still FALSE. Combined Boomerang/lift returns
   false before starting motion. Qualify active holding under the actual load before
   enabling this existing flag. Early completion otherwise uses the normal active
   sensor/PID Hold state while navigation continues, NOT just motor braking.
2. pinnacle_config::claw_toggle_overlap_verified is FALSE. Mechanical clearance and
   safe overlap must be established independently before enabling this new helper.
   A successful claw_close write is NOT proof that an object is secured.
3. Scoring requires a single-use PinnacleArrival produced by a successful combined
   segment plus explicit per-delivery descent-clearance and object-security evidence.
   These booleans represent external physical verification, not automatic sensing.
4. Existing calibration still says ALLIANCE_GOAL_1_DEG=32.1. It is NOT overwritten.
   The newly reported 401.7 clearance and 140.2 preload score are explicit arguments.
   Heights for later stacks, field poses, and measured timing margins remain unknown.
5. The lift timeout and update-gap abort are COOPERATIVE. If the autonomous task
   stalls or is terminated, these helpers cannot independently stop lift output.
   Existing disabled/driver lifecycle cleanup is retained. The 5000 ms watchdog
   independently stops the DRIVE, not the lift. No physical timing guarantee is made.
6. Calls are for one enabled autonomous task only, with no competing direct mechanism
   or navigation commands. Keep arrival evidence local to that invocation; never save
   it across mode transitions. Every bool must be checked; failures latch existing
   autonomous cancellation, but callers must still stop their sequence on false.
7. Stop API success is not physical rest, holding load capacity, or grip confirmation.
   No bottom-limit sensor, zero reset, actuator readback or dimensions were invented.

## Interfaces and illustrative calls (not selected or active)

Include pinnacle_auton.hpp and subsystems.hpp in a future qualified test. Symbols
like goal and measured are caller-supplied measurements, not provided route defaults.
No timeout should be copied from a guessed motor-running duration.

    PinnacleArrival arrival;
    if (!auton_boomerang_with_lift(goal.x, goal.y, goal.heading_deg, 70,
            401.7, measured.up_timeout_ms, measured.drive_timeout_ms, &arrival)) return;
    if (!auton_score_at_goal(arrival, 140.2, measured.down_timeout_ms,
            descent_clearance_verified, object_secured_verified)) return;

Drive timeout must be 1..5000 ms. Final arrival requires a normal native XY AND
angular exit, no interference/cancellation/deadline failure, measured rest, actual
XY error <=0.75 in AND actual heading error <=3 degrees. Small Exit alone is not success.
The final lift angle must still be in tolerance after rest and stop.

Standalone intake (from above or below, using the existing named calibration):

    if (!auton_lift_to_intake(measured.intake_timeout_ms)) return;

Intake while returning to a measured loader pose:

    const auto intake = lift_calibration::positions[
        static_cast<unsigned>(lift_calibration::Position::INTAKE)];
    if (!intake.configured) { auton_safe_abort("INTAKE_NOT_CALIBRATED"); return; }
    if (!auton_boomerang_with_lift(loader.x, loader.y, loader.heading_deg, speed,
            intake.degrees, measured.intake_timeout_ms, measured.drive_timeout_ms)) return;

Claw close and toggle, only after mechanical overlap qualification:

    if (!auton_claw_close_with_toggle()) return; // existing 395 ms default
    // Alternative: auton_claw_close_with_toggle(measured.toggle_pulse_ms)

Standalone normal nonblocking lift test, same task, drivetrain stopped:

    if (!lift_auton_stop()) { auton_safe_abort("LIFT_STOP_FAILED"); return; }
    if (!lift_auton_target_start(target_deg, measured.lift_timeout_ms)) {
        auton_safe_abort("LIFT_START_REJECTED"); return;
    }
    if (!auton_wait_for_lift(target_deg, measured.lift_timeout_ms)) return;

Do not insert a blocking delay between start and wait. Both use the original target
start/deadline; wait does not re-command the target or grant it a fresh timeout.
An already settled target does not wait for another settle interval. Success stops
lift output; it does not leave unqualified active holding enabled.

    auton_safe_abort("CALLER_SPECIFIC_STATIC_REASON"); // no claw release
    // Deliberately inactive interfaces: return false, cancel/stop, never dispatch.
    auton_acquire_from_loader(PinnacleLoaderPlan{});
    auton_run_pinnacle_route(PinnacleRoutePlan{});

The current function signatures are authoritative in include/pinnacle_auton.hpp.
The arrival type is noncopyable and invalidated on every scoring attempt. Scoring
rechecks measured rest/pose before descent and before release, requires downward or
already-in-tolerance target, normal settled lift success, final P16 tolerance, and
still-commanded-closed claw. Failure does not release the object.

## Timing and diagnostics

New summaries print after stopping outputs. Existing subsystem fault/command logs
are unchanged. Combined report includes command/start/native-completion/finish times,
requested lift target/deadline, initial/final P16, first observed settled time, final
pose, native XY/angular results, watchdog result and rest status. Command-to-settle
is not a measured motor-onset time. Settling is sampled at service cadence, not exact
physical arrival. Scoring and intake reuse existing lift completion diagnostics.
Pulse log reports measured software pulse duration and stop API result. Claw logs
report electrical command success, not cylinder motion or object capture.

## Future chaining

Existing NavProgress supplies pose, accumulated distance, elapsed time and native
waypoint index. NavEvent performs one-shot mechanism actions during native waits.
XY/Pure Pursuit CHAIN supports intermediate handoff; current pose/Boomerang overloads
reject CHAIN. These scoring helpers intentionally use SETTLE and independent actual
final-pose validation. No new continuous follower or altered path mathematics.

## Staged physical qualification (not performed)

1. With builders supporting/clearing the lift and its existing startup-home condition
   established, qualify stationary named intake from above and below. Record P16,
   start-to-settle time, stop behavior, timeout/fault outcomes and cancellation.
2. Confirm physical claw polarity and safe claw/toggle overlap independently. Only
   then qualify one stationary default pulse. Command success does not prove pickup.
3. Qualify normal nonblocking lift updates and loaded hold without driving. Do not
   assert hold_validated merely to bypass the gate. Measure travel/settle budgets.
4. After hold qualification, test one combined segment to a measured pose in clear
   space, initially at a separately approved test speed. Verify cancellation and
   both original lift and drivetrain deadlines. No automatic retry or route enabled.
5. Compare actual final P16, XY, heading and rest with reported completion. Lift-first
   and navigation-first cases must both pass without drift or missed servicing.
6. Qualify stationary descent to the measured preload score and physical release
   (existing stationary custom test stays intact), then one combined arrival/score.
7. Collect repeated goal/loader poses in one coordinate frame and per-stack heights,
   verify acquisition/clearance, then design and qualify the inactive full route.

## Software validation

Run node tests/pinnacle_auton_test.cjs. It compiles the actual new helper source and
actual configurable pulse against deterministic hardware/navigation doubles, with
qualification constants substituted only in the test header. No motors are accessed.
It covers early/late settling, original deadlines, cancellation-before-service,
sensor/native/rest/pose failures, intake both directions, toggle failures and cleanup,
single-use release evidence, blocked unqualified operations and inactive routes.
This does not simulate loaded motor dynamics, pneumatic timing or PROS scheduling.
Use the existing PROS environment for pros make. No upload is part of validation.
