# Mechanism / lift commissioning handoff

The existing driver integration initializes mechanisms once and updates them once
per normal 10 ms driver loop. Temporary L1/L2 inputs call `test_lift_manual`:
L1 commands up, L2 commands down, and release commands zero voltage while feedback
is disabled. There is no background mechanism task or automatic startup homing.
No hardware behavior has been physically validated.

## Confirmed lift gearing (corrected 2026-10-03) and first test

Lift motors are on Smart Ports 14/15 and the Rotation Sensor is on 16. Both motors
still use `pros::v5::MotorGears::green`: nominal 200 RPM cartridges. The builders
confirmed external gearing is 1:1, so nominal lift output is also 200 RPM with no
extra motor-to-output scaling. This is a hardware specification, not a measured
speed. `LIFT_MOTOR_RPM` is 200, `LIFT_SPEED_UP_RATIO` is 1.0, and
`LIFT_OUTPUT_RPM` equals `LIFT_MOTOR_RPM`; the voltage controller needs no new RPM calculation.

Sensor mounting remains unconfirmed. `LIFT_SENSOR_DEG_PER_MECHANISM_DEG` stays 1.0,
which assumes direct coupling to the final lift/output shaft. Before feedback is
enabled, physically identify the sensor shaft and verify sensor degrees per output
degree. A sensor before the external gearing needs the corresponding measured
ratio; do not apply motor gearing to a sensor already on the final output shaft.

Only lift manual commissioning is enabled. Lift feedback, limits, scoring presets,
and `ENABLE_LIFT_HOME_SWITCH_COMMAND` remain false. The six-press logic is unchanged
and disabled. Both lift reversal flags remain false pending physical testing.
`LIFT_MANUAL_MV` remains 1500; this is a voltage-based direction test, not a speed test.

After uploading:

1. Physically support the lift, leave clearance from stops, keep hands clear of
   pinch points, and keep the drive joysticks centered. Verify both motors and the
   Rotation Sensor are connected to the ports above. No zeroing is required for
   this basic unreferenced manual direction check.
2. Enter driver control with L1/L2 released. Confirm no powered lift movement.
3. Give L1 a very short tap, then release. Verify BOTH linked motors cooperate and
   the lift moves upward. If they fight, bind, or move incorrectly, release and
   disable immediately; determine the needed reversal before trying again.
4. Only after that passes, give L2 a very short tap with downward clearance, then
   release. Verify both motors cooperate and the lift moves downward.
5. Confirm release stops the manual command. Keep supporting the lift: zero
   voltage is not active height hold, and calibrated travel limits are disabled.
   Do not increase voltage merely because 1500 mV does not move the supported load.
6. Release both buttons before leaving driver control. Next verify Rotation Sensor
   mounting, scale, and direction separately; only afterward consider output-speed
   tests. Leave all automatic/calibration flags false for these first taps.

This gearing update changes only header documentation/constants and this guide.
The following broader commissioning stages are for later, deliberate calibration.

## Files changed by the original subsystem implementation

- `include/subsystems.hpp`: centralized wiring, commissioning gates, calibration
  constants, modes, public functions, and diagnostic declarations.
- `src/subsystems.cpp`: replaces unused placeholder mechanisms with lift, arm,
  claw, flip, feedback control, guarded distance detection, and diagnostics.
- `Makefile`: removes only `subsystems.cpp` from source exclusions; distance
  correction stays excluded. Existing build flags are preserved.
- `MECHANISM_HANDOFF.md`: this guide.

The former intake/outtake/scoring motors and clamp/descore functions were unused
placeholder code. They were removed rather than left competing for ports. Legacy
`lift_up`, `lift_down`, and `lift_stop` names remain as wrappers. `lift_stop` now
means release-to-hold; `stop_all_mechanisms` is the explicit zero-voltage stop.

## Provisional wiring and conflicts

| Smart Port | Assignment |
| --- | --- |
| 14 | Left lift 11 W motor, green 200 RPM |
| 15 | Right lift 11 W motor, green 200 RPM |
| 16 | Lift Rotation Sensor; final-output coupling assumed, TODO verify mounting |
| 17 | Independent arm 11 W motor, green 200 RPM |
| 18 | Independent arm Rotation Sensor |
| 19 | Future claw/object Distance Sensor; not accessed with feature disabled |

| ADI port | Assignment | Initial commanded state when explicitly enabled |
| --- | --- | --- |
| A | Claw pneumatic | Closed (`claw_set(false)`) |
| B | Claw flip pneumatic | Normal (`claw_flip_set(false)`) |

Both ADI assignments and valve polarities are TODO CONFIRM. Pneumatics are disabled
by default. The software flip state starts false, but an unwritten software state
is not proof of a cylinder's physical position. Debug output includes `written`.
The default electrical normal flip output is also false.

The suggested 11/12/13 lift map would collide with `backRightDistance(11)`,
`leftDistance(12)`, and `rightDistance(13)` in `distance_correction.cpp`. Although
that file remains excluded from the build, this implementation avoids its ports
entirely. Its other placeholders stay at 8/9/10. Ports 11-13 therefore remain
reserved for those placeholders; 14-19 contain all six new Smart Port devices.
There are no spare ports within 11-19 with both provisional sets accounted for.
No distance-correction declaration or localization logic was changed.

Existing drivetrain 1/2/3/4, horizontal tracker 5, vertical tracker 7, and IMU 20
are unchanged. All new wiring choices are in `mechanism_config` in the header.

## Control behavior and enable gates

Both axes have `Disabled`, `Manual`, `Hold`, `MoveToTarget`, and `Fault` modes.
Initialization writes zero voltage and selects brake mode. It does not assume
startup is home, start hold, or move toward a mechanical stop.

The position calculation is Rotation Sensor accumulated centidegrees divided by
100 and by the sensor-to-mechanism ratio. The installed PROS API was inspected:
`get_position()` reads accumulated position; `reset_position()` zeros it.
`reset()` is not used as a substitute for position zero.

Manual up/down immediately replaces an automatic command and blocks new targets
while the manual state remains active. Keep refreshing manual input every loop.
On release, capture the current position once; subsequent updates hold that fixed
position. With reference/feedback gates not ready, release sends zero voltage
instead of enabling an unverified feedback loop. `lift_cancel_auto()` and
`arm_cancel_auto()` only cancel automatic movement; they do not interrupt manual.

The simple controller is:

```text
error_deg = target_deg - current_deg
voltage_mV = KP * error_deg - KD * measured_velocity_deg_per_second
```

Output is capped. No integral, motion profiling, motor-encoder position hold,
Kalman filter, or LQR is used. The rotation sensor is the primary feedback.
Starter gains are NOT TUNED. P-only control may settle below its target under
load; the starter voltage caps may not support the mechanism at all.

Enable stages separately in the header, rebuilding after each change:

1. `ENABLE_LIFT_MANUAL` / `ENABLE_ARM_MANUAL`: supervised commissioning jogs.
2. `LIFT_FEEDBACK_VERIFIED` / `ARM_FEEDBACK_VERIFIED`: only after motor direction,
   sensor sign, coupling, and reference are verified; allows release-to-hold.
3. `*_LIMITS_CALIBRATED`: only after replacing min/max placeholders with safe,
   measured travel limits. Automatic targets also require a valid home reference.
4. `LIFT_SCORING_POSITIONS_CALIBRATED` / `ARM_SCORING_POSITIONS_CALIBRATED`:
   unlock named scoring positions after each associated angle is measured.
5. `ENABLE_PNEUMATICS`: only after ADI wiring and output polarities are confirmed.
6. `POST_SCORE_RESET_VERIFIED`: only after confirming that claw-close/lift-lower
   commands may overlap safely and the arm will not collide along the return.
7. Both `ENABLE_CLAW_DISTANCE_AUTO_ARM` and `CLAW_DISTANCE_CALIBRATED`: only after
   separately validating detection and arm travel. Leave these false initially.

Raw `lift_set_target` / `arm_set_target` need reference, verified feedback, and
calibrated limits, but do not need the named-preset flag. This permits incremental
calibration within a measured safe range. Nonfinite or out-of-range targets are
rejected, not silently clamped. A false return leaves the previous command intact.
`lift_move_max` uses the calibrated maximum, not the scoring-preset flag.

A known home blocks downward voltage at the configured minimum. The maximum is
only enforced after limit calibration. Before homing, there are no trustworthy
absolute software limits; commissioning jogs require physical support and direct
supervision. Software limits cannot prevent momentum carrying a mechanism past
an endpoint: set conservative limits inside the physical stops.

Missing/invalid sensor readings and reported motor-write errors cause zero-voltage
requests and a latched Fault. Target travel times out after 5 seconds. A fault
invalidates home and needs explicit initialization followed by safe re-zeroing.
Zero output/brake mode does not guarantee a suspended load stays in place.

## Safe zeroing: lift and arm

Lift:

1. Physically support the lift. Stop motor commands; do not drive into the bottom.
2. Call `lift_initialize()` once, with the sensor connected. If Fault is shown,
   fix the connection/configuration before proceeding.
3. Physically place the lift at its known FULLY DOWN position, without forcing it.
4. Call `lift_zero_at_current_position()` once and check it returns true.
5. Confirm `lift_position_deg()` is approximately zero. It remains Disabled.
   Zeroing itself never starts hold.

Arm:

1. Support the arm and call `arm_initialize()` once.
2. Place it at the mechanically identified home corresponding to `ARM_HOME_DEG=0`.
   Do not assume the arbitrary startup position is that reference.
3. Call `arm_zero_at_current_position()` once; require true.
4. Check `arm_position_deg()` is approximately zero. It remains Disabled.

Re-zero after reboot, reinitialization, sensor replacement/reconnection, or any
fault that invalidates the reference. Do not call a zero function from an update
loop or use it to label an arbitrary scoring position as home.

## Exact first lift test: sensor, then low-voltage manual commissioning

Keep arm, pneumatics, distance-auto, lift feedback, and automatic-position gates
false. No test below runs by itself merely because the project was downloaded.
Use a dedicated supervised test loop following the integration contract below.

1. With motor power disabled and the mechanism supported, repeatedly call
   `test_lift_rotation_sensor()`. Move the axle gently through a small safe range
   by hand. Confirm a smooth reading and increasing degrees when raising the
   lift. Check a known output-angle change to verify the assumed 1:1 sensor coupling. Correct
   `LIFT_ROTATION_REVERSED` if necessary; rebuild before powered testing.
2. Verify both motor mounting directions before loading the linked axle. If
   unknown, mechanically decouple/unload the motors so an incorrect sign cannot
   make them fight through the linkage. Do not discover direction by applying
   full power to the assembled lift. Keep both motors electrically connected.
3. Set only `ENABLE_LIFT_MANUAL=true`; leave `LIFT_FEEDBACK_VERIFIED=false`.
   Call `lift_initialize()` once. Keep calling `mechanisms_update()` every 10 ms.
4. In that supervised loop call `test_lift_manual(true, false)` for at most a
   brief 100 ms initial jog, refreshing every 10 ms. Then call
   `test_lift_manual(false, false)` and continue updates. There should be zero
   output after release in this stage because feedback is deliberately disabled.
5. Verify BOTH left and right motor output directions correspond to lift-up when
   connected to the common axle. Change `LEFT_LIFT_REVERSED` and/or
   `RIGHT_LIFT_REVERSED` as needed. Neither false default is a physical claim.
   Rebuild, repeat unloaded, then reconnect the linkage only after directions agree.
6. With safe clearance, repeat briefly using `test_lift_manual(false, true)`.
   This must produce the opposite direction. End with
   `test_lift_manual(false, false)` and `stop_all_mechanisms()`.
7. Verify under supported linkage that positive motor command actually increases
   the lift sensor reading. Stop immediately if motors oppose, bind, buzz, or the
   reading moves the wrong way. The 1500 mV starter jog may be too weak to lift a
   load; lack of motion is not justification for a large voltage jump.

The diagnostic accepts held-state booleans, not fixed controller buttons. Both
true commands zero manual voltage and blocks automatic targets; both false
captures hold once on release when feedback is ready.

## Exact second lift test: release-to-hold

1. Complete the direction/sensor checks above. Enable `LIFT_FEEDBACK_VERIFIED`;
   retain `ENABLE_LIFT_MANUAL`. Keep named scoring and distance-auto disabled.
2. Initialize, physically place fully down, and zero using the procedure above.
3. Support the lift and jog to a small height with ample clearance by refreshing
   `test_lift_manual(true, false)` while updating every 10 ms.
4. Release with `test_lift_manual(false, false)`. Continue that false/false input
   and updates; the helper captures only the release edge, not a drifting target.
   Alternatively, call `test_lift_hold()` ONCE and require true.
5. Call `mechanisms_print_debug()` in the loop. Confirm mode Hold and a fixed
   target while current position is corrected toward it. Observe first for a few
   seconds with support available. Do not leave an untuned lift unattended.
6. Briefly command manual down/up and release again. The target must be replaced
   by the new release position. Finish supported with `stop_all_mechanisms()`.

Stable hold means disturbances decay, the target stays fixed, and the lift does
not repeatedly overshoot, chatter, buzz, drift continuously, or heat excessively.
A mode label or `lift_at_target()` alone is not physical evidence of stability.
`at_target` requires position inside its tolerance for 200 ms while updates run;
it is not a complete motion-settling or load-capacity test.

For later tuning, start unloaded/supported with correct feedback sign. Keep the
output cap conservative. Increase `LIFT_HOLD_KP` in small steps until sag/error is
acceptable; reduce it if oscillation begins. If needed, add small
`LIFT_HOLD_KD` steps to damp motion, watching for noisy/chattering output. Change
one value at a time and record target, measured position, load, and behavior.
Do not use gain changes to compensate for reversed feedback or binding mechanics.
The cap may limit performance; assess load and motor heating before changing it.
P-only hold has steady-state error under gravity; increasing KP indefinitely is
not a solution. Consider a separately measured gravity compensation only in a
future tuning change if necessary. Recheck at different heights and real loads.

After measuring safe min/max and setting `LIFT_LIMITS_CALIBRATED=true`, call
`test_lift_target(measured_safe_angle)` ONCE. Keep updating, verify settling, then
press manual input during travel: mode must become Manual immediately, and release
must hold the new position instead of resuming the old target. Test an out-of-range
target rejection before enabling named low/mid/high positions. Start inside the
measured range, not at the maximum or at a mechanical stop.

## Exact arm test sequence

1. Keep lift stopped/supported, arm motor power disabled, and distance-auto false.
   Repeatedly call `test_arm_rotation_sensor()` while gently moving the arm.
   Verify increasing angle toward score, home orientation, reversal, and coupling
   ratio. Change `ARM_ROTATION_REVERSED` / sensor ratio only from measurements.
2. Enable only `ENABLE_ARM_MANUAL`, initialize the arm, and use brief supported
   `test_arm_manual(true, false)` / `test_arm_manual(false, true)` jogs with a
   10 ms update loop. Release using false/false. Verify `ARM_MOTOR_REVERSED`.
3. After positive output increases sensed angle, enable `ARM_FEEDBACK_VERIFIED`.
   Initialize, physically establish home, zero, and check a true return.
4. Jog a small safe amount, release using false/false, and observe fixed-target
   hold. Tune `ARM_HOLD_KP`, `ARM_HOLD_KD`, and output cap conservatively.
5. Measure collision-free min/max travel with the lift in relevant positions,
   set those limits and `ARM_LIMITS_CALIBRATED`, then call
   `test_arm_target(measured_safe_angle)` once while continuing updates.
6. Test manual override during travel. Then measure the intended scoring and
   max-lift poses. Only after checking physical angles and collision clearance
   enable `ARM_SCORING_POSITIONS_CALIBRATED` and test `arm_move_score()` followed
   separately by `arm_move_max_lift_position()`. The intended 90/135 degrees are
   not currently calibrated promises.
7. End with support and `stop_all_mechanisms()`.

## Exact pneumatic test sequence

1. Stop/support lift and arm. Confirm ADI A is claw, B is flip, wiring, and valve
   polarity. Begin with air isolated while checking electrical mapping.
2. Enable `ENABLE_PNEUMATICS` only after those checks; keep space clear of cylinder
   travel. `mechanisms_initialize()` will explicitly command closed/normal if
   called with this gate true; it does not toggle either output.
3. Individually call `test_claw(false)`, require true, and verify closed. Then
   call `test_claw(true)` and verify open. Return with `test_claw(false)`.
4. Individually call `test_claw_flip(false)` and verify normal. Then call
   `test_claw_flip(true)` and verify the intended approximately 180-degree reverse
   orientation. Return with `test_claw_flip(false)`.
5. After explicit states are verified, try `claw_toggle()` once and then restore
   closed; try `claw_flip_toggle()` once and then restore normal. Observe each
   motion before issuing another command. No automatic timed cycling is provided.

A successful ADI write cannot prove pneumatic movement, pressure, or endpoint.
`stop_all_mechanisms()` does not vent air or change either pneumatic output.

## Distance and post-score behavior

Detection is off by default and does not even instantiate the Distance device.
When enabled and calibrated, each sample checks PROS errors/no-object values,
positive distance, calibrated range, and confidence where PROS supplies it
(above 200 mm). Three consistent samples spaced at least 20 ms are required;
invalid/out-of-range readings clear detection. Gaps over 100 ms reset the sequence.
Thresholds and sample count are not physically validated.

A new confirmed detection requests the arm scoring pose only from Hold mode.
It cannot replace Manual or another MoveToTarget. Rejected requests are discarded,
not queued to run on manual release. Distance auto-arm still requires home,
feedback verification, calibrated limits, and the named-arm-preset gate.

`mechanism_return_home()` is gated by `POST_SCORE_RESET_VERIFIED`. It checks lift
home-target eligibility, commands claw closed, then requests lift home. It leaves
the arm unchanged. This establishes command order only, not completion of claw
closure; verify safe overlap before enabling it. If physical closure must finish
before lowering, keep the gate false until a closure sensor or measured nonblocking
sequence is implemented. No unverified simultaneous arm/lift reset is scheduled.
Manual lift input overrides this home target just like any other target.

## Integration contract and future extensions

The current main.cpp already initializes mechanisms once and calls the lift manual
helper plus mechanisms_update in its normal driver loop, with L1/L2 as up/down.
The example below describes broader future integration, including the arm; it is
not a request to add a second initialization, update call, or mechanism task.

Include `subsystems.hpp` in a future mechanism test/driver entry point. Initialize
only the axis being tested (`lift_initialize` or `arm_initialize`), or initialize
both using `mechanisms_initialize`. Never initialize in every loop. Commands and
updates must have one owning task; this code has no synchronization for multiple
writers. Choose final controller mappings separately.

A future driver loop should perform these operations in order:

```cpp
// Once, outside the loop:
mechanisms_initialize(); // Does not zero; establish physical references separately.

// Each 10 ms iteration, alongside the existing driver logic:
test_lift_manual(lift_up_held, lift_down_held); // Your chosen input booleans.
test_arm_manual(arm_up_held, arm_down_held);
// Optional target requests belong on button/event EDGES, not every iteration.
// Process manual state first so held manual input rejects automatic requests.
mechanisms_update();
mechanisms_print_debug(); // Internally throttled to twice per second.
// The owning loop supplies its normal 10 ms delay.
```

The booleans above are integration placeholders, not new button mappings or a
standalone runnable entry point. Every diagnostic is individually callable.
Always call `stop_all_mechanisms()` before leaving a mechanism test or changing
competition modes. A future disabled callback should also cancel mechanism state
with that function. Re-establish a safe hold/reference deliberately on re-enable;
never resume an abandoned automatic target implicitly.

In autonomous, call target setters once and keep `mechanisms_update()` running
while the drivetrain moves. This implementation does NOT make blocking
`chassis.pid_wait()` or existing navigation calls service mechanisms. Use a future
cooperative loop that checks drivetrain completion and updates mechanisms, or
explicitly design a single-owner worker later. No drivetrain wait/loop was changed
in this task. Do not energize a mechanism then enter an existing blocking drive
routine without arranging continued updates.

The 100 ms stale-command/update checks are COOPERATIVE: they can stop only while
code is executing. They are not an independent watchdog and cannot remove motor
voltage during a blocked task. Do not stop servicing updates while a mechanism is
energized. Continue servicing a hold until intentionally stopping it with physical
support available. A complete hardware emergency-stop/watchdog is not claimed.

## Physical TODO checklist / constants still requiring calibration

All values are in `include/subsystems.hpp`:

- Confirm every Smart/ADI port, pneumatic polarity (`CLAW_OPEN_OUTPUT`,
  `FLIP_REVERSE_OUTPUT`), and normal flip orientation.
- Verify `LEFT_LIFT_REVERSED` AND `RIGHT_LIFT_REVERSED` against the common axle;
  verify `ARM_MOTOR_REVERSED` independently. No direction has been guessed as final.
- Verify both rotation reversal flags, coupling ratios, mounting security,
  monotonic position, and physical home references. Lift ratio 1:1 assumes final
  output-shaft coupling; actual mounting must be confirmed after the gearing update.
- Measure `LIFT_MIN_DEG`, `LIFT_MAX_DEG`, `ARM_MIN_DEG`, `ARM_MAX_DEG`, allowing
  stopping margin and checking clearance across lift/arm combinations.
- Measure `LIFT_LOW_DEG`, `LIFT_MID_DEG`, `LIFT_HIGH_DEG`; home is physically defined
  zero and max comes from the limit. Confirm `ARM_HOME_DEG`, `ARM_SCORE_DEG`, and
  `ARM_MAX_LIFT_DEG` against real geometry (90/135 are intended only).
- Tune `LIFT_HOLD_KP/KD`, `ARM_HOLD_KP/KD`, both `*_MANUAL_MV`, both
  `*_CLOSED_LOOP_MAX_MV`, both `*_TARGET_TOLERANCE_DEG`, and `TARGET_TIMEOUT_MS`.
  Verify `TARGET_SETTLE_MS` is appropriate. Check load, sag, heating, and repeatability.
- Verify the 10 ms loop schedule and 100 ms timeout margins during real usage.
  Do not increase timing limits to conceal blocking integration.
- Measure `CLAW_DETECT_MIN_MM/MAX_MM`; validate confidence threshold, consistent
  reading count, sample period, false-positive rejection, sensor placement, and
  intended object detection before enabling distance auto-arm.
- Verify post-score overlap and collisions before `POST_SCORE_RESET_VERIFIED`.
  Simultaneous arm return remains deliberately unimplemented until clearance is known.
- Set commissioning/calibration flags true only after their stated checks pass.

## Verification and scope

The installed ARM PROS toolchain compiled and linked the project successfully.
Compile success does not establish correct wiring, motion, safe physical travel,
feedback stability, pneumatic operation, or distance detection. No robot download
or automatic physical tests were performed.

`main.cpp`, `navigation.cpp`, `autons.cpp`, their navigation/auton headers, and
`distance_correction.cpp` were not edited by this mechanism task. Main/autonomous
files changed separately during the task and those changes were left intact.
Navigation and distance-correction file hashes still matched the initial audit.
No drivetrain/tracker/IMU configuration or drivetrain PID constants were edited
by this task. No hardware behavior is claimed as physically validated.

## 2026-10-01: claw, flex wheels, active toggle integration

This section supersedes historical claw-area wiring assumptions above. Current
configuration is in include/subsystems.hpp. No new ports are assigned. ADI C
remains the historical six-press input. Legacy flip/distance code is unchanged,
disabled, and is not part of this open-loop mechanism layer. Keep the legacy
ENABLE_PNEUMATICS flag false; the single claw uses CLAW_PNEUMATIC_CONFIGURED.

Set real ports and their configured flags only after wiring confirmation. Port 0
never constructs a device. Availability validates ranges, opposite flex reversals,
and known port conflicts; it does not prove a device is physically connected.
Smart ports reserved by existing devices/placeholders are rejected. If those old
placeholders are retired later, update the reservation check deliberately.
CLAW_OPEN_OUTPUT selects open polarity; closed is its complement. The first
piston construction uses the requested output, avoiding an intermediate closure.
No new pneumatic command is issued on disabled or safe reset.

Driver: R1 holds claw_grab (close + intake); R2 holds claw_score (open + outtake).
Release stops flex and retains piston state. Both held stops flex and retains the
piston state. B holds toggle_run; release calls toggle_stop. Existing L1/Down/L2
and Up behavior is retained. The lift-return close interlock remains in force;
its readiness guard now checks the real claw availability instead of the legacy
pneumatics flag. A rejected piston command stops flex and rejects the composite.
Composite commands report success only when both actions succeed; they are not
transactional (a successful piston write is not rolled back on a motor failure).

All semantic APIs are nonblocking and callable from driver or autonomous code.
Use pickup_prepare, route movement, pickup_capture, continued movement, then
pickup_finish. Prepare opens and intakes; capture closes and intakes; finish only
stops flex. No duration, navigation, or controller dependency is inside them.
claw_score is the single R2/scoring policy. toggle_run requests 200 motor RPM
with a green cartridge and configurable reversal; the route calls toggle_stop.
Each flex motor has its own reversal flag; flags must be opposite. Both receive
the same command, with one shared FLEX_INTAKE_SIGN. Outtake reverses that command.
Powers accept magnitudes 0..127; defaults are ZERO until commissioned, not tuned.
Stops use zero voltage, without adding HOLD mode. Initialization, disabled, and
entry to either competition mode stop the new motors via existing reset hooks.
Commands reject starting while disabled. Existing single-task ownership applies.

Tests in autons.cpp are separate, explicit, nonblocking steps, not timed routines:
- claw_flex_test(ClawFlexTestStep::Open/Close/Intake/Outtake/Stop, power)
- toggle_test(true) to run, toggle_test(false) to stop (default is stop)
- pickup_sequence_test(PickupTestStep::Prepare/Capture/Finish)
Call steps once at operator-selected events. Do not call diagnostic steps every
loop (they print on each invocation). No tests are selected automatically. Every
started test must end with Stop/Finish, or disabled; no guessed timeout is added.
Unavailable tests report configuration failure and return without fake hardware.
Autonomous callers should check bool results and stop/abort on rejected commands.

Remaining physical-test TODOs:
1. Assign actual left flex Smart Port.
2. Assign actual right flex Smart Port.
3. Verify each flex motor reversal and opposite shaft rotation.
4. Verify/flip FLEX_INTAKE_SIGN for inward transport.
5. Establish FLEX_INTAKE_POWER (currently zero).
6. Establish FLEX_OUTTAKE_POWER (currently zero).
7. Decide whether/how long flex continues after capture; route controls timing.
8. Assign claw pneumatic ADI port (never C or the lift bottom switch).
9. Verify true/false output that physically opens the claw.
10. Verify complementary output physically closes it.
11. Assign active-toggle Smart Port.
12. Verify/flip TOGGLE_REVERSED.
13. Confirm motor clockwise (record viewing side) gives final output counterclockwise.
14. Determine toggle stop coast/brake preference; currently zero-voltage stop.
15. Determine whether claw_score/R2 needs flex outtake or only piston open.
16. Measure actual autonomous pickup/scoring timing, including any piston settling.

First physical test after wiring is supplied: secure the robot with drivetrain
clear and claw area empty; leave flex defaults at zero and B released. Bind the
claw alone and issue Open, then Close as separate operator commands, checking
polarity before adding motors. Next bind flex, select a low positive test power
at the test session, issue Intake then Stop under operator control, verify opposite
shafts and inward transport, then Outtake/Stop. Test toggle separately, with an
operator-controlled B hold/release at its required 200 RPM, checking viewing side
and final output direction. Verify disabled stops both motor mechanisms and leaves
the piston state unchanged before trying combined pickup or any autonomous motion.
No physical direction, power, duration, or brake preference is claimed validated.
