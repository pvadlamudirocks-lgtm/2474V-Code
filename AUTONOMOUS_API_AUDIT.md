# Autonomous action interface audit — 2474V

Audited against the current local working tree, 2026-10-09. No robot upload or physical testing was performed. The PROS `make quick -j4` build succeeded. Existing uncommitted work was preserved.

## Execution and remaining commissioning

`src/main.cpp` has three mutually exclusive paths:

| Mode | Configuration | Dispatch |
| --- | --- | --- |
| Development (current default) | `RUN_DEV_TEST=true`, `RUN_CUSTOM_AUTON_TEST=false` | `run_dev_test(DevTest::GOTO_POSE)`; then return |
| Custom development | `RUN_DEV_TEST=false`, `RUN_CUSTOM_AUTON_TEST=true` | `custom_auton_test()`; then return |
| Competition | Both flags false | Existing Brain selector's `selected_auton_call()` |

A compile-time check rejects both development flags being true. The custom routine retains the existing distance-12/speed-127, delay-1500, claw-close test; it is NOT currently selected. Its existing numbers were not invented by this audit. `CUSTOM_AUTON_USES_NAVIGATION=true` retains IMU/tracker preflight. Set it false only after replacing the custom routine with mechanism-only code. Competition always requires successful startup IMU calibration and current sensor health. DevTests retain their own preflight and geometry. Mechanism APIs themselves do not depend on the IMU.

The selector now registers `auton_left`, `auton_right`, `auton_safe`, and `skills`. Their bodies deliberately remain unconfigured and print that fact. No match coordinates were supplied or inserted. Normal autonomous return cancels navigation and stops mechanism motors. `disabled()` and `opcontrol()` retain lifecycle mechanism cleanup; this also handles competition task termination, where a C++ destructor alone cannot be relied on.

No changes were made to navigation/odometry implementation, NavSequence, DevTest geometry, drivetrain configuration, tracker registration/offsets, IMU calibration, navigation constants, exits, chain behavior, Pure Pursuit, or EZ patch sources. Before/after SHA-256 comparison confirmed those source/header files unchanged. This task changed only `src/main.cpp`, `src/autons.cpp`, `src/auton_actions.cpp`, `src/subsystems.cpp`, and their relevant `autons`, `auton_actions`, and `subsystems` headers, plus this report.

Status categories used below:

- **A:** software complete and physically validated for the stated behavior.
- **B:** software complete; physical validation required.
- **C:** software complete; physical calibration values required.
- **D:** blocked by unknown hardware assignment.
- **E:** software incomplete.

| Mechanism / feature | Status | Specific remaining requirement |
| --- | --- | --- |
| Claw ADI A | B | Observe OPEN/CLOSE polarity and mechanism/plumbing behavior |
| Flex P8/P9 explicit-power commands | B | Confirm semantic inward/outward direction and loaded operation; existing reversals preserved |
| Pickup and scoring combination helpers | C | Choose real intake/outtake powers; also validate claw/flex behavior |
| Lift full-voltage autonomous API | B | Driver electrical UP signs were previously confirmed; new autonomous integration has not been physically tested |
| Lift position control | C | Reference, feedback direction, safe limits, target angles, controller tuning/voltage capability, and measured timeout |
| Lift bottom-switch return | D | Actual switch port/polarity; then measured return timeout; currently unavailable |
| Toggle | D | Real Smart Port; then direction/gear verification |
| Claw flip ADI B | B, inactive/unconfirmed | Confirm this mechanism is still installed, wiring and polarity before enabling |
| Arm P17/P18 | C, inactive/unconfirmed | Confirm presence/coupling, reference, safe limits, targets and tuning before enabling |
| Claw distance P19 auto-arm feature | C, inactive/unconfirmed | Confirm presence, detection range/confidence and arm commissioning |
| Existing navigation | B for this audit | Retained software; no new physical navigation validation claimed |
| Excluded wall-distance module | D | Placeholder ports conflict with active hardware; see historical-module notes below |

There is no newly claimed category A mechanism. The existing lift-UP sign confirmation is historical evidence, not a claim that this build was tested. Competition route bodies still need the team's actual route design; they are not completed match routines.

## Claw: explicit answers

1. `claw_open()` reaches the real ADI A `DigitalOut` through `claw_set(true)`.
2. `claw_close()` reaches that same object through `claw_set(false)`.
3. Both accept enabled autonomous calls, without controller inputs.
4. OPEN currently writes **ON / true / 1**.
5. CLOSE currently writes **OFF / false / 0**.
6. The old `ENABLE_PNEUMATICS=false` does **not** block these claw commands. It gates the separate flip mechanism. Availability does reject an ADI conflict if that separate mechanism is enabled on the claw's port.
7. Driver R2 now calls `claw_toggle()` and therefore uses the same semantic state and ADI A object. It also stops flex as before.
8. Exactly one lazy claw `DigitalOut` exists: `claw_piston(...)`. No second autonomous solenoid was added.
9. Polarity is centralized in `mechanism_config::CLAW_OPEN_OUTPUT` in `include/subsystems.hpp`.
10. If physical OPEN/CLOSE are reversed, change **only `CLAW_OPEN_OUTPUT` from true to false**. CLOSE automatically uses its inverse; do not invert route calls.
11. No duplicate autonomous claw implementation was added.

**Software command path: COMPLETE. ADI port: A. Raw OPEN: ON. Raw CLOSE: OFF. Physical polarity: NOT VERIFIED.** Startup retains its existing raw OFF initialization, with semantic state derived from the polarity setting. Successful writes update the shared state. Write errors return false; software state is not proof of cylinder position. CLOSE is also rejected while the optional switch-return descent owns the lift.

Claw state stays commanded until another explicit command changes it. Stop-all does not change it. No solenoid pulses, timed release, pressure manipulation or 20 PSI assumption were added. Physically command OPEN and CLOSE and observe each; then test flex intake in the intended claw state to distinguish polarity, plumbing, solenoid, linkage/geometry and pressure effects.

## Lift: explicit answers and ownership

1. UP follows `lift_full_command(+1, owner)` → `write_voltage(true,12000)` → `write_lift_pair(12000)`. With current configuration, both existing objects receive `+12000`; P15's configured reversal makes its electrical output negative. Electrical result: **P14 +12000, P15 -12000 mV**.
2. DOWN follows the same path with `-12000`. Electrical result: **P14 -12000, P15 +12000 mV**.
3. The electrical signs and reversal compensation exist in one writer, `write_lift_pair`.
4. Driver L1/L2 and autonomous full-voltage functions both use that writer. No reversal or port configuration was changed.
5. `mechanisms_update()` cannot replace an owned full-voltage command with hold/target PID output.
6. `LiftOutputOwner::Driver` / `Autonomous` and `LiftMode::DriverFull` / `AutonFull` identify exclusive ownership. `lift_update()` returns before axis PID while owned. Legacy manual, hold capture, target and switch-return starts cannot compete. Explicit full commands, blocking moves, stop, lifecycle cleanup, and safety stops may transfer/cancel ownership.
7. P16 is read as accumulated centidegrees divided by 100 and the existing **1.0** ratio. It is the output/mechanism angle; no extra 2:1 scaling. Invalid reads return NAN.
8. Measure a reproducible home/reference, minimum/maximum safe output angles, intended HOME/LOW/MID/HIGH and any intake/clearance/score angles, upward sensor sign, and travel time with the actual load. Validate controller gain, voltage cap, tolerance and settle behavior. Existing values are not newly certified measurements.
9. `LIFT_FEEDBACK_VERIFIED=false` and `LIFT_LIMITS_CALIBRATED=false` currently prevent target control. Targets also need initialization, explicit physical homing/zero, finite valid sensor values, no latched axis fault, valid ordered limits and an in-range target. LOW/MID/HIGH additionally require `LIFT_SCORING_POSITIONS_CALIBRATED`.
10. `lift_auton_move_to_deg(target, timeout_ms)` is implemented and built, ready for commissioning after those values/gates are supplied. It uses the existing controller, services mechanisms, checks sensor/fault/mode/timeout, and only succeeds after the existing settle check AND a successful stop. It is not physically proven yet.

| Lift control mode | P14/P15 writer | Lifetime / cancellation |
| --- | --- | --- |
| Current driver L1/L2 | Driver full owner → shared pair writer | Held command; release/both buttons stop; opposite direction stops before restarting next loop |
| Autonomous full | Autonomous full owner → shared pair writer | Latched until explicit change/stop, normal route end, mode/disable cleanup, or safety/write failure |
| Legacy low-voltage manual | Existing axis controller → shared pair writer | Refresh each loop; rejects a full owner; stale command falls back to hold/stop |
| Hold | Existing P/D axis controller → shared pair writer | Requires serviced updates and verified reference/feedback; full owner excludes it |
| Automatic target | Same existing axis controller → shared pair writer | Existing gates/settling; no second PID or task |
| Optional switch return | Existing return state → shared pair writer | Currently unavailable: switch port and timeout both zero |

Existing motor object accessors are `left_lift_motor()` (P14, not reversed) and `right_lift_motor()` (P15, reversed). Current local cartridge configuration is **red 100 RPM**, not the historical green configuration. It was preserved.

The full-voltage API deliberately retains the driver's open-loop commissioning behavior: it does not require P16 homing/feedback calibration, enforce unmeasured angle limits, or invent a travel timeout. Motor-write faults latch separately and prevent restart until explicit reinitialization. A configured bottom limit stops downward output when serviced; unconfigured bottom input is not fabricated. Routine updates are cooperative, not an independent watchdog.

The blocking wrapper stops prior axis output before validating a new request. Invalid target, missing calibration, mode loss, sensor fault or timeout returns false. On success it commands zero and leaves the axis Disabled, **not powered Hold**. Consequently `lift_at_target()` is not a latched success flag after this wrapper returns: use its returned bool. Mechanical BRAKE is not closed-loop position hold and may sag under load. Continuous holding alongside blocking navigation needs deliberate commissioning; no background concurrency was added.

`lift_print_debug()` reports P16 degrees/validity, effective owner/mode, target or NAN, homed state, axis fault and output fault. `mechanisms_print_debug()` includes it with existing throttling.

## Flex, pickup and toggle: explicit answers

**Flex:** driver R1 and autonomous now both call `flex_intake`, which uses one `flex_command`. Reversals remain centralized: P8 false, P9 true; `FLEX_INTAKE_SIGN=1`. Intake intends both wheels inward; outtake reverses the same semantic direction. Commands accept magnitudes 0–127. Both motor writes are attempted, and a failed pair triggers best-effort zero on both. `flex_stop()` always attempts both available motors even if the first reports failure. Hardware errors return false, not proof of a physical stop. Inward/outward meaning and loaded behavior remain unverified. Driver hold power 127 and release power 13 are preserved; **13 is approximately 10% command, not a tuned matchload speed**.

Pickup preparation = OPEN+intake; capture/grab = CLOSED+intake; finish = flex stop with claw unchanged; claw score = OPEN+outtake. All are immediate/nonblocking, with no drive motion or hidden delays. Combined actions accept an explicit power 1–127. Existing no-argument defaults are zero/uncommissioned and therefore safely reject preparation/capture/scoring rather than report collection with stopped wheels. Choose power at the call site now, or configure a measured default later. If a later write fails, flex is stopped but an already successful pneumatic command is not automatically undone; check the returned bool.

**Toggle software: COMPLETE. Port: UNKNOWN (`TOGGLE_PORT=0`, `TOGGLE_CONFIGURED=false`). Direction: UNVERIFIED.** `toggle_run()` currently returns false without constructing a motor. Once a real nonconflicting port is assigned/configured, the same code accepts enabled autonomous or driver calls, runs the existing 200 RPM request until changed, and stops through `toggle_stop()`. Currently `toggle_stop()` returns false because there is no configured motor; it never invents port 0. Driver B uses these same APIs. `TOGGLE_REVERSED` controls direction centrally after physical testing.

## Function audit

The tables cover the project-level autonomous API, mechanism helpers, compatibility/diagnostic entry points and excluded historical module. Library internals are not route APIs. Grouped rows explicitly list functions sharing the same contract.

Column conventions: **Auton** means software callable in enabled autonomous, not proof of physical action. **Validation** uses the categories above. **Changes/stops** identifies later commands or owners that can replace output. Device-writing bools report PROS command acceptance; they do not measure physical completion. Read-only functions have no output to overwrite.

### Navigation, unchanged

| Function | Purpose | Real hardware reached | Auton | Validation | Calibration still needed | Gate | Changes/stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `nav_set_start` | Seed field pose/heading | Chassis odometry + P20 heading | Yes | B | Actual start pose | Route sensor preflight; wait/readback is not bool success | Later pose set/reset, tracking |
| `nav_go_to` | Forward XY | Drive P1–4, trackers P5/P7, P20 | Yes | B | Measured route/arrival | Route preflight; sequence overload also validates | Next drive command, cancel, disable |
| `nav_go_to_reverse` | Reverse XY | Same | Yes | B | Same | Same | Same |
| `nav_go_to_pose` | Forward XY + FRONT heading | Same | Yes | B | Same | Same; sequence pose CHAIN rejected | Same |
| `nav_go_to_pose_reverse` | Reverse XY + FRONT heading | Same | Yes | B | Same | Same | Same |
| `nav_turn_to` | Absolute FRONT heading | Same | Yes | B | Heading/turn validation | Route preflight; sequence validation | Same |
| `nav_drive_distance` | Relative inches, positive FRONT-first | Drive encoders/P20 | Yes | B | Travel/exit validation | Route preflight; sequence validation | Same |
| `nav_follow_path` | Native injected Pure Pursuit | Drive/odom/P20 | Yes | B | Path validation | Nonempty finite points, speed/direction; sequence scope for explicit end | Same |
| `nav_follow_path_pose` | Path with final FRONT heading | Same | Yes | B | Path/final heading | Above plus heading validation | Same |
| `nav_validate_pose` | Compare reported pose to caller tolerances | Read odometry | Yes | B | Trustworthy odometry and tolerances | Caller supplies valid values | None |
| `nav_pose_get`, `nav_theta_get` | Current public pose/heading | Read odometry | Yes | B | Existing odometry verification | Does not itself prove health | None |
| `nav_game_to_internal_pose`, `nav_internal_to_game_pose`, `nav_game_to_internal_heading`, `nav_internal_to_game_heading` | Coordinate/heading normalization | None | Yes | Software only | None; canonical frame preserved | Finite caller data | None |
| `nav_show_pose`, `nav_print_pose` | Pose snapshot | Read odom, LCD/terminal | Yes | B for sensor truth | None new | `NAV_DIAGNOSTICS` | Display writers only |
| `nav_show_result`, `nav_print_result`, `nav_debug_result` | Final error report | Read drive velocity/odom | Yes after motion | B | None new | Diagnostic flag, finite data, measured rest | Display writers only |
| `nav_print_sensor_health` | Live IMU/tracker checks | P5/P7/P20 | Yes | B | Wiring/physical signs not established by detection | Installed/readable/not calibrating | None |
| `nav_print_snapshot`, `nav_print_test_result`, `nav_print_action_record` | Diagnostics | Sensor reads or supplied records | Yes | B for sensor truth | None new | Valid readings; rest for final test report | None |
| `nav_manual_odom_sample` | Manual-push telemetry | P5/P7/P20 reads | Yes | B | Physical push tests | Valid sensors, sample interval | None |
| `NavSequence`, `ok`, `finish` | Existing guarded movement scope | Drive and mechanism service/cleanup | Yes | B | Motion acceptance/route testing | Single task, no nesting, deadlines/rest | Constructor and finish stop mechanisms |
| `nav_events` | One-shot next-move mechanism callbacks | Callback-selected hardware | Yes | B | Event design/validation | Valid sequence; short nonblocking callbacks | Callback commands can replace mechanism output |
| `nav_ok`, `nav_fault` | Sequence state | None | Yes | Software only | None | Existing sequence state | None |
| `nav_abort`, `nav_cancel_sequence` | Abort/lifecycle cleanup | Drive + mechanism stops | Yes | B | Stop verification | Existing ownership rules | Intentionally cancels motor commands |
| `nav_trace_marker`, `nav_trace_dump` | Buffered diagnostics | Trace buffers/reads | Yes | B for telemetry | None new | Dump after finish/rest | None |

Legacy blocking movement overloads do not themselves service mechanism feedback or return reliable arrival success. A returned `void` or accepted path is not proof of reaching the target. Explicit `NavEndMode` overloads require `NavSequence` and retain its existing checks, hooks and cleanup. Construct a sequence **before** starting mechanisms: its constructor stops them. Finish/abort also stops them. These behaviors were not rewritten.

### Claw, flex and pickup

| Function | Purpose | Real hardware reached | Auton | Validation | Calibration still needed | Gate | Changes/stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `claw_open` | Explicit OPEN | ADI A ON | Yes | B | Physical polarity | Config/valid port, enabled, write success | Next claw command only |
| `claw_close` | Explicit CLOSED | ADI A OFF | Yes | B | Physical polarity | Above; no close during return descent | Next claw command only |
| `claw_set` | Explicit semantic state | Same ADI A | Yes | B | Same | Same | Same |
| `claw_toggle` | Invert last successful semantic state | Same ADI A | Yes, explicit states preferred | B | Same | Same | Same; shared with R2 |
| `claw_pneumatic_available` | Configuration validity | None | Yes | Software only | Confirm hardware | Not a connection test | None |
| `flex_intake` | Latched inward request | P8/P9 | Yes | B | Intake sign, selected power | Config, enabled, 0–127, write success | Other flex call, stop-all/lifecycle/sequence cleanup |
| `flex_outtake` | Latched outward request | P8/P9 | Yes | B | Same | Same | Same |
| `flex_stop` | Zero both | P8/P9 | Yes, also disabled | B | Verify stop physically | Available ports; attempts both | Next flex command |
| `flex_available` | Configuration validity | None | Yes | Software only | Hardware verification | Port/sign/conflict checks | None |
| `pickup_prepare` | OPEN + intake | A + P8/P9 | Yes with chosen power | C | Power and polarity/sign | Power 1–127 and underlying gates | Next claw/flex command; motor cleanup |
| `pickup_capture` | CLOSE + intake | A + P8/P9 | Yes with chosen power | C | Same | Same; close-return guard | Same |
| `pickup_finish` | Stop flex, preserve claw | P8/P9 only | Yes | B | Stop validation | Flex available | Next flex call |
| `claw_grab` | Alias of capture semantics | A + P8/P9 | Yes with power | C | Same as capture | Same | Same |
| `claw_score` | OPEN + outtake | A + P8/P9 | Yes with power | C | Outtake power/polarity | Underlying gates | Next command; flex remains active until stopped |

### Lift

| Function | Purpose | Real hardware reached | Auton | Validation | Calibration still needed | Gate | Changes/stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `lift_auton_up_full` | Latched full UP | P14 +12V / P15 -12V electrical | Yes | B; old UP sign evidence | Physical autonomous test | Enabled auton, initialized, no output fault; no return owner | Explicit stop/change, safety/lifecycle; PID cannot overwrite |
| `lift_auton_down_full` | Latched full DOWN | Opposite P14/P15 signs | Yes | B | Same | Above; configured bottom switch | Same |
| `lift_auton_stop` | Cancel lift owner/target, zero both | P14/P15 | Yes, also disabled | B | Stop validation | Attempts both regardless of axis initialization | Next accepted lift command |
| `lift_position_deg` | Output angle | P16 | Yes | B | Reference/sign validation | NAN for invalid read | Read only |
| `lift_print_debug`, `lift_mode` | Effective state/feedback | P16 read / state | Yes | B for sensor truth | None new | None | Read only |
| `lift_set_target` | Nonblocking existing target controller | P16 then P14/P15 on update | Currently rejected | C | Reference, feedback, limits, controller | Feedback/limits/home/fault/target/manual/owner gates | Serviced PID; explicit stop/full command/new target |
| `lift_auton_move_to_deg` | Blocking settled target, then stop | P16/P14/P15 | Currently rejected | C | Above + caller-measured timeout | Above + enabled auton and timeout | Owns sequential move; success/failure zero |
| `lift_move_home` | Nonblocking configured HOME target | Same | Currently rejected | C | Actual home/reference | Target gates | Same as target |
| `lift_move_low` | Nonblocking LOW preset | Same | Currently rejected | C | LOW | Target + scoring-calibrated flag | Same |
| `lift_move_mid` | Nonblocking MID preset | Same | Currently rejected | C | MID | Same | Same |
| `lift_move_high` | Nonblocking HIGH preset | Same | Currently rejected | C | HIGH | Same | Same |
| `lift_move_max` | Nonblocking safe MAX | Same | Currently rejected | C | Safe MAX | Target gates | Same |
| `lift_at_target` | Live settled tolerance check | P16 + axis state | Yes | C | Valid controller commissioning | Active target/hold, fresh update, settled/in tolerance | Read only; stop clears settled state |
| `lift_update` | Service current owner/controller | P14/P15/P16, optional bottom switch | Yes | B/C by mode | As above | Mode/fault/gap/feedback gates | Full owner bypasses PID; safety may zero |
| `lift_initialize` | Stop, brake, initialize feedback state | P14/P15/P16 | Callable; startup preferred | B | Hardware check | No automatic home | Cancels existing axis commands |
| `lift_zero_at_current_position` | Explicit physical reference reset | P16 and P14/P15 zero | Yes when stationary/supported | C | Known physical home | Initialized, no fault/full owner/return | Cancels old axis motion; no movement home |
| `lift_manual_up`, `lift_manual_down`, `lift_up`, `lift_down` | Legacy 1500 mV jog | P14/P15/P16 | Callable but not preferred auton API | B | Feedback/sign verification | Manual-enable, valid sensor, refresh, no full/return owner | Manual release, stale/fault/stop |
| `lift_manual_stop_and_hold`, `lift_stop`, `lift_cancel_auto` | Existing hold/cancel behavior | P16/P14/P15 | Yes | C | Homing/feedback/tuning | Hold requires feedback; full owner protected | PID hold, new command, stop |
| `lift_return_home_start`, `mechanism_return_home` | Optional switch descent then zero/close | P14/P15/P16 + switch + A | Currently rejected | D | Switch binding/polarity, measured timeout | Port=0, timeout=0, other existing return gates | Stop/fault/disable/mode change; closes claw only on success |
| `lift_return_home_active`, `lift_bottom_limit_read` | Return/switch state | Optional input; none at port 0 | Yes | D for switch | Switch assignment | Unbound returns Unconfigured | Read only |

### Toggle, flip, arm and common lifecycle

| Function | Purpose | Real hardware reached | Auton | Validation | Calibration still needed | Gate | Changes/stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `toggle_run` | Latched velocity request | None currently | Implemented, currently false | D | Real port, direction/gearing | `TOGGLE_CONFIGURED=false`, port0; enabled mode | Stop/next command/cleanup once configured |
| `toggle_stop` | Zero toggle | None currently | Implemented, currently false | D | Port | Availability; stop allowed disabled | Next run |
| `toggle_available` | Config validity | None | Yes | D | Port | Conflict checks | None |
| `color_toggle_set` | Compatibility run/stop alias | Toggle above | Currently false | D | Same | Same | Same |
| `claw_flip_set`, `claw_flip_forward`, `claw_flip_reverse`, `claw_flip_toggle` | Explicit/toggled flip state | ADI B if enabled | Currently false | B, inactive | Presence, wiring/polarity | `ENABLE_PNEUMATICS=false`, enabled mode | Next flip call; stop-all preserves it |
| `arm_set_target` | Existing nonblocking target | P17/P18 | Currently rejected | C, inactive | Presence, reference, coupling, limits, tuning | Arm feedback/limit/home/fault gates | Axis update/new command/stop |
| `arm_auton_move_to_deg` | Sequential settled move, then stop | P17/P18 | Currently rejected | C, inactive | Above + measured timeout | Above + enabled auton | Stops success/failure |
| `arm_move_home` | HOME preset | P17/P18 | Currently rejected | C | Home | Target gates | Same |
| `arm_move_score`, `arm_move_max_lift_position` | Existing score/max-lift presets | P17/P18 | Currently rejected | C | Existing 90/135 values NOT validated | Target + scoring flag | Same; optional detection can request score when enabled |
| `arm_position_deg`, `arm_at_target`, `arm_mode` | Read position/settled/mode | P18/state | Yes | C | Reference/controller commissioning | Invalid sensor → NAN; settle freshness | Read only |
| `arm_initialize`, `arm_zero_at_current_position` | Initialize / explicit reference | P17/P18 | Callable | C | Known physical home | Initialize stops; zero requires no fault | Cancels axis command |
| `arm_manual_up`, `arm_manual_down` | Legacy jog | P17/P18 | Disabled by config | C | Physical commissioning | `ENABLE_ARM_MANUAL=false`, axis gates | Refresh/release/stop |
| `arm_manual_stop_and_hold`, `arm_cancel_auto` | Capture hold/cancel target | P17/P18 | Currently hold unavailable | C | Feedback/home/tuning | Existing axis gates | Axis update/new command/stop |
| `arm_update` | Existing axis service | P17/P18 | Via `mechanisms_update` | C | Same | Existing fault/gap/feedback gates | Writes active arm controller |
| `claw_object_detected` | Cached detection | P19 sampled by update if enabled | Currently false | C | Presence/range/confidence | Distance feature/calibration flags false, freshness | Read only; enabled detection may request arm score |
| `mechanisms_initialize` | Startup initialization | Configured mechanisms, P14–18, A | Startup use | B/C | Per mechanism | Resets states; existing raw A OFF startup | Cancels old commands; not a route action |
| `mechanisms_update` | Cooperative axis and sensor service | Active mechanisms | Yes | B/C | Per mechanism | Disabled triggers stop-all | Full lift ownership protected; optional detection/home input may issue gated requests |
| `stop_all_mechanisms` | Best-effort motor stop/cancel | Available flex/toggle; initialized lift/arm | Yes, also disabled | B | Physical stop verification | Attempts each group without short-circuiting later groups | Cancels full/manual/target/return and driver flex latch; NO pneumatic changes |
| `mechanisms_print_debug` | Throttled state output | Sensor/state reads | Yes | B for sensor truth | None new | Existing debug period | None |

Arm initialization still attempts the existing P17/P18 hardware; active arm motion remains gated off. If absent, its fault can make stop-all report false. No new arm hardware was enabled. `stop_all_mechanisms()` skips unconfigured flex/toggle devices; success means every applicable stop write succeeded, not that every planned mechanism is installed.

### Action adapters, dispatch and diagnostic helpers

| Function | Purpose | Real hardware reached | Auton | Validation | Calibration still needed | Gate | Changes/stops |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `auton_actions::drive_to_point`, `drive_to_point_reverse`, `drive_to_pose`, `turn_to_heading`, `approach_goal` | Synchronous navigation adapters | Existing nav hardware | Yes | B | Route/arrival validation | Input/enabled-auton checks | Existing nav commands; failure cleanup |
| `auton_actions::follow_path`, `run_sequence` | Optional path/sequence adapters | Existing nav/optional mechanism callbacks | Currently feature-disabled | B | Path/hook validation | `ENABLE_ADVANCED_PATHS=false`; hooks also mechanism gate | Sequence cleanup |
| `auton_actions::mechanism_action` | Checked synchronous subsystem callback | Caller-selected mechanism | Currently feature-disabled | B/C | Underlying mechanism | `ENABLE_MECHANISM_ACTIONS=false` | Callback/failure cleanup |
| `auton_actions::collect_object(power)` | Immediate capture | A, P8/P9 | Implemented, feature-disabled | C | Intake power/polarity | Mechanism gate, valid power, underlying gates | Flex continues until stop |
| `auton_actions::score_object(power)` | Immediate open/outtake | A, P8/P9 | Implemented, feature-disabled | C | Outtake power/polarity | Same | Same |
| `auton_actions::score_high(timeout,power)`, `score_low(timeout,power)` | Settled preset lift, stop, then open/outtake | P14–16, A, P8/P9 | Implemented, gated | C | Measured preset/timeout/power | Scoring + feedback/limits + mechanism gate | Sequential wrapper stops lift; flex continues |
| `collect_object(Context)`, `score_object(Context)` | Compatibility using configured power | Same after commissioning | Currently Unconfigured | C | Defaults currently zero | Config power and adapter gates | Same |
| `score_high(Context)`, `score_low(Context)` | Compatibility with no measured timeout | None | Returns Unconfigured | C | Caller must use explicit-timeout overload | No timeout invented | None |
| `auton_actions::correct_pose_from_wall` | Explicit stationary callback | Caller-selected correction | Feature-disabled; module excluded | D | Hardware/reference calibration | Distance adapter flag false; do not enable excluded module blindly | Callback may reset pose |
| `auton_actions::cancel_route` | Cancel route | Drive/mechanism stops | Yes | B | Stop verification | None | Stops motors |
| `status_name`, `ActionResult::can_continue` | Result interpretation | None | Yes | Software only | None | ReturnedUnverified is NOT physical success | None |
| `mirror_x` (both overloads), `mirror_normalized_heading`, `run_helper_math_test` | Geometry math/test | None | Yes | Software only | Caller supplies mirror axis | Valid inputs | None |
| `register_autons` | Register permanent functions | Brain selector UI | Startup | Software only | Actual route design | Development dispatch flags | No motion |
| `auton_left`, `auton_right`, `auton_safe`, `skills` | Permanent route locations | None yet | Dispatch ready, routes unconfigured | Route content pending | Actual field route and mechanism measurements | Navigation preflight | No motion yet |
| `custom_auton_test` | Isolated preexisting temporary routine | Drive and A | Only custom mode | B | Existing test validation | Custom gate + nav preflight | Exit cleanup |
| `auton_example_template` | Inert development example | None until explicit configuration | Not selected | Route content pending | Real start/approach | Unset optionals and false enable | No motion by default |
| `run_dev_test` | Existing named progression | Selected sensors/drive | Yes, current default | B | Physical progression | IMU/preflight; advanced tests disabled | Existing test stops/route cleanup |
| `claw_flex_test`, `toggle_test`, `pickup_sequence_test` | Explicit mechanism test steps | Underlying mechanisms | Yes when gates satisfied | B/C/D | Chosen power, polarity or port | Underlying availability; pickup defaults uncommissioned | Explicit Stop/Finish required |
| `test_lift_motor_response` | Current L1/L2 driver full commissioning | P14/P15, P16 telemetry | **No; driver only** | Prior UP signs, new refactor B | Physical regression test | Driver enabled, output write success | Release/both stops; uses shared full writer |
| `claw_driver_control` | Current R1/R2 inputs | Same flex and claw APIs | **No; driver only** | B | Physical regression test | Driver enabled | R2 stops flex/toggles A; R1 resumes intake |
| `test_lift_manual`, `test_arm_manual`, `test_lift_hold`, `test_lift_target`, `test_arm_target` | Legacy explicit commissioning helpers | Axis hardware | Callable via underlying gates | C | Axis commissioning | Existing manual/feedback/target gates | Axis controller/stop |
| `test_lift_rotation_sensor`, `test_arm_rotation_sensor` | Read-only telemetry | P16/P18 | Yes | B | Sensor/reference validation | Debug throttle | None |
| `test_claw`, `test_claw_flip` | Compatibility semantic calls | A / gated B | Yes / gated | B | Polarity | Underlying command gates | Next pneumatic command |
| `default_constants` | Existing chassis tuning setup | Chassis controller config | Startup | Existing tuning, preserved | Physical tuning outside this task | No automatic route use | Changes controller settings; do not call mid-move |

Direct subsystem calls do **not** require `ENABLE_MECHANISM_ACTIONS`; that retained flag applies only to optional `auton_actions` adapters/hooks. `collect_object` is capture only; it does not secretly approach an object. Scoring adapters do not add settling delays for pneumatics or assume object release has physically finished.

Other unchanged navigation diagnostics from `autons.hpp`:

| Functions | Purpose/hardware | Auton/gate | Validation/calibration | Changes/stops |
| --- | --- | --- | --- | --- |
| `measure_offsets`, `measure_tracking_offsets` | Existing turn/report offset routine; drive/trackers/IMU | Explicit test call | B; actual offset measurement | Diagnostic drive commands |
| `run_turn_drift_test`, `odom_geometry_turn_90_test`, `odom_turn_90_speed_40_test`, `odom_turn_90_speed_60_test`, `odom_turn_90_speed_100_test`, `odom_turn_180_speed_40_test`, `odom_turn_360_speed_40_test` | Existing turn tests; drive/odom | Explicit tests | B; physical drift/geometry checks | Reset/test drive/stop as already implemented |
| `test_go_to`, `test_go_to_pose`, `test_turn_to`, `test_drive_distance` | Existing nav diagnostics | Explicit tests | B; route results | Existing navigation output |
| `odom_clean_point_test`, `odom_first_coordinate_test`, `odom_forward_return_test`, `odom_two_point_coordinate_test`, `odom_frame_exit_test` | Existing field-frame diagnostics | Explicit tests | B; physical route/geometry checks | Existing pose setup and drive |
| `motion_chaining_test`, `auton_two_move_chain_test` | Existing chain diagnostics | Explicit/advanced DevTest gate | B; chain validation | Existing chain output/cleanup |
| `odom_start_screen_test`, `odom_trace_screen_show`, `odom_diagnostic_screen_is_active`, `odom_diagnostic_screen_release`, `odom_display_freeze` | Screen state/pose diagnostics | Callable; display ownership | Software complete; displayed pose still requires valid sensors | Screen ownership; start-screen test seeds pose |

`motion_exit_trace` and `path_motion_trace` contain existing instrumentation hooks, not match actions: `pose_exit_trace_begin/finish`, `motion_exit_trace_arm/begin/finish/show`, `motion_controller_snapshot_start`, `turn_velocity_trace_begin/record_result/result`, `final_turn_trace_arm/target/exit/wait_returned/nav_returned`, `path_motion_trace_arm/begin/exit/wait_returned/stop_and_dump`. They are retained; stop-and-dump explicitly stops drive. No new telemetry loops were inserted into drivetrain motion.

### Excluded historical distance module

`src/distance_correction.cpp` remains excluded by the current Makefile. It declares historical sensor ports 8–13; **P8/P9 now belong to flex motors**. Do not link/enable it until real nonconflicting distance hardware is supplied and the legacy routines are separately reviewed. Its headers alone do not make these functions available in this build. This audit did not rewrite excluded navigation/correction code.

| Functions | Purpose/hardware intended | Works in this build | Status/calibration/gate | Output overwrite |
| --- | --- | --- | --- | --- |
| `dist_mm_to_in`, `dist_project_perpendicular`, `dist_angle_safe_for_reset` | Conversion/projection math | No linked implementation | Excluded module; finite caller data | None |
| `dist_read_front_left_in`, `dist_read_front_right_in`, `dist_read_back_left_in`, `dist_read_back_right_in`, `dist_read_left_in`, `dist_read_right_in` | Distance readings on placeholder P8–13 | No | D; real assignments/mounts/confidence | None |
| `dist_front_average_in`, `dist_back_average_in`, `dist_front_difference_in`, `dist_back_difference_in`, `dist_front_readings_valid`, `dist_back_readings_valid`, `dist_side_readings_valid`, `dist_print_debug` | Derived reads/diagnostics | No | D; same | None |
| `dist_correct_front_distance`, `dist_correct_back_distance`, `dist_square_to_front_surface`, `dist_square_to_back_surface` | Legacy distance-based drive correction | No | D; hardware/tuning/signs/timeouts and separate safety review | Would command drivetrain |
| `dist_reset_y_from_front_reference`, `dist_reset_x_from_front_reference`, `dist_reset_y_from_back_reference`, `dist_reset_x_from_back_reference`, `dist_reset_x_from_right_reference`, `dist_reset_x_from_left_reference`, `dist_reset_y_from_right_reference`, `dist_reset_y_from_left_reference`, `dist_reset_pose_from_front_reference` | Legacy field reference resets | No | D; real sensor offsets/reference geometry and review | Would change odometry |

## Example only — not installed in a route

All uppercase identifiers below are caller-supplied **measured/configured placeholders**, not new values. The autonomous dispatcher supplies preflight and final motor cleanup. This uses sequential lift positioning; no automatic concurrent lift motion is introduced. Navigation return is still not proof of physical arrival; validate against the team's chosen acceptance criteria.

```cpp
void example_only() {
  nav_set_start(START_X, START_Y, START_HEADING);
  if (!pickup_prepare(CHOSEN_INTAKE_POWER)) return;
  nav_go_to(PICKUP_X, PICKUP_Y, TRAVEL_SPEED);
  if (!pickup_capture(CHOSEN_INTAKE_POWER)) return;
  if (!pickup_finish()) return;

  nav_go_to_pose(SCORE_X, SCORE_Y, SCORE_HEADING, APPROACH_SPEED);
  if (!lift_auton_move_to_deg(MEASURED_SCORE_DEG, MEASURED_TIMEOUT_MS)) return;
  if (!claw_open()) return;
  if (!lift_auton_move_to_deg(MEASURED_HOME_DEG, MEASURED_TIMEOUT_MS)) return;

  // Only after the real toggle port has been assigned and validated.
  if (!toggle_run()) return;
  nav_go_to(NEXT_X, NEXT_Y, TRAVEL_SPEED);
  if (!toggle_stop()) return;
}
```

Validation performed: source audit, ownership/error-path review, successful PROS compile/link/binary generation, protected-file hash comparison, Git diff/status review. No runtime simulation, upload, motor actuation or physical validation was performed. No commit or push was made.
