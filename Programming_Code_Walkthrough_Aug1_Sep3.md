# Programming Code Walkthrough: August 1 through September 3, 2026

## Evidence baseline

These line numbers correspond to the workspace state on September 3, 2026. This folder is not a Git repository, so no commit hash is available. If the code changes, locate each screenshot again using the function or constant anchors instead of relying only on the recorded line number.

The current `Makefile` compiles `src/main.cpp` and deliberately excludes `src/autons.cpp`, `src/navigation.cpp`, `src/distance_correction.cpp`, and `src/subsystems.cpp`. Those files are preserved engineering work, but they are not active in the current robot program. Screenshots of them must be described as planned architecture, test routines, or source under development.

## Current implementation status

| Feature | Status on September 3 | Evidence and limitation |
|---|---|---|
| Turn sensitivity | CODED AND COMPILED; DRIVER TESTING REQUIRED | `TURN_MULTIPLIER = 1.30` is active. It has not been documented as physically or driver tested. |
| Joystick curve | CODED AND COMPILED AS NEUTRAL | EZ forward and turn curves are `0.0`. The old three-profile code is comments only. |
| EZ active brake | CODED AND COMPILED; PHYSICAL TUNING REQUIRED | `DRIVER_ACTIVE_BRAKE_KP = 2.0` is a starting value, not a final value. |
| PROS brake mode | CODED AND COMPILED; PHYSICAL TEST REQUIRED | `opcontrol()` selects `MOTOR_BRAKE_BRAKE`. |
| Controller input | PHYSICALLY TESTED | Left Y and Right X changed on the Brain during the temporary diagnostic. |
| Motor direction | PHYSICALLY TESTED | The recorded tests support left `{1, -4}` and right `{3, -2}`. |
| Driver evaluation | NOT YET DOCUMENTED | The driver still needs to test turning feel and stopping behavior. |
| Autonomous/navigation | SOURCE PRESENT; EXCLUDED FROM CURRENT BUILD | Do not call it active competition code yet. |
| Distance correction | SOURCE PRESENT WITH PLACEHOLDERS; EXCLUDED | Sensor ports, signs, geometry, and tuning need physical validation. |
| Mechanisms | PLACEHOLDER SOURCE; EXCLUDED | Ports, reversals, cartridges, and device names are marked TODO. |

## Current-code mismatches and cautions

- The active driver mixer is custom split-arcade math. `chassis.opcontrol_arcade_standard(ez::SPLIT)` is not currently called.
- EZ-Template still owns the drive and receives the final side commands through `chassis.opcontrol_joystick_threshold_iterate()`, which preserves the active-brake path.
- The old SOFT/NORMAL/AGGRESSIVE profile code and D-pad selection code are ordinary comments; they cannot run.
- `autonomous()` in the active `src/main.cpp` is empty. The older selector and autonomous callback survive only in the backup.
- The current build excludes every advanced source file listed above.
- IMU port 7, effective wheel RPM, distance ports 8-13, sensor mounting, and mechanism hardware all need personal verification.
- There is no implemented claw subsystem and no lift level/step logic in the current source. Do not invent screenshots or claims for them.

## Best screenshots for judges

Use roughly these 16 screenshots in the main notebook. The full entries later in this document provide all required start/end anchors and explanations.

| Order | Full-list ID | Priority | File and lines | Anchor | Suggested caption |
|---|---|---|---|---|---|
| 1 | S02 | ESSENTIAL | `src/main.cpp`, 5-11 | `ez::Drive chassis(` | "Final EZ-Template drivetrain configuration based on individual motor-direction tests." |
| 2 | S03 | ESSENTIAL | `src/main.cpp`, 14-19 | `pros::Controller master` | "One-controller setup and centralized driver-control tuning values." |
| 3 | S05 | GOOD TO INCLUDE | `src/main.cpp`, 66-80 | `apply_driver_deadzone()` | "Neutral noise rejection and legal motor-output clamping for driver control." |
| 4 | S06 | ESSENTIAL | `src/main.cpp`, 82-94 | `driver_arcade_control()` | "Turn amplification added after driver feedback requested stronger rotation with less stick travel." |
| 5 | S07 | ESSENTIAL | `src/main.cpp`, 96-110 | `initialize()` | "Neutral EZ curves and a starting active-brake gain configured before chassis initialization." |
| 6 | S08 | ESSENTIAL | `src/main.cpp`, 125-152 | `opcontrol()` | "Current split-arcade driver loop with BRAKE mode and live input diagnostics." |
| 7 | S09 | ESSENTIAL | `src/autons.cpp`, 12-24 | `default_constants()` | "Autonomous PID values remain separate from driver turn sensitivity." |
| 8 | S10 | ESSENTIAL | `src/autons.cpp`, 26-47 | `// Exit conditions` | "Autonomous exit, chaining, slew, and odometry behavior settings." |
| 9 | S14 | ESSENTIAL | `include/navigation.hpp`, 5-35 | `// Absolute coordinate convention` | "Absolute field-coordinate convention and navigation interface." |
| 10 | S15 | ESSENTIAL | `src/navigation.cpp`, 25-64 | `nav_set_start()` | "EZ-Template odometry wrappers for forward, reverse, pose, and heading targets." |
| 11 | S19 | ESSENTIAL | `src/autons.cpp`, 114-141 | `odom_forward_absolute_test()` | "Absolute-position tests for returning to the origin and driving a square." |
| 12 | S21 | ESSENTIAL | `src/distance_correction.cpp`, 8-39 | `DISTANCE_ENABLE_DEBUG` | "Planned sensor ports and bounded correction settings, clearly marked for hardware validation." |
| 13 | S23 | ESSENTIAL | `src/distance_correction.cpp`, 88-105 | `confidence_valid()` | "Distance readings are accepted only after confidence and range checks." |
| 14 | S25 | ESSENTIAL | `src/distance_correction.cpp`, 139-173 | `correct_distance_with_average()` | "Bounded distance correction with invalid-reading protection and timeout." |
| 15 | S27 | ESSENTIAL | `src/distance_correction.cpp`, 209-248 | `reset_axis_from_sensor()` | "One-axis odometry correction preserves the other position state." |
| 16 | S28 | ESSENTIAL | `src/distance_correction.cpp`, 251-267 | `dist_mm_to_in()` | "Unit conversion, cosine projection, and angle-safety math for sensor pose correction." |

## Screenshots to Take on September 3

Take these current driver-control screenshots first:

1. S02: final motor mapping and drivetrain dimensions.
2. S03: one master controller and the 1.30, 2.0, and 5 driver constants.
3. S05: deadzone and output clamping.
4. S06: amplified turn calculation and split-arcade mixing.
5. S07: neutral EZ curves and active-brake initialization.
6. S08: BRAKE mode, joystick reads, current driver loop, and LCD values.
7. S01: temporary drivetrain-only build scope, for honest status documentation.

Then capture recoverable historical evidence from the backup:

8. S46: original all-positive motor ports before physical direction testing.
9. S48: earlier LCD initialization and neutral curve/disabled active-brake state.
10. S49: earlier autonomous selector and controller-rumble feedback.
11. S53: recoverable Left Y/Right X controller diagnostic.

## Historical Code I Should Try to Recover

### Recoverable from the current backup

The file `diagnostic_backups/main.cpp.before_minimal_execution_test.bak` is available directly. No Git checkout is required or possible.

- Original positive drivetrain mapping: lines 23-34, detailed as S46.
- Commented tracking-wheel example: lines 36-42, detailed as S47.
- Earlier LCD initialization, curve configuration, and disabled active brake: lines 50-71, detailed as S48.
- Earlier autonomous selector and IMU-dependent controller rumble: lines 80-106, detailed as S49.
- Earlier autonomous callback: lines 142-149, detailed as S50.
- Earlier EZ screen task and competition-state behavior: lines 154-202, detailed as S51.
- Earlier PID/autonomous controller shortcuts: lines 209-225, detailed as S52.
- Earlier Left Y/Right X LCD diagnostic: lines 246-257, detailed as S53.

### Not recoverable from this workspace

- `PROGRAM IS RUNNING` minimal execution test: Historical screenshot unavailable - code was temporary and is not recoverable from the current repository.
- A/B/X/Y individual motor-direction test: Historical screenshot unavailable - code was temporary and is not recoverable from the current repository.
- Temporary direct four-motor joystick drive: Historical screenshot unavailable - code was temporary and is not recoverable from the current repository.

For those three tests, use a notebook flowchart, a dated test-result table, or a photograph of the physical test setup. Do not recreate or fabricate a code screenshot.

## Documentation claims to verify before judging

- Verify that the Brain IMU is actually connected to port 7.
- Verify 2.75-inch drive wheels and the gearing that produces 450 RPM effective wheel speed.
- Record the driver's result for `TURN_MULTIPLIER = 1.30`; do not call it optimized beforehand.
- Physically compare active-brake behavior before calling `2.0` successful or final.
- Verify that BRAKE plus active brake does not cause oscillation, chatter, or unwanted mechanical shock.
- Do not describe advanced modules as active while the Makefile exclusion remains.
- Replace and test placeholder distance and mechanism ports before presenting them as installed devices.
- Record actual odometry, distance-correction, chaining, and scoring-approach results before claiming reliability.

## Full programming screenshot list

This appendix is intentionally exhaustive. It is a source map, not a recommendation to place every screenshot in the main notebook.

### S01 - Temporary build isolation

- Priority: GOOD TO INCLUDE
- Notebook section: Evidence baseline and controlled drivetrain isolation
- File: `Makefile`
- Current lines: 35-37
- Function/constant: `EXCLUDE_SRCDIRS`
- Start at: `# TEMP execution diagnostic: build only main.cpp`
- End after: `EXCLUDE_SRCDIRS+=$(SRCDIR)/autons.cpp ...`
- Make sure visible: all four excluded source files and the reason for the temporary build scope.
- Why: It proves the driver test was isolated and prevents accidental claims that unfinished modules are active.
- Suggested caption: "Temporary main-only build used to isolate driver-control testing from unfinished advanced systems."

### S02 - EZ-Template drivetrain configuration

- Priority: ESSENTIAL
- Notebook section: Four-motor drivetrain configuration
- File: `src/main.cpp`
- Current lines: 5-11
- Function/constant: `ez::Drive chassis`
- Start at: `ez::Drive chassis(`
- End after: `600.0 / 450.0);`
- Make sure visible: `{1, -4}`, `{3, -2}`, IMU port 7, 2.75-inch wheels, 600 RPM cartridges, and 450 RPM effective output ratio.
- Why: It connects the physical one-motor-at-a-time direction test to the final software mapping.
- Suggested caption: "Updated EZ-Template chassis configuration after individually testing drivetrain motor direction."

### S03 - One controller and driver constants

- Priority: ESSENTIAL
- Notebook section: Driver-control architecture and September 3 tuning
- File: `src/main.cpp`
- Current lines: 13-19
- Function/constant: `master`, `TURN_MULTIPLIER`, `DRIVER_ACTIVE_BRAKE_KP`, `DRIVER_DEADZONE`
- Start at: `// The robot uses one master controller`
- End after: `constexpr int DRIVER_DEADZONE = 5;`
- Make sure visible: one master controller, 1.30 turn multiplier, 2.0 active-brake kP, and deadzone 5.
- Why: It places the three driver-feel settings in one reviewable location.
- Suggested caption: "Single-controller configuration with centralized turn, active-brake, and deadzone tuning values."

### S04 - Disabled curve-profile experiment

- Priority: OPTIONAL
- Notebook section: Investigated alternatives
- File: `src/main.cpp`
- Current lines: 21-64
- Function/constant: commented `DriveProfile`, `apply_drive_profile()`, and selection helpers
- Start at: `// Optional driver response profiles.`
- End after: the commented closing brace of `select_softer_profile()`
- Make sure visible: every line is commented, curve values 6.0/3.0/0.0, and the calls to `opcontrol_curve_default_set()`.
- Why: It documents an investigated EZ-Template curve option without pretending the profiles are active.
- Suggested caption: "Earlier EZ curve-profile experiment retained as comments after determining that nonnegative curves soften response rather than amplify it above linear."

### S05 - Driver deadzone and output clamping

- Priority: GOOD TO INCLUDE
- Notebook section: Turn-sensitivity safety limits
- File: `src/main.cpp`
- Current lines: 66-80
- Function/constant: `apply_driver_deadzone()` and `clamp_driver_output()`
- Start at: `int apply_driver_deadzone(int input) {`
- End after: the closing brace of `clamp_driver_output()`
- Make sure visible: neutral range test, the +127 clamp, the -127 clamp, and integer conversion.
- Why: It shows that turn amplification cannot exceed PROS's legal motor-command range and does not amplify neutral noise.
- Suggested caption: "Driver input helpers reject small neutral noise and clamp every command to -127 through +127."

### S06 - Turn amplification and arcade mixing

- Priority: ESSENTIAL
- Notebook section: Driver feedback - increased turn sensitivity
- File: `src/main.cpp`
- Current lines: 82-94
- Function/constant: `driver_arcade_control()`
- Start at: `void driver_arcade_control(int forwardInput, int turnInput) {`
- End after: `chassis.opcontrol_joystick_threshold_iterate(leftOutput, rightOutput);`
- Make sure visible: deadzone use, `turn * TURN_MULTIPLIER`, `forward + amplifiedTurn`, `forward - amplifiedTurn`, both clamps, and the EZ-Template handoff.
- Why: This is the exact implementation of the driver's request for stronger turning with less Right X travel.
- Suggested caption: "Custom split-arcade mixing amplifies only the turn component, clamps both sides, and returns control to EZ-Template's active-brake path."

### S07 - Curve and active-brake initialization

- Priority: ESSENTIAL
- Notebook section: Driver feedback - forceful stopping
- File: `src/main.cpp`
- Current lines: 96-110
- Function/constant: `initialize()`
- Start at: `void initialize() {`
- End after: `pros::lcd::set_text(0, "EZ DRIVE READY");`
- Make sure visible: LCD initialization, curve buttons disabled, both curves set to 0.0, active-brake kP setup, and `chassis.initialize()`.
- Why: It proves the project uses neutral EZ curves while enabling the starting active-brake value.
- Suggested caption: "Initialization keeps EZ joystick curves linear and enables a starting driver active-brake gain of 2.0."

### S08 - Current driver-control loop

- Priority: ESSENTIAL
- Notebook section: Current driver control
- File: `src/main.cpp`
- Current lines: 125-152
- Function/constant: `opcontrol()`
- Start at: `void opcontrol() {`
- End after: the final closing brace after `pros::delay(ez::util::DELAY_TIME);`
- Make sure visible: `MOTOR_BRAKE_BRAKE`, Left Y and Right X reads, `driver_arcade_control()`, LCD diagnostics, and the loop delay.
- Why: It shows exactly what runs during driver control and confirms the old D-pad profile controls are disabled comments.
- Suggested caption: "Current one-controller driver loop uses BRAKE mode, split-arcade inputs, turn amplification, and live Brain diagnostics."

### S09 - Autonomous PID constants

- Priority: ESSENTIAL
- Notebook section: Autonomous control constants
- File: `src/autons.cpp`
- Current lines: 12-24
- Function/constant: `DRIVE_SPEED`, `TURN_SPEED`, and `default_constants()`
- Start at: `// These are out of 127`
- End after: `chassis.pid_odom_boomerang_constants_set(...)`
- Make sure visible: drive, heading, turn, swing, odom angular, and boomerang PID calls.
- Why: It distinguishes autonomous closed-loop tuning from driver joystick sensitivity.
- Suggested caption: "Independent autonomous PID constants for drive, heading, turn, swing, and odometry motion."

### S10 - Exit, chaining, slew, and odometry settings

- Priority: ESSENTIAL
- Notebook section: Autonomous reliability and motion strategy
- File: `src/autons.cpp`
- Current lines: 26-47
- Function/constant: remainder of `default_constants()`
- Start at: `// Exit conditions`
- End after: the closing brace following `chassis.pid_angle_behavior_set(ez::shortest);`
- Make sure visible: every exit condition, chain constant, slew setting, odom look-ahead/bias setting, and shortest-angle behavior.
- Why: It documents how planned autonomous moves decide when to finish, chain, accelerate, and choose a turn path.
- Suggested caption: "Autonomous completion tolerances, chaining thresholds, slew limits, and odometry motion settings."

### S11 - Relative drive PID test

- Priority: GOOD TO INCLUDE
- Notebook section: Basic autonomous validation
- File: `src/autons.cpp`
- Current lines: 49-64
- Function/constant: `drive_test()`
- Start at: `// Runs a simple relative drive test`
- End after: the closing brace of `drive_test()`
- Make sure visible: the 24-inch forward target, two -12-inch targets, speed, slew choice, and every `pid_wait()`.
- Why: It shows a small repeatable test used before attempting a complex route.
- Suggested caption: "Relative drive PID test separates basic distance validation from full autonomous development."

### S12 - Absolute-heading turn test

- Priority: GOOD TO INCLUDE
- Notebook section: Basic autonomous validation
- File: `src/autons.cpp`
- Current lines: 66-79
- Function/constant: `turn_test()`
- Start at: `// Runs a simple turn test`
- End after: the closing brace of `turn_test()`
- Make sure visible: 90, 45, and 0 degree targets with `pid_wait()` calls.
- Why: It provides a focused autonomous turn-PID test that is separate from driver Right X response.
- Suggested caption: "Autonomous heading test checks repeatable turns independently from driver-control sensitivity."

### S13 - Placeholder mechanism test routines

- Priority: OPTIONAL
- Notebook section: Planned subsystem bring-up
- File: `src/autons.cpp`
- Current lines: 81-112
- Function/constant: `intake_test()` and `full_system_test()`
- Start at: `// Runs intake and outtake briefly`
- End after: the closing brace of `full_system_test()`
- Make sure visible: timed intake, pneumatic, lift, and scoring calls plus the word `placeholder`.
- Why: It shows a staged wiring-test plan, but it must not be presented as verified hardware behavior.
- Suggested caption: "Short placeholder subsystem tests prepared for future wiring and direction checks."

### S14 - Absolute navigation interface

- Priority: ESSENTIAL
- Notebook section: Navigation architecture
- File: `include/navigation.hpp`
- Current lines: 5-35
- Function/constant: coordinate convention and core `nav_*` declarations
- Start at: `// Absolute coordinate convention:`
- End after: `void nav_turn_to(double theta_deg, int speed);`
- Make sure visible: +X/+Y convention, heading convention, fixed start-pose warning, forward, reverse, pose, and turn declarations.
- Why: It explains the coordinate system before showing any route code.
- Suggested caption: "Navigation interface defines one fixed absolute coordinate system and explicit forward, reverse, pose, and heading commands."

### S15 - Core navigation implementation

- Priority: ESSENTIAL
- Notebook section: Navigation architecture
- File: `src/navigation.cpp`
- Current lines: 25-64
- Function/constant: `nav_set_start()`, `nav_go_to()`, `nav_go_to_pose()`, reverse variants, and `nav_turn_to()`
- Start at: `void nav_set_start(double x_in, double y_in, double theta_deg) {`
- End after: the closing brace of `nav_turn_to()`
- Make sure visible: `chassis.odom_xyt_set()`, forward/reverse direction flags, odom target calls, absolute turn target, and wait calls.
- Why: It shows how the documented coordinate model is translated into EZ-Template commands.
- Suggested caption: "Navigation wrappers translate absolute field targets into EZ-Template odometry and heading commands."

### S16 - Pose validation and debugging

- Priority: GOOD TO INCLUDE
- Notebook section: Navigation verification
- File: `src/navigation.cpp`
- Current lines: 68-89
- Function/constant: `nav_validate_pose()` and `nav_print_pose()`
- Start at: `bool nav_validate_pose(`
- End after: the closing brace of `nav_print_pose()`
- Make sure visible: X/Y/heading error checks and Brain/terminal pose output.
- Why: It documents how planned movement can be checked quantitatively instead of judged only by appearance.
- Suggested caption: "Navigation verification compares expected and measured pose and reports the current odometry state."

### S17 - Navigation distance-correction interface

- Priority: GOOD TO INCLUDE
- Notebook section: Sensor-assisted navigation architecture
- File: `include/navigation.hpp`
- Current lines: 46-90
- Function/constant: distance correction, squaring, and one-axis reset declarations
- Start at: `// Optional distance wrapper: corrects final front distance`
- End after: `nav_reset_x_with_left_distance(...)`
- Make sure visible: front/back correction, front/back squaring, and front/back/side pose-reset wrappers.
- Why: It shows that distance sensors are planned as a final correction layer around odometry, not as a replacement for navigation.
- Suggested caption: "Optional navigation wrappers isolate distance trimming, wall squaring, and one-axis pose correction."

### S18 - Chaining and speed presets

- Priority: GOOD TO INCLUDE
- Notebook section: Autonomous path strategy
- File: `src/navigation.cpp`
- Current lines: 176-215
- Function/constant: `nav_chain_to()`, `nav_chain_to_reverse()`, and speed preset helpers
- Start at: `void nav_chain_to(double x_in, double y_in, int speed) {`
- End after: the closing brace of `nav_precise_to_pose_reverse()`
- Make sure visible: `pid_wait_quick_chain()`, fast/controlled/precise speed constants in use, and forward/reverse variants.
- Why: It captures the planned distinction between quick travel and a precise final approach.
- Suggested caption: "Navigation helpers use chaining for travel and separate speed presets for controlled or precise final motion."

### S19 - Absolute forward and square tests

- Priority: ESSENTIAL
- Notebook section: Odometry validation
- File: `src/autons.cpp`
- Current lines: 114-141
- Function/constant: `odom_forward_absolute_test()` and `odom_square_test()`
- Start at: `// Tests that +Y is forward`
- End after: the closing brace of `odom_square_test()`
- Make sure visible: one fixed origin, forward and reverse return, four square coordinates, and pose prints.
- Why: It shows deliberate tests for the coordinate convention and whether the origin remains fixed.
- Suggested caption: "Absolute odometry tests verify +Y motion, return to the origin, and a square path without resetting coordinates after each move."

### S20 - Cardinal-heading odometry test

- Priority: GOOD TO INCLUDE
- Notebook section: Odometry validation
- File: `src/autons.cpp`
- Current lines: 143-158
- Function/constant: `odom_turn_accuracy_test()`
- Start at: `// Tests absolute heading control`
- End after: the closing brace of `odom_turn_accuracy_test()`
- Make sure visible: 90, 180, 270, and 0 degree targets plus pose printing.
- Why: It creates a simple test for accumulated heading error at known cardinal directions.
- Suggested caption: "Cardinal-heading test prepared to measure odometry turn accuracy and accumulated heading error."

### S21 - Distance-sensor configuration and safety limits

- Priority: ESSENTIAL
- Notebook section: Distance-correction research
- File: `src/distance_correction.cpp`
- Current lines: 8-39
- Function/constant: feature toggles, direction signs, sensor objects, safety constants, and `SensorMount`
- Start at: `constexpr bool DISTANCE_ENABLE_DEBUG = true;`
- End after: `enum class SensorMount { Front, Back, Left, Right };`
- Make sure visible: TODO placeholder warning, ports 8-13, valid distance range, confidence threshold, maximum correction step, square step, and reset angle limit.
- Why: It documents both the planned six-sensor architecture and the limits that still require hardware validation.
- Suggested caption: "Distance-correction configuration centralizes placeholder ports, direction signs, validity bounds, and correction limits."

### S22 - Distance geometry helpers

- Priority: OPTIONAL
- Notebook section: Distance-correction mathematics
- File: `src/distance_correction.cpp`
- Current lines: 52-86
- Function/constant: `clamp_double()`, angle helpers, `mount_angle()`, and `reference_heading_for_axis()`
- Start at: `double clamp_double(double value, double min_value, double max_value) {`
- End after: the closing brace of `reference_heading_for_axis()`
- Make sure visible: angle wrapping and the mapping between sensor mount direction and field-axis reference headings.
- Why: It explains the coordinate math used later by pose-reset safety checks.
- Suggested caption: "Geometry helpers normalize headings and map each sensor mount to the intended field-axis reference."

### S23 - Confidence and range validation

- Priority: ESSENTIAL
- Notebook section: Sensor validity and failure handling
- File: `src/distance_correction.cpp`
- Current lines: 88-105
- Function/constant: `confidence_valid()` and `read_distance_in()`
- Start at: `bool confidence_valid(pros::Distance &sensor, std::int32_t reading_mm) {`
- End after: the closing brace of `read_distance_in()`
- Make sure visible: invalid millimeter rejection, confidence threshold, unit conversion, and accepted inch range.
- Why: It demonstrates that raw sensor values are checked before they can influence robot motion or odometry.
- Suggested caption: "Distance input is accepted only after raw-error, confidence, conversion, and physical-range validation."

### S24 - Paired averaging and difference math

- Priority: ESSENTIAL
- Notebook section: Sensor fusion and wall squaring
- File: `src/distance_correction.cpp`
- Current lines: 107-121
- Function/constant: `average_pair()` and `difference_pair()`
- Start at: `bool average_pair(`
- End after: the closing brace of `difference_pair()`
- Make sure visible: both-readings-must-succeed checks, arithmetic average, and left-minus-right difference.
- Why: It shows why paired sensors can stabilize distance estimates and reveal angular misalignment.
- Suggested caption: "Paired sensors are combined only when both readings are valid; their average estimates distance and their difference estimates skew."

### S25 - Bounded distance-correction loop

- Priority: ESSENTIAL
- Notebook section: Sensor-assisted final approach
- File: `src/distance_correction.cpp`
- Current lines: 139-173
- Function/constant: `correct_distance_with_average()`
- Start at: `bool correct_distance_with_average(`
- End after: the closing brace following `stop_drive(); return false;`
- Make sure visible: feature guard, timeout, invalid-read fail-stop, tolerance exit, bounded correction step, direction sign, odom command, and final stop.
- Why: It captures the safety structure around planned sensor-based movement correction.
- Suggested caption: "Distance correction uses validated averages, bounded motion steps, a tolerance exit, timeout, and fail-stop behavior."

### S26 - Paired-sensor squaring loop

- Priority: ESSENTIAL
- Notebook section: Wall and goal alignment
- File: `src/distance_correction.cpp`
- Current lines: 175-207
- Function/constant: `square_with_difference()`
- Start at: `bool square_with_difference(`
- End after: the closing brace following `stop_drive(); return false;`
- Make sure visible: timeout, invalid-read handling, difference tolerance, bounded degree step, turn sign, absolute target heading, and final stop.
- Why: It shows how left-right sensor disagreement is converted into a small heading correction.
- Suggested caption: "Wall-squaring logic converts paired-sensor difference into small bounded heading corrections with timeout and fail-stop protection."

### S27 - One-axis pose-reset calculation

- Priority: ESSENTIAL
- Notebook section: Correcting odometry drift with a known reference
- File: `src/distance_correction.cpp`
- Current lines: 209-248
- Function/constant: `reset_axis_from_sensor()`
- Start at: `bool reset_axis_from_sensor(char axis,`
- End after: the closing brace immediately before `}  // namespace`
- Make sure visible: feature guard, reading validation, current heading, reference heading, angle-safety check, cosine projection, robot-center offset, selected-axis update, and preserved pose components.
- Why: It provides the strongest evidence of deliberate sensor geometry and limited-state correction.
- Suggested caption: "One-axis pose reset projects a validated sensor reading onto a known reference while preserving the other coordinate and heading."

### S28 - Conversion, projection, and angular safety

- Priority: ESSENTIAL
- Notebook section: Distance-correction mathematics
- File: `src/distance_correction.cpp`
- Current lines: 251-267
- Function/constant: `dist_mm_to_in()`, `dist_project_perpendicular()`, and `dist_angle_safe_for_reset()`
- Start at: `double dist_mm_to_in(double mm) {`
- End after: the closing brace of `dist_angle_safe_for_reset()`
- Make sure visible: division by 25.4, cosine projection, wrapped angle error, and maximum-angle comparison.
- Why: It makes the physical units and angled-sensor correction mathematically reviewable.
- Suggested caption: "Pose-correction math converts millimeters to inches, projects angled readings with cosine, and rejects unsafe headings."

### S29 - Public sensor reads, averages, and differences

- Priority: GOOD TO INCLUDE
- Notebook section: Distance-sensor software interface
- File: `src/distance_correction.cpp`
- Current lines: 269-290
- Function/constant: `dist_read_*_in()`, `dist_front_average_in()`, `dist_back_average_in()`, and difference wrappers
- Start at: `bool dist_read_front_left_in(double &out_in)`
- End after: the closing brace of `dist_back_difference_in()`
- Make sure visible: all six sensor wrappers and the paired front/back average and difference calls.
- Why: It shows how internal validation and pair logic are exposed to the rest of the robot program.
- Suggested caption: "A small public interface exposes validated sensor readings and paired front/back measurements to navigation code."

### S30 - Brain debug output and grouped validity checks

- Priority: GOOD TO INCLUDE
- Notebook section: Sensor bring-up diagnostics
- File: `src/distance_correction.cpp`
- Current lines: 292-334
- Function/constant: `dist_print_debug()` and grouped validity functions
- Start at: `void dist_print_debug() {`
- End after: the closing brace of `dist_side_readings_valid()`
- Make sure visible: Brain LCD lines for sensor values, invalid-value labels, and front/back/side validity checks.
- Why: It provides a planned way to verify wiring and readings before enabling sensor-driven motion.
- Suggested caption: "Distance-sensor debug output reports each reading and grouped validity before correction routines are trusted."

### S31 - Public movement-correction wrappers

- Priority: GOOD TO INCLUDE
- Notebook section: Distance-correction API
- File: `src/distance_correction.cpp`
- Current lines: 336-368
- Function/constant: front/back distance and squaring wrappers
- Start at: `bool dist_correct_front_distance(`
- End after: the closing brace of `dist_square_to_back_surface()`
- Make sure visible: front/back direction signs, average callbacks, difference callbacks, and square-turn signs.
- Why: It shows how one tested internal algorithm can be reused for front and back configurations.
- Suggested caption: "Front and back correction wrappers reuse common bounded algorithms while keeping hardware direction signs configurable."

### S32 - Front and back one-axis pose-reset wrappers

- Priority: GOOD TO INCLUDE
- Notebook section: Odometry correction from known references
- File: `src/distance_correction.cpp`
- Current lines: 370-400
- Function/constant: front/back X and Y reset wrappers
- Start at: `bool dist_reset_y_from_front_reference(`
- End after: the closing brace of `dist_reset_x_from_back_reference()`
- Make sure visible: axis selection, sensor mount selection, offset, reference coordinate, and facing direction.
- Why: It documents how the generic reset math is specialized without duplicating the calculation.
- Suggested caption: "Front and back wrappers specialize the same one-axis pose-reset calculation for different field references."

### S33 - Side and generic pose-reset wrappers

- Priority: GOOD TO INCLUDE
- Notebook section: Side-wall odometry correction
- File: `src/distance_correction.cpp`
- Current lines: 402-441
- Function/constant: right/left X/Y resets and `dist_reset_pose_from_front_reference()`
- Start at: `bool dist_reset_x_from_right_reference(`
- End after: the closing brace of `dist_reset_pose_from_front_reference()`
- Make sure visible: right and left sensor variants, axis arguments, and delegation to the generic reset function.
- Why: It shows the planned extension from front/back scoring references to side-wall references.
- Suggested caption: "Side-sensor wrappers extend one-axis odometry correction to left and right field references."

### S34 - Distance conversion and read declarations

- Priority: GOOD TO INCLUDE
- Notebook section: Distance-correction module interface
- File: `include/distance_correction.hpp`
- Current lines: 5-29
- Function/constant: conversion, projection, raw-read, paired-read, debug, and validity declarations
- Start at: `double dist_mm_to_in(double mm);`
- End after: `bool dist_side_readings_valid();`
- Make sure visible: physical-unit helpers, every sensor position, paired values, debug output, and validity functions.
- Why: It provides a concise architectural overview of the module's input and diagnostic surface.
- Suggested caption: "Distance-correction header separates unit math, sensor reads, paired measurements, and diagnostics."

### S35 - Distance movement and squaring declarations

- Priority: GOOD TO INCLUDE
- Notebook section: Distance-correction module interface
- File: `include/distance_correction.hpp`
- Current lines: 31-45
- Function/constant: `dist_correct_*` and `dist_square_*`
- Start at: `bool dist_correct_front_distance(`
- End after: the declaration of `dist_square_to_back_surface()`
- Make sure visible: target distance, tolerance, speed limit, and timeout parameters.
- Why: It shows that correction behavior is bounded and configurable at each call site.
- Suggested caption: "Distance and squaring APIs require explicit tolerances, speed limits, and timeouts."

### S36 - Pose-reset declarations

- Priority: GOOD TO INCLUDE
- Notebook section: Distance-correction module interface
- File: `include/distance_correction.hpp`
- Current lines: 47-77
- Function/constant: front/back/left/right reset declarations and generic front reset
- Start at: `bool dist_reset_y_from_front_reference(`
- End after: `bool robot_facing_positive_axis);`
- Make sure visible: known reference coordinate, robot sensor offset, axis/facing arguments, and all sensor orientations.
- Why: It documents the complete planned one-axis reset interface and the geometric information each call needs.
- Suggested caption: "Pose-reset API makes the known field reference, sensor offset, corrected axis, and facing direction explicit."

### S37 - Front distance and odometry-combination tests

- Priority: GOOD TO INCLUDE
- Notebook section: Staged distance-sensor testing
- File: `src/autons.cpp`
- Current lines: 160-191
- Function/constant: `distance_debug_test()`, `distance_front_correct_test()`, `distance_square_test()`, and `odom_then_distance_correct_test()`
- Start at: `// Prints distance readings only.`
- End after: the closing brace of `odom_then_distance_correct_test()`
- Make sure visible: debug-first sequence, placeholder 10-inch target, front squaring, odom movement, and final distance trim.
- Why: It demonstrates a planned progression from reading sensors to combining odometry with a small final correction.
- Suggested caption: "Front-sensor tests progress from passive debugging to bounded correction and odometry-plus-distance final alignment."

### S38 - Back distance and scoring-alignment tests

- Priority: GOOD TO INCLUDE
- Notebook section: Planned scoring-side alignment
- File: `src/autons.cpp`
- Current lines: 193-223
- Function/constant: back debug, correction, squaring, and scoring-alignment tests
- Start at: `// Prints every distance sensor reading`
- End after: the closing brace of `back_scoring_alignment_test()`
- Make sure visible: placeholder warnings, reverse approach, back squaring, back distance correction, and the commented scoring action.
- Why: It shows that mechanism activation was intentionally deferred until alignment geometry is proven.
- Suggested caption: "Back-side test sequence plans reverse travel, squaring, and distance trimming before enabling a scoring mechanism."

### S39 - Side-wall X reset test

- Priority: GOOD TO INCLUDE
- Notebook section: Side-sensor pose correction
- File: `src/autons.cpp`
- Current lines: 225-233
- Function/constant: `side_distance_x_reset_test()`
- Start at: `// Tests correcting only X`
- End after: the closing brace of `side_distance_x_reset_test()`
- Make sure visible: fixed start pose, debug read, TODO placeholder coordinates, right-sensor reset, and pose output.
- Why: It provides a focused test for changing only one odometry coordinate from a side reference.
- Suggested caption: "Focused side-wall test prepares a one-axis X correction while preserving Y and heading."

### S40 - Chained movement test

- Priority: ESSENTIAL
- Notebook section: Fast travel followed by precision
- File: `src/autons.cpp`
- Current lines: 235-243
- Function/constant: `chained_motion_test()`
- Start at: `// Tests fast chained travel`
- End after: the closing brace of `chained_motion_test()`
- Make sure visible: two chained coordinate targets, final precise pose, and pose output.
- Why: It clearly illustrates the strategy of carrying speed through travel moves and settling only at the important final target.
- Suggested caption: "Chained travel maintains pace between intermediate points before a precise non-chained final pose."

### S41 - Precise reverse scoring approach

- Priority: ESSENTIAL
- Notebook section: Planned scoring approach strategy
- File: `src/autons.cpp`
- Current lines: 245-256
- Function/constant: `precise_back_approach_test()`
- Start at: `// Tests fast travel near a goal`
- End after: the closing brace of `precise_back_approach_test()`
- Make sure visible: fast initial travel, precise reverse pose, optional sensor correction comments, and pose output.
- Why: It connects travel speed, controlled reverse geometry, and optional final sensor alignment without claiming the sensors are installed.
- Suggested caption: "Planned scoring approach uses fast travel, a precise reverse pose, and optional sensor trimming after hardware validation."

### S42 - Placeholder mechanism hardware declarations

- Priority: OPTIONAL
- Notebook section: Future subsystem architecture
- File: `src/subsystems.cpp`
- Current lines: 3-20
- Function/constant: anonymous-namespace motor, pneumatic, state, and speed objects
- Start at: `namespace {`
- End after: `}  // namespace`
- Make sure visible: both TODO warnings, motor ports 14-17, piston ports A/B, state variables, and mechanism speed.
- Why: It records planned separation of mechanism hardware from behavior while clearly showing the declarations are placeholders.
- Suggested caption: "Placeholder subsystem hardware is centralized for later wiring validation; these ports are not yet competition-ready."

### S43 - Intake and pneumatic functions

- Priority: OPTIONAL
- Notebook section: Future subsystem architecture
- File: `src/subsystems.cpp`
- Current lines: 23-60
- Function/constant: intake, outtake, clamp, and descore functions
- Start at: `void intake_on() {`
- End after: the closing brace of `descore_set()`
- Make sure visible: mutually exclusive intake/outtake commands, stops, pneumatic toggles, setters, and tracked states.
- Why: It demonstrates small single-purpose subsystem functions, but it does not prove the placeholder hardware works.
- Suggested caption: "Planned intake and pneumatic functions isolate driver/autonomous commands from hardware details."

### S44 - Scoring, lift, and stop functions

- Priority: OPTIONAL
- Notebook section: Future subsystem architecture
- File: `src/subsystems.cpp`
- Current lines: 63-97
- Function/constant: scoring functions, lift functions, and `stop_all_mechanisms()`
- Start at: `void scoring_on() {`
- End after: the closing brace of `stop_all_mechanisms()`
- Make sure visible: forward/reverse/stop commands, lift up/down/stop, and the combined stop function.
- Why: It shows the intended mechanism command surface while preserving the caveat that ports and behavior are unverified.
- Suggested caption: "Basic scoring and lift command functions were scaffolded for later hardware testing, with a shared stop routine."

### S45 - Subsystem header

- Priority: OPTIONAL
- Notebook section: Future subsystem architecture
- File: `include/subsystems.hpp`
- Current lines: 5-47
- Function/constant: all mechanism declarations
- Start at: `// Mechanism declarations only.`
- End after: `void stop_all_mechanisms();`
- Make sure visible: intake, clamp, descore, scoring, lift, and stop declarations.
- Why: It shows module boundaries but must not be described as a completed claw or lift-level controller.
- Suggested caption: "Subsystem interface separates planned mechanism commands from placeholder hardware definitions."

### S46 - Historical all-positive drivetrain mapping

- Priority: GOOD TO INCLUDE
- Notebook section: Drivetrain iteration before physical direction testing
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 23-34
- Function/constant: historical `ez::Drive chassis` and `master`
- Start at: `ez::Drive chassis(`
- End after: `pros::Controller master(pros::E_CONTROLLER_MASTER);`
- Make sure visible: `{1, 4}`, `{3, 2}`, the comments saying directions were not yet tested, and one-controller setup.
- Why: It provides recoverable evidence that reversals were not guessed before the individual-motor physical test.
- Suggested caption: "Earlier diagnostic configuration kept every motor port positive until physical wheel-direction testing supplied reversal evidence."

### S47 - Historical tracking-wheel example comments

- Priority: OPTIONAL
- Notebook section: Odometry ideas investigated but not activated
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 36-42
- Function/constant: commented `horiz_tracker` and `vert_tracker` examples
- Start at: `// Uncomment the trackers you're using here!`
- End after: the commented `vert_tracker` declaration
- Make sure visible: all lines remain comments and ports/dimensions are merely template examples.
- Why: It can document an investigated odometry option while preventing a false claim that tracking wheels were installed.
- Suggested caption: "EZ-Template tracking-wheel examples remained commented while the drivetrain and sensor hardware were still being defined."

### S48 - Historical LCD, curve, and active-brake setup

- Priority: GOOD TO INCLUDE
- Notebook section: Driver-control evolution
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 50-71
- Function/constant: historical `initialize()` configuration
- Start at: `void initialize() {`
- End after: `chassis.opcontrol_curve_default_set(0.0, 0.0);`
- Make sure visible: LCD initialization, `Initialize ran`, curve buttons enabled, active brake set to 0.0, and neutral curves.
- Why: It provides a before-state for the later decision to disable curve buttons and enable active brake at 2.0.
- Suggested caption: "Earlier initialization used neutral curves and disabled active brake while execution and controller input were still being diagnosed."

### S49 - Historical autonomous selector and rumble

- Priority: GOOD TO INCLUDE
- Notebook section: Earlier integrated project state and execution feedback
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 80-106
- Function/constant: `ez::as::auton_selector.autons_add()` and historical `initialize()` ending
- Start at: `// Autonomous Selector using LLEMU`
- End after: `master.rumble(chassis.drive_imu_calibrated() ? "." : "---");`
- Make sure visible: selector test names, `chassis.initialize()`, selector initialization, LCD message, and rumble feedback.
- Why: It documents earlier integration work and the use of controller feedback, not the later unrecoverable minimal test.
- Suggested caption: "Earlier initialization registered focused autonomous tests and used Brain/controller feedback after chassis setup."

### S50 - Historical autonomous callback

- Priority: GOOD TO INCLUDE
- Notebook section: Earlier autonomous execution flow
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 142-149
- Function/constant: historical `autonomous()`
- Start at: `void autonomous() {`
- End after: `ez::as::auton_selector.selected_auton_call();`
- Make sure visible: PID target reset, IMU reset, drive-sensor reset, HOLD brake, and selected autonomous call.
- Why: It shows the intended competition callback before the current driver-only isolation. It is not active now.
- Suggested caption: "Historical autonomous callback reset drivetrain state, selected HOLD braking, and ran the chosen test routine."

### S51 - Historical EZ Brain-screen task

- Priority: OPTIONAL
- Notebook section: Earlier debugging interface
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 154-202
- Function/constant: `screen_print_tracker()`, `ez_screen_task()`, and commented task creation
- Start at: `void screen_print_tracker(`
- End after: `// pros::Task ezScreenTask(ez_screen_task);`
- Make sure visible: odometry X/Y/heading output, tracker display, competition-state check, loop delay, and disabled task construction.
- Why: It records an earlier debug-display system and why it was disabled while fixed diagnostic lines were needed.
- Suggested caption: "Historical EZ screen task displayed odometry and tracker data but was disabled to avoid overwriting fixed LCD diagnostics."

### S52 - Historical PID/autonomous controller shortcuts

- Priority: OPTIONAL
- Notebook section: Earlier EZ-Template operator tools
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 204-225
- Function/constant: `ez_template_extras()`
- Start at: `/**` above `Gives you some EZ Template extras in opcontrol`
- End after: the closing brace of `ez_template_extras()`
- Make sure visible: competition-state guard, LEFT+X PID tuner, B+DOWN autonomous shortcut, saved brake mode, and tuner iteration.
- Why: It shows previous diagnostic controls and explains why those combinations should not be confused with active driver-profile buttons.
- Suggested caption: "Earlier EZ-Template shortcuts exposed PID tuning and autonomous tests only when disconnected from competition control."

### S53 - Recoverable joystick-input diagnostic

- Priority: ESSENTIAL
- Notebook section: Controller-input diagnosis
- File: `diagnostic_backups/main.cpp.before_minimal_execution_test.bak`
- Historical lines: 246-257
- Function/constant: historical diagnostic `opcontrol()`
- Start at: `void opcontrol() {`
- End after: the final closing brace following `pros::delay(20);`
- Make sure visible: `Opcontrol started`, Left Y read, Right X read, both LCD print calls, and 20 ms delay.
- Why: It is direct recoverable evidence of the test that separated controller communication from drivetrain behavior.
- Suggested caption: "Temporary controller diagnostic displayed raw Left Y and Right X values on the Brain before motor control was restored."

### S54 - Autonomous test declaration map

- Priority: OPTIONAL
- Notebook section: Programming appendix architecture
- File: `include/autons.hpp`
- Current lines: 3-58
- Function/constant: every autonomous test declaration
- Start at: `// Sets drive, turn, swing, and odom PID constants`
- End after: `void precise_back_approach_test();`
- Make sure visible: basic PID tests, odometry tests, distance tests, chaining, and precise approach declarations.
- Why: It provides one compact table of contents for source that is preserved but currently excluded from the build.
- Suggested caption: "Autonomous test interface organizes basic motion, odometry, distance correction, chaining, and approach experiments."

## Functions requested but not present

- Autonomous selector: present only in the historical backup at S49; absent from the active `src/main.cpp`.
- Example autonomous callback: present only in the historical backup at S50; the active callback is empty.
- Manual lift controls: no controller-button lift code exists in the current source or available backup.
- Lift level/step logic: not implemented in the current source or available backup.
- Claw functions: no claw object or claw function exists in the current source or available backup.
- Temporary individual-motor and direct-drive diagnostics: not recoverable; use written physical test evidence instead.

## Recommended screenshot workflow

1. Open the exact file in VS Code and use Go to Line for the recorded range.
2. Confirm the start and end anchors before taking the screenshot.
3. Include the file tab and visible line numbers in the image.
4. Crop tightly enough for the code to remain readable, but keep the function or constant name visible.
5. Use the suggested caption and add a test-result sentence only when a real physical test supports it.
6. Recheck S01 before judging. If the Makefile exclusions are removed later, update every claim about which modules are active.
