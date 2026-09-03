# Engineering Notebook: August 1 through September 3, 2026

## Evidence and status note

This document records the development visible in the current project and the test results I reported during drivetrain debugging. The folder is not currently a Git repository, so there is no commit hash that can prove a complete day-by-day history. The line numbers and screenshots in this notebook correspond to the workspace as it existed on September 3, 2026. If the code changes, I should relocate each screenshot using the function or constant anchors rather than trusting the old line number.

The current Makefile deliberately compiles only `src/main.cpp`. The autonomous, navigation, distance-correction, and mechanism files are preserved, but they are excluded from the current robot build while I focus on reliable driver control. Therefore, I must describe those modules as source-code development or test scaffolding, not as active competition behavior.

[SCREENSHOT 01 HERE]

File: `Makefile`

Lines: 35-37

Shows: the temporary exclusion of autonomous, navigation, distance-correction, and subsystem source files.

Suggested caption: "Temporary drivetrain-only build scope used to isolate driver control from unfinished autonomous, sensor, and mechanism systems."

## Drivetrain bring-up and physical direction testing

The first problem was more basic than driver feel: the robot program appeared to run, but the drivetrain did not respond. I reduced the project to a minimal PROS execution test. The Brain displayed `PROGRAM IS RUNNING`, `OPCONTROL STARTED`, and `OPCONTROL RUNNING`, and the master controller rumbled. That physical test established that the program was building, uploading, and executing and that PROS was entering both `initialize()` and `opcontrol()`.

After program execution was proven, I displayed Left Y and Right X on the Brain. The values changed when the sticks moved, which established that the master controller was connected and joystick input was reaching PROS. I then tested one drive motor at a time at low power. I judged direction by whether each wheel would propel the robot forward or backward, not by clockwise or counterclockwise rotation because the two sides are mirrored.

The physical results I recorded were:

- L1, port 1: forward at positive power.
- L2, port 4: backward at positive power.
- R1, port 3: forward at positive power.
- R2, port 2: backward at positive power.

That evidence produced the final EZ-Template motor groups `{1, -4}` and `{3, -2}`. The current chassis also records 2.75-inch wheels, blue 600 RPM cartridges, and an external ratio that represents 450 RPM effective wheel speed. IMU port 7 is retained from the existing configuration but still needs physical confirmation before autonomous testing.

[SCREENSHOT 02 HERE]

File: `src/main.cpp`

Lines: 3-11

Shows: `ez::Drive chassis(` through `600.0 / 450.0);`, including the verified motor reversals and drivetrain dimensions.

Suggested caption: "Final EZ-Template drivetrain configuration based on individual physical motor-direction tests."

The project uses one master controller. No partner controller is active.

[SCREENSHOT 03 HERE]

File: `src/main.cpp`

Lines: 13-19

Shows: the master controller declaration and the three current driver-feel constants.

Suggested caption: "One-controller configuration and centralized driver-control tuning constants."

## September 3 driver feedback: turn sensitivity

### What the driver reported

The driver's request was approximately: "When I move the turn joystick, I want the robot to turn harder and faster without needing to move the stick as far."

I interpreted this as a joystick-response problem, not a request for a higher physical motor top speed and not an autonomous turn-PID problem. The robot already has a fixed physical maximum. The requested change was to reach a larger portion of that available turning command earlier in the stick's travel.

Another programmer who normally uses manual C++ drivetrain code suggested multiplying the turn input by a number greater than one. His idea was conceptually:

`amplified turn = raw turn input * multiplier`

For example, a raw Right X value of 40 remains 40 with ordinary linear mapping. With a 1.30 multiplier it becomes 52. This does not make the motors exceed their maximum speed. It changes how quickly the command approaches that maximum:

less joystick movement -> larger turning command -> high turning power is reached sooner.

### EZ-Template curve investigation

I first investigated EZ-Template's supported joystick curves. The project exposes `opcontrol_curve_default_set()` and separate forward/turn curve scales. However, this version's supported nonnegative curve values only add low-speed control. Larger values make the response softer, while `0.0` is linear. The built-in adjustment logic clamps the scale at zero, so it cannot create a response stronger than linear.

The current project explicitly sets both EZ curves to `0.0`. The earlier SOFT/NORMAL/AGGRESSIVE profile experiment is retained only as ordinary comments and does not compile or run. Because the driver requested a response above linear for turning only, I used the custom multiplier approach while keeping EZ-Template as the drivetrain owner.

### Current implementation

The active constant is:

`TURN_MULTIPLIER = 1.30`

This value is coded and compiled, but it is not yet proven by driver testing and is not finalized. Values 1.10, 1.20, 1.40, and 1.50 are possible future comparison values only. I have no evidence that those alternatives have been physically tested.

A small deadzone of 5 removes controller noise near zero. A clamp then guarantees that the amplified turn and final side commands remain in the legal PROS motor-command range of -127 through +127.

[SCREENSHOT 04 HERE]

File: `src/main.cpp`

Lines: 66-80

Shows: `apply_driver_deadzone()` and `clamp_driver_output()`.

Suggested caption: "Driver-control safety helpers reject neutral-stick noise and constrain commands to the PROS motor range."

### Arcade mixing

The actual implementation uses split arcade inputs:

- Forward is Left Y.
- Turn is Right X.
- Only the turn value is multiplied by 1.30.
- Left output is forward plus amplified turn.
- Right output is forward minus amplified turn.

When the driver requests a right turn, the turn term increases one side and decreases the other. Amplifying that term increases the difference between the left and right commands, so the robot should rotate harder for the same Right X movement. Straight forward and backward sensitivity remain unchanged.

The final left and right commands are clamped independently. The function then passes those commands to `chassis.opcontrol_joystick_threshold_iterate()`. This preserves EZ-Template's driver active-brake behavior even though the standard `opcontrol_arcade_standard(ez::SPLIT)` mixer had to be bypassed.

[SCREENSHOT 05 HERE]

File: `src/main.cpp`

Lines: 82-94

Shows: `driver_arcade_control()`, the turn multiplication, arcade mixing, output clamps, and handoff to EZ-Template.

Suggested caption: "Custom split-arcade turn amplification added after the driver requested stronger rotation with less joystick travel."

### Tradeoffs and next test

A larger multiplier should create faster reaction and require less stick movement. It can also become twitchy, reduce fine alignment control, and make scoring approaches harder. For that reason, 1.30 is a starting value rather than a claimed optimum.

My test process will be:

1. Use an open floor area and begin at low speed.
2. Test approximately 25%, 50%, and full Right X input.
3. Ask the driver whether 1.30 is quicker without becoming unpredictable.
4. Change only `TURN_MULTIPLIER`, then repeat the same maneuvers.
5. Try 1.10 or 1.20 if 1.30 is twitchy; try 1.40 or 1.50 only if 1.30 still feels too weak.
6. Verify precise goal alignment, not only fast free-space turns.

## September 3 driver feedback: forceful stopping

### What the driver reported

The driver's request was approximately: "When I let go of the joystick, I want the robot to stop instead of continuing to roll."

I understood the original coast behavior as this sequence:

driver releases joystick -> motor command becomes zero -> drivetrain momentum keeps the robot rolling.

The requested behavior was:

driver releases joystick -> drivetrain resists continued motion -> robot stops sooner.

### Comparing stopping modes

`COAST` removes driving power without strongly resisting wheel motion. It allows the most rolling.

`BRAKE` electrically resists continued motor rotation after the command reaches zero. It normally stops more firmly than coast without continually trying to hold an exact position.

`HOLD` actively tries to maintain a position. It can be useful in some mechanisms or autonomous situations, but it can make a driver drivetrain abrupt and jerky.

EZ-Template active brake is driver-control logic that remembers the drivetrain position when the sticks return to neutral and uses feedback to oppose continued movement. It is distinct from selecting a PROS motor brake mode.

### Current implementation

The current code combines `MOTOR_BRAKE_BRAKE` with EZ-Template active brake. I chose BRAKE instead of HOLD because the driver wants decisive stopping without permanently locking the drivetrain at every neutral-stick moment.

The active-brake starting value is:

`DRIVER_ACTIVE_BRAKE_KP = 2.0`

The code calls `chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP)` during initialization. The custom arcade mixer sends its final values through EZ-Template's `opcontrol_joystick_threshold_iterate()`, which is the part that applies active braking when both side commands return to zero.

The 2.0 value is coded and compiled, but it is not a universal answer and has not yet been physically tuned. Values 1.5, 2.5, and 3.0 are possible comparison values only.

[SCREENSHOT 06 HERE]

File: `src/main.cpp`

Lines: 96-109

Shows: neutral EZ curves, `opcontrol_drive_activebrake_set()`, chassis initialization, and the ready message.

Suggested caption: "Driver active brake initialized at a starting kP of 2.0 while EZ joystick curves remain neutral."

[SCREENSHOT 07 HERE]

File: `src/main.cpp`

Lines: 125-152

Shows: `opcontrol()`, `MOTOR_BRAKE_BRAKE`, Left Y and Right X reads, the custom driver mixer call, LCD diagnostics, and loop delay.

Suggested caption: "Active driver-control loop using stronger PROS braking, split-arcade inputs, and live joystick diagnostics."

### Why stopping matters strategically

The robot may need to travel quickly between game objects and scoring locations. If it coasts too far, alignment becomes inconsistent, the driver must release early, and the robot can overshoot a desired scoring position. A stronger stop may let the driver travel aggressively and transition into alignment or scoring sooner.

Maximum braking is not automatically best. Excessive active brake can make the robot jerk, increase mechanical shock, make small corrections uncomfortable, and reduce driver confidence. I need to treat this as driver tuning:

1. Drive forward at moderate speed.
2. Release Left Y suddenly.
3. Observe or measure the continued travel distance.
4. Reduce the value if stopping is violent or oscillatory.
5. Increase it gradually if the robot still rolls too far.
6. Repeat backward and turning releases as well as straight-line stops.

## Three controls that must not be confused

Turn sensitivity determines how the driver's Right X input maps to the turning portion of the drivetrain command. Its current tuning constant is `TURN_MULTIPLIER` in `src/main.cpp`.

Active brake determines what the drivetrain does when the driver's mixed command returns to neutral. Its current tuning constant is `DRIVER_ACTIVE_BRAKE_KP` in `src/main.cpp`.

Autonomous turn PID determines how autonomous code reaches a heading target. Its constants are in `default_constants()` in `src/autons.cpp`. Changing the driver multiplier does not tune autonomous PID, and changing autonomous turn PID does not change the driver's Right X mapping.

[SCREENSHOT 08 HERE]

File: `src/autons.cpp`

Lines: 12-24

Shows: autonomous speed constants and drive, heading, turn, swing, and odometry PID constants.

Suggested caption: "Autonomous motion constants kept separate from the September 3 driver-control response settings."

[SCREENSHOT 09 HERE]

File: `src/autons.cpp`

Lines: 26-46

Shows: exit conditions, chain constants, slew constants, and odometry tuning values.

Suggested caption: "Autonomous completion, chaining, and slew settings, which were not changed during driver-feel tuning."

## Driver-programmer collaboration

I am responsible for implementing and maintaining the code, but the driver is the person who feels the robot's response under match-like conditions. I should not declare a sensitivity or brake value "best" from code inspection alone. The engineering loop is:

change one parameter -> let the driver test -> record feedback -> adjust one parameter -> repeat.

Driver feedback can directly influence the turn multiplier, future joystick curve choices, active-brake strength, and deadzone. I should preserve controlled comparisons by changing one variable at a time and repeating the same driving task.

## Navigation and autonomous architecture developed for later testing

The navigation layer establishes an absolute coordinate convention and wraps EZ-Template odometry commands. The header defines +Y as forward, +X as right, and headings from the original starting pose. The implementation contains helpers for forward, reverse, pose, and absolute-heading commands.

These files are currently excluded from the active build and must not be described as physically validated autonomous behavior.

[SCREENSHOT 10 HERE]

File: `include/navigation.hpp`

Lines: 5-35

Shows: absolute-coordinate conventions and declarations for start, forward, reverse, pose, and turn helpers.

Suggested caption: "Navigation interface establishes one absolute field-coordinate convention for planned autonomous motion."

[SCREENSHOT 11 HERE]

File: `src/navigation.cpp`

Lines: 23-64

Shows: `nav_set_start()`, `nav_go_to()`, forward/reverse pose helpers, and `nav_turn_to()`.

Suggested caption: "Navigation implementation wraps EZ-Template odometry commands without resetting the origin after each move."

The autonomous test file contains routines for absolute forward motion, a square, and cardinal-heading turns. These are test routines, not a finalized match autonomous.

[SCREENSHOT 12 HERE]

File: `src/autons.cpp`

Lines: 114-158

Shows: `odom_forward_absolute_test()`, `odom_square_test()`, and `odom_turn_accuracy_test()`.

Suggested caption: "Planned odometry validation routines for absolute positions and headings before building a match route."

## Distance-correction research and safety logic

The distance-correction module includes placeholder smart ports 8 through 13, conversion from millimeters to inches, validity and confidence checks, paired averaging, paired difference for squaring, bounded movement correction, and one-axis pose-reset math. The source explicitly labels the sensor ports as placeholders. None of this should be claimed as installed or tuned hardware until the real sensors are wired and tested.

[SCREENSHOT 13 HERE]

File: `src/distance_correction.cpp`

Lines: 8-39

Shows: feature toggles, direction signs, placeholder distance-sensor objects, validity limits, correction limits, and mount enum.

Suggested caption: "Distance-correction configuration separates planned sensors, safety bounds, and direction signs before hardware validation."

[SCREENSHOT 14 HERE]

File: `src/distance_correction.cpp`

Lines: 88-121

Shows: confidence validation, reading validation, pair averaging, and left-right difference calculation.

Suggested caption: "Sensor readings are rejected when invalid and combined only after both members of a pair pass validation."

The correction loops use timeouts and stop the drive when readings fail. Distance corrections are limited to small steps, while squaring uses a small heading adjustment based on the left-right distance difference.

[SCREENSHOT 15 HERE]

File: `src/distance_correction.cpp`

Lines: 139-207

Shows: bounded distance correction and paired-sensor squaring loops with validation, timeout, and stop behavior.

Suggested caption: "Planned distance and squaring loops use bounded corrections, timeouts, and fail-stop behavior."

Pose reset projects an angled sensor reading onto the reference normal with cosine, checks angular safety, and changes only the selected odometry axis while preserving the other axis and heading.

[SCREENSHOT 16 HERE]

File: `src/distance_correction.cpp`

Lines: 209-267

Shows: angle-safe one-axis pose-reset calculation, millimeter conversion, cosine projection, and angular guard.

Suggested caption: "One-axis pose correction preserves the rest of the odometry state and rejects unsafe sensor angles."

## September 3 implementation status

| Item | Current evidence-based status | Notes |
|---|---|---|
| Turn sensitivity | CODED AND COMPILED; DRIVER TESTING REQUIRED | `TURN_MULTIPLIER = 1.30`; not finalized. |
| Joystick curve | CODED AND COMPILED AS NEUTRAL | Active scales are `0.0, 0.0`; optional profiles are comments only. |
| Active brake | CODED AND COMPILED; PHYSICAL TUNING REQUIRED | Driver active-brake kP starts at `2.0`. |
| PROS brake mode | CODED AND COMPILED; PHYSICAL TEST REQUIRED | Driver control uses `MOTOR_BRAKE_BRAKE`, not COAST or HOLD. |
| Controller input | PHYSICALLY TESTED | Left Y and Right X were observed changing on the Brain. |
| Motor direction | PHYSICALLY TESTED | Results support `{1, -4}` and `{3, -2}`. |
| Driver evaluation | NOT YET DOCUMENTED | The driver still needs to compare response and stopping feel. |
| Autonomous/navigation | SOURCE PRESENT; EXCLUDED FROM CURRENT BUILD | Not ready to claim as active or physically validated. |
| Distance correction | SOURCE PRESENT WITH PLACEHOLDERS; EXCLUDED | Ports and geometry require real hardware verification. |
| Mechanisms | PLACEHOLDER SOURCE; EXCLUDED | Ports, names, reversals, and cartridges are explicitly marked TODO. |

## Claims I must personally verify before showing judges

- Confirm the IMU is physically connected to smart port 7.
- Confirm the 2.75-inch wheel diameter and gearing that produces 450 RPM effective output.
- Record a driver test of `TURN_MULTIPLIER = 1.30` before calling it successful or optimized.
- Measure or consistently observe stopping behavior at active-brake values such as 1.5, 2.0, 2.5, and 3.0 before comparing them.
- Confirm that BRAKE plus active brake does not create oscillation, wheel chatter, or excessive mechanical shock.
- Do not claim the autonomous, odometry, distance, or mechanism modules are in the current robot build while the Makefile exclusion remains.
- Replace and verify all placeholder distance-sensor and mechanism ports before describing them as installed hardware.
- Record actual field-test results for navigation, pose reset, chaining, and scoring approach routines before calling them reliable.

## Historical diagnostic evidence

The available `diagnostic_backups/main.cpp.before_minimal_execution_test.bak` file contains an earlier LCD initialization message, IMU-dependent controller rumble, an autonomous-selector draft, and a Left Y/Right X display loop. It does not contain the later minimal `PROGRAM IS RUNNING` test, the A/B/X/Y individual-motor test, or the temporary direct four-motor joystick test.

The full historical screenshot instructions are in `Programming_Code_Walkthrough_Aug1_Sep3.md`. Where code is unavailable, I should use a notebook diagram or a written test-result table rather than fabricating a screenshot.
