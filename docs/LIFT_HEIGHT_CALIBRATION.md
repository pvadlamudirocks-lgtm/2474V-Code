# Downward lift sensor commissioning

The new sensor Smart Port is **UNASSIGNED**. Set
`mechanism_config::LIFT_HEIGHT_DISTANCE_PORT` in `include/subsystems.hpp` only
after the builders identify it. Zero disables construction and reads. Availability
checks reject occupied ports, including configured wall sensors, before constructing
the device. Existing P19 claw-distance code is retired; P19 is a toggle motor.

Current map audited from the working tree: drive P1/P2/P3/P4, horizontal tracker
P5, vertical tracker P7, IMU P20, lift motors P14/P15 (red cartridges, left normal,
right reversed), lift Rotation Sensor P16, arm motor P17, flex P8/P10, toggle
P6/P19. Existing mechanism checks also reserve P13. Arm rotation and old claw
distance are unbound. Wall sensors are unconfigured. No port was reassigned.

## Meaning of the reading

`lift_ground_distance_mm()` calls `lift_height_distance_sensor().get()` and returns
validated, unfiltered **beam distance**, in millimeters. Inches use exactly
`mm / 25.4`. The name describes the intended floor target, not proof of that target.
The sensor sees an object, hand, goal, or robot part below it instead of the floor.
Keep the entire beam path clear during calibration.

To report sensor-face vertical height, verify that the beam points vertically down
throughout lift travel and set `LIFT_HEIGHT_SENSOR_MOUNTED_VERTICALLY = true`.
Alternatively, for a genuinely fixed downward angle, set
`LIFT_HEIGHT_SENSOR_FIXED_ANGLE_CONFIGURED = true` and supply the measured
`LIFT_HEIGHT_SENSOR_ANGLE_FROM_VERTICAL_DEG` (0 <= angle < 90). Leave the vertical
flag false in that case. Vertical height is beam distance times cos(angle).
Both configurations default off; unknown/conflicting geometry returns NAN height.
If claw motion changes the angle, a static correction is invalid: dynamic linkage
geometry must be established first. Beam distance remains available independently.

Claw-reference height is separate and defaults to NAN. To enable it, measure the
signed **claw reference height minus sensor-face height** in inches and configure
`LIFT_HEIGHT_SENSOR_TO_CLAW_OFFSET_IN` plus its `_CONFIGURED` flag. Do not guess the
sign. A fixed offset is appropriate only if that vertical separation stays constant.
No offset is needed for sensor-face clearance itself.

## Validity and filtering

Absent/wrong device, invalid/conflicting port, PROS_ERR, no-object return 9999,
and readings outside 20..2000 mm return NAN, never zero. Optional measured beam
bounds can be enabled with `LIFT_HEIGHT_DISTANCE_BOUNDS_CONFIGURED` and the matching
MIN/MAX_MM values. They do not set lift-control limits.

The installed `include/pros/distance.hpp` documents PROS_ERR/9999.
[VEX documents the 20..2000 mm measurement range](https://api.vex.com/v5/home/blocks/sensing/distance-sensor.html).
The stated accuracy is approximately +/-15 mm below 200 mm and 5% above it;
two displayed decimals do not imply hundredth-inch accuracy. Filtering reduces
jitter but cannot correct mounting error, target obstruction, or sensor bias.

The driver loop samples every 20 ms into a fixed three-value median window.
Startup uses one sample, then the mean of two, then the median of three.
This rejects isolated spikes with roughly one sample of lag during steady motion.
An invalid sample immediately clears the window and displayed distance. Sampling
gaps over 100 ms reset the window; snapshots older than 100 ms return invalid.
There is no heap allocation or added delay. Reset occurs on entering driver control.

## Display and diagnostics

The centralized dashboard uses controller row indices 0, 1, 2 (15 columns each):
`H301 X-16.6`, `Y14.3 L842`, `D18.42in`. The third physical row is index 2.
`D18.42in b` explicitly means beam distance with unconfirmed mounting geometry;
without `b`, D is configured sensor-face vertical height, never claw-reference height.
`D---` is unavailable/invalid distance; `L---` is invalid or too-wide rotation.
Measurements retain full precision. Dashboard captures are 100 ms apart;
wireless-safe writes are staggered every 50 ms, so each row refreshes every 150 ms.
The installed PROS misc.h warns that faster wireless writes may be discarded.
Pose recorder buttons and terminal output remain; it no longer overwrites rows.
Use terminal diagnostics for full precision or values that exceed display width.

Include `lift_height.hpp`. Call `print_lift_height_calibration()` on demand from
the owning control task for a timestamp, rotation degrees, raw/filtered millimeters,
sensor-face height, claw-reference height, validity, and diagnostic confidence.
Confidence is 0..63 only above 200 mm; -1 means unavailable. No quality cutoff
is imposed. No automatic serial output
or new button mapping is installed. Service `lift_height_telemetry_update()` when
using diagnostics outside driver control to maintain the sample window. APIs share
single-task ownership; no hidden task is created.

`lift_height_calibration_sample()` returns a copyable timestamped record for future
logging and analysis of real (rotation, measured height) pairs. No degree-to-height
formula or automatic cross-check/correction is enabled. A future cross-check must
first use measured pairs in the same rotation zero frame and physical height
reference, and define a validated tolerance. The existing lift preset/limit gates
in `lift_calibration.hpp` remain unchanged and unconfigured. Rotation Sensor
degrees (accumulated centidegrees / 100, existing ratio 1:1) remain control feedback.

## Measurement worksheet

| Position | Rotation deg | Sensor-face height in |
|---|---|---|
| Home/intake | | |
| Clearance | | |
| Score low | | |
| Score high | | |
| Max safe | | |

1. Confirm mounting configuration and clear the floor below the beam.
2. Move using existing driver controls, stop, and allow the short filter to settle.
3. Record rotation and height together, noting the rotation zero reference.
4. Move away, return, and repeat several times at each desired position.
5. Use repeatability to choose actual degree targets. Only then evaluate whether
   a linear fit, interpolation, or degree targets alone suit the real cascade.

No lift directions, PID, targets, limits, safety ownership, autonomous movement,
navigation, claw/flex behavior, or toggle configuration were changed for telemetry.

## Final software audit (2026-10-15)

**SOFTWARE READY FOR HARDWARE TESTING.** No known remaining height software issue.
Port remains 0/UNASSIGNED and all mounting/offset flags remain unconfigured.
The installed PROS 4.1.1 distance.hpp and current
[PROS Distance documentation](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-distance.html)
agree on mm, PROS_ERR, and 9999. The 20..2000 mm gate is the current official VEX
device operating envelope, not an assumed PROS API return limit. It deliberately
rejects any readings beyond the documented device range even if firmware reports
them. Optional physical bounds remain disabled. Both boundary values are accepted.

The fixed filter is now directly covered by compile-time tests for startup,
spikes, real movement and invalid-data reset. Existing freshness and sampling
checks were reviewed. Raw mm/in remain independent of filtering and mounting.
Wall-sensor configuration now reciprocally rejects the configured lift-height port.
Rotation Sensor feedback and all lift control code remain unchanged.

Physical work remaining: builders identify an unused Smart Port; configure it;
confirm device detection; verify vertical mounting throughout travel (or measure
a genuinely fixed angle); test raise/lower direction, loss of target, and repeated
readings; configure a signed claw offset only if that reference is needed; record
real degree/height pairs. Physical accuracy is not yet validated.
