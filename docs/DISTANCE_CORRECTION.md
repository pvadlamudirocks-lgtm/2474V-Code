# Conservative wall correction

Legacy distance_correction.cpp remains excluded by Makefile: its placeholder devices
overlap current mechanism ports and its old motion/reset helpers are not commissioned.
Use wall_correction.hpp/cpp instead. No sensor is constructed for an unconfigured or
conflicting new entry. All four records are unconfigured and enable_application=false.
P18/P19 are reserved even though the actual toggle config currently uses P6/P19.

For each sensor measure/configure Smart Port, local right/forward offset from the
tracking center, ray heading relative to FRONT, valid minimum/maximum inches. No
mount side is inferred from a name. Check duplicate ports and all current devices.
A wall request explicitly supplies axis X/Y, wall coordinate in the same start
frame, facing sign (+/- toward wall), allowed heading error below 90 degrees, and
maximum coordinate residual. A configured request asserts one known flat unobstructed
surface; do not configure requests where game pieces or multiple walls can be seen.
Software cannot identify which physical surface produced a range reading.

For heading h, local offset (right,forward) rotates to
`offsetX = right*cos(h) + forward*sin(h)` and
`offsetY = -right*sin(h) + forward*cos(h)`.
The sensor ray is `(sin(h+mountHeading), cos(h+mountHeading))`.
Predicted range is `(wallCoordinate - currentAxis - offsetAxis) / rayAxis`.
Corrected tracking-center coordinate is `wallCoordinate - offsetAxis - range*rayAxis`.
Residual is corrected minus current. This handles all four cardinal wall normals
and angled mounts without treating body offsets as fixed world offsets.

Reject unavailable/misconfigured/conflicting sensors, PROS_ERR/no-return readings,
NaN, bad ranges, missing or invalid wall axis/facing, near-parallel rays, heading
outside the supplied gate, walls behind the ray, and excessive residual. Optional
rejection leaves XYH untouched. Required checkpoint correction must be accepted to
complete the segment. Log both acceptance and rejection.

Application requires Session ownership, an allowed/required correction checkpoint,
no active sequence or drive PID, caller-chosen stationary RPM/duration/deadline and
valid navigation sensors. Call session.stop_motion() first. Use supported EZ
odom_x_set or odom_y_set for one translation axis only; do not reset IMU, trackers
or the other axis. Setter integration compiles but remains disabled until measured
stationary correction tests confirm sign, reference and odometry behavior.

Pure helpers in auton_math.hpp are independent of PROS. paired_heading_estimate
requires parallel rays, distinct sensors, a nonzero perpendicular baseline and
equal longitudinal offsets. Both readings must pass their configured wall gates.
For signed baseline B (sensor B to the right of A's ray), squareness delta is
`atan((rangeB-rangeA)/B)`. Swapping sensors and baseline preserves the answer.
Reject unequal longitudinal mounts, nonparallel rays or unsupported geometry.
This estimate is report-only: no paired heading application or IMU reset is enabled.

Qualification: wall coordinate survey; each offset/heading/port; range validity across
materials and incidence angles; residual distribution; stationary gate selection;
single-axis before/after physical error; bad surface/occlusion rejection; paired
baseline/sign; disable while waiting. Do not tune gates to hide systematic odometry
errors. Tracking wheels/IMU remain the continuous estimator.
