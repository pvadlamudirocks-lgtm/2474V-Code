# Reusable continuous paths

The reusable API contains no route. A later, explicitly requested diagnostic
autonomous now calls it with the user's test points. See
[PATH_MOTION_DIAGNOSTICS.md](PATH_MOTION_DIAGNOSTICS.md) for the one-mode selector,
buffered trace and source analysis. No red-to-yellow field route is supplied.

## Calling the API later

Include `navigation.hpp`. Supply absolute field X/Y coordinates in inches, in
travel order; the last entry is the destination. EZ adds the current odometry
position as the start, so you do not need to repeat it. Public +Y and +X retain
their existing meanings. Heading always describes the claw.

Documentation-only example using variables you must define from physical testing:

```cpp
std::vector<nav_waypoint> my_waypoints{
    {my_first_x, my_first_y},
    {my_next_x, my_next_y},
    {my_destination_x, my_destination_y},
};
nav_follow_path(my_waypoints, my_speed);
```

Alternatively, for a final public heading, use **one** call:

```cpp
nav_follow_path_pose(my_waypoints, my_final_claw_heading, my_speed);
```

To travel old-front-first for the entire path, append
`nav_path_direction::claw_reverse` as the final argument to either call.
The default `claw_forward` maps to EZ `rev`; `claw_reverse` maps to EZ `fwd`.
The wrapper intentionally keeps one direction throughout the motion, avoiding
direction-reversal stops. Speed must be 1 through 127.

Both functions block once through the existing `chassis.pid_wait()`. Empty paths,
nonfinite coordinates/headings, invalid direction values, and out-of-range speeds
return false without starting a motion. True means the motion was dispatched and
the wait returned, not that arrival succeeded. Existing exit conditions and
interference behavior still apply. Inspect `nav_pose_get()` or use
`nav_validate_pose()` with your own tolerances when checking arrival.

## Exact EZ-Template 3.2.2 capabilities inspected

Installed `include/EZ-Template/drive/drive.hpp` exposes:

- `pid_odom_pp_set(std::vector<ez::odom>)`: native multi-point following.
- `pid_odom_injected_pp_set(std::vector<ez::odom>)`: injects points between targets.
- `pid_odom_smooth_pp_set(std::vector<ez::odom>)`: injects and smooths.
- `pid_odom_set(std::vector<ez::odom>)`: selects the smoothed variant.

Each also has an explicit slew boolean overload. Each `ez::odom` contains a pose,
direction and maximum XY speed. An omitted pose heading uses `ANGLE_NOT_SET`.

These wrappers choose `pid_odom_injected_pp_set`, preserving current global slew,
spacing, lookahead and PID settings. The versioned implementation was inspected in
`set_pid/set_odom_pid.cpp`, `purepursuit_math.cpp` and `pid_tasks.cpp` under
[EZ-Template v3.2.2](https://github.com/EZ-Robotics/EZ-Template/tree/v3.2.2/src/EZ-Template/drive).
The injected API avoids the smoother's fixed 500-point arrays and does not relocate
your input shape points by smoothing. Native injection linearly subdivides the
route; native lookahead steering produces the continuous motion. This is not an
exact circular-arc or spline generator, nor a guarantee of bounded curvature.

`pp_task()` advances a target when within the configured lookahead distance,
without a per-waypoint stop/wait. It calls `boomerang_task()` when the current
target has a heading and `ptp_task()` otherwise. Therefore intermediate XY points
are non-stopping shaping targets, not mandatory precisely-hit locations. Corners
can be cut. The existing project's final-waypoint wait correction is retained.

## Final heading and continuous transition

XY-only Pure Pursuit does not specify a final heading. `nav_follow_path_pose`
puts the converted public heading on only the last target. EZ skips linearly
injecting the segment into that heading-bearing point and uses its native
Boomerang approach as that point becomes active. This is one Pure Pursuit motion
with an internal controller transition, not a separately started pose command.
The wrapper adds no stop or final turn. Controller speeds can still decrease;
continuous execution does not imply constant speed or exact heading convergence.

For a future public heading of 180 degrees, supply that value yourself to
`nav_follow_path_pose` when ready. The existing conversion requests EZ heading
0 degrees. Choose the last shaping point and clearance so Boomerang has room to
make its approach. Final position and heading depend on geometry, existing
controller settings, exits, traction and odometry; no exact heading is guaranteed.

## Geometry explanation only — DO NOT put these coordinates into autonomous code

Previously discussed coordinates are not approved route data and are deliberately
omitted here. A future peel-away location can be omitted if clearance permits or
included as a non-stopping shaping waypoint within one path. It need not be a
separate `nav_go_to_reverse` command. You choose all such points after testing.

For limited-space testing later, manually supply only your chosen short path.
Its last point becomes the stopping destination; truncating a longer path changes
the endpoint behavior. Nothing in this implementation selects or runs that test.

Physical testing is still needed for obstacle/robot clearance, corner cutting,
travel direction, speed, the final Boomerang approach and actual pose error.
No tuning, tracker configuration, coordinate conversion, existing navigation
function, PID correction, or wait/turn fix was changed.
