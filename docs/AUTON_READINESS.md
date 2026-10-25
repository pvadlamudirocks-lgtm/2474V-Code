# Autonomous readiness dashboard

All new embedded sources compile in the PROS build. SOFTWARE READY means implemented,
not robot-qualified. All new motion/mechanism behaviors still need field evidence.
The initial working tree was preserved; its user-reported successful foundation is
not replaced by fabricated results here. See AUTON_SOFTWARE_GUIDE.md for controls.

| Feature | Software status | Physical blocker | Next field test |
|---|---|---|---|
| Odometry | SOFTWARE READY | reference/contact/scale after hardware changes | manual pushes and turn drift |
| nav_go_to | SOFTWARE READY | repeatability at route loads/speeds | short/long/X/Y/diagonal |
| nav_go_to_reverse | SOFTWARE READY | loaded reverse qualification | reverse and return |
| nav_turn_to | SOFTWARE READY | heading accuracy under route conditions | right/left 90 and 180 |
| nav_go_to_pose | BLOCKED for competition | reported major short stop; no proven static cause | physical exit/lead-point diagnostic only |
| Chaining | PHYSICAL VALIDATION REQUIRED | release error/speed/turn transitions | existing two-move and small-turn A/B |
| Pure Pursuit | PHYSICAL VALIDATION REQUIRED | path/lookahead/final completion | existing basic path, forward/reverse |
| Distance correction | HARDWARE ASSIGNMENT REQUIRED; application disabled | ports, mounts, walls, range/residual/heading gates | stationary single-axis correction |
| Paired distance heading | SOFTWARE READY, report only | parallel mount geometry/baseline | known wall squareness |
| Claw | PHYSICAL VALIDATION REQUIRED | polarity and loaded grip after changes | open/close/load |
| Flex | CALIBRATION REQUIRED | autonomous power/retention/pickup timing | controlled pickup/release |
| Toggle | PHYSICAL VALIDATION REQUIRED | P6/P19 current config versus P18/P19 request; duration | wiring/cooperation and loaded actuation |
| Lift/cascade manual | PHYSICAL VALIDATION REQUIRED | new cascade geometry/direction | supported up/down/release |
| Lift/cascade sensor | CALIBRATION REQUIRED | direction, coupling and home | measured rise versus P16 degrees |
| Lift target control | CALIBRATION REQUIRED | gates, limits, tolerance, timeout, gains | supervised target, then loaded |
| Lift limits | CALIBRATION REQUIRED; new gates disabled | rising sign, string-safe home, max, reference | downward home inhibition and upper bound |
| Lift hold | PHYSICAL VALIDATION REQUIRED; new finish option disabled | safe gain/cap/loaded sag/update ownership | supported STOP versus HOLD |
| Concurrent lift | PHYSICAL VALIDATION REQUIRED; no background task | same-task service and loaded safety | qualified NavSequence event, then drive overlap |
| Arm / flip | NOT USED by current routes | retained hardware decision / assignment | inventory before enabling |
| Pose Recorder | SOFTWARE READY; disabled by default | physical start reference and controller check | A capture, reset counter, explicit start chord |
| Route segment harness | SOFTWARE READY | measured stage start/target/actions | SNAPSHOT then B1_APPROACH alone |
| Logging | SOFTWARE READY | real serial capture / lifecycle stop validation | snapshot, rejected stage, interrupted run |
| PC analyzer | SOFTWARE READY | real run data | compare odom statistics to surveyed endpoints |
| Checkpoints / motion classes | SOFTWARE READY | targets, tolerances, required mechanism states | stage start rejection and final validation |
| Preflight / abort | SOFTWARE READY | hardware fault-injection confirmation | unplug, timeout, disable and mode changes |
| Mirroring | SOFTWARE READY | field asymmetry | MIRROR_MATH then explicitly chosen mirrored test |
| Block 1 | CALIBRATION REQUIRED; inert | approach/alignment/score/exit measurements | individual stages then 1-object prefix |
| Block 2 | BLOCKED pending qualified B1 | pickup/return/score geometry and proven prefix | B2 stages then 1+2 |
| Block 3 | BLOCKED pending qualified B1+B2 | same for B3 | B3 stages then 1+2+3 |
| Block 4 | BLOCKED pending qualified B1+B2+B3 | same for B4 | B4 stages then complete route |
| Skills | SOFTWARE READY entry/reuse; route unconfigured | Skills start/geometry/timing | qualified helper reuse from measured Skills start |

Exact values still needed (no values supplied):

- Left/right/Skills physical start reference and XYH; each B1-B4 approach, pickup,
  return, alignment, score and exit target; chosen drive/turn speeds; motion and
  mechanism deadlines; stage start/final tolerances; intended load and desired state.
- HOME/INTAKE, lower string-safe bound, CLEARANCE, SCORE, MAX_SAFE in a documented
  sensor reference; increasing/decreasing rise sign; feedback direction normalization;
  safe gain/cap, target tolerance, settle duration, loaded hold behavior and descent time.
- Flex autonomous power/retention; toggle actuation duration; any needed pneumatic
  dwell times based on measurement, never generic sleep guesses.
- Every distance port, right/forward offset, mount angle, usable range, known wall
  axis/coordinate/facing, incidence tolerance, residual limit and stationary RPM/time/
  deadline; pair perpendicular baseline and equality of longitudinal mount projections.

Exact hardware decisions: resolve the toggle port discrepancy without overriding the
current P6/P19 source; assign distance sensors only to verified unused ports excluding
P18/P19; optional physical bottom-switch ADI port/polarity; decide whether arm P17,
its currently unbound rotation sensor and disabled flip ADI B remain used. Existing
drive/tracker/IMU/lift/flex/claw assignments are preserved, not newly verified.
