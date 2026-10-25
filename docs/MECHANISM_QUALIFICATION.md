# Mechanism qualification

No physical results are supplied by this software task. User reports competition
toggle success; verify current wiring because repo is P6/P19 versus request P18/P19.
Do not change assignments from recollection. Use semantic APIs in route callbacks.

| Mechanism | Current software | Required physical qualification |
|---|---|---|
| Claw | claw_open/close; ADI A; output false opens per existing comment | actual polarity, travel, grip under load, repeatability |
| Flex | P8/P10, opposite reversals; intake/outtake/stop/pickup helpers | intake direction, power, retention, pickup consistency; choose nonzero autonomous power |
| Toggle | normal/reverse/stop; B/X; P19/P6 | resolve wiring, confirm cooperation and direction, chosen run duration, load |
| Lift/cascade | P14/P15, P16 centidegree rotation, manual/full/target APIs | commissioning sequence below |
| Arm | P17; feedback unbound, manual disabled | decide if used; assign feedback only if present; verify direction/limits |
| Claw flip | ADI B placeholder, disabled | decide if used; verify assignment and polarity before enabling |

Lift commissioning order (record target, actual, result, notes, successful repetitions
for every step):

1. Support the mechanism, verify P14/P15 cooperate; up is existing P14 positive /
   P15 negative electrical voltage. Do not duplicate sign logic in routes.
2. Measure P16 direction as lift rises. Current feedback controller assumes increasing
   normalized position for positive/up voltage. Verify/normalize sensor reversal before
   setting LIFT_FEEDBACK_VERIFIED. Soft limits independently accept measured +/- sign.
3. Define physically repeatable HOME/INTAKE and zero reference. Confirm sensor coupling
   and external gearing after cascade changes. Zero is a procedure, not inferred startup.
4. Measure lower string-safe bound: DOWN must not unwind slack at home. Configure
   lift_calibration::limits in the same referenced frame; set enable_limits only after
   validating sign/home/max. Unknown/invalid enabled limits block output. Full-voltage
   commands require a known home when these limits are enabled. Also commission the
   existing feedback MIN/MAX limits; keep both consistent. Existing uncalibrated lower
   bound no longer silently clamps at zero.
5. Measure CLEARANCE, SCORE, MAX_SAFE; configure each preset flag/degrees independently
   in lift_calibration.hpp. Existing LOW/MID/HIGH remain legacy gated presets; avoid
   mixing the two sets in a route. No new angles were assigned.
6. Commission a measured target, tolerance, settle time and deadline under supervision.
   Feedback gains/caps in subsystems.hpp are inherited UNVALIDATED values, not new tuning.
7. Test STOP versus HOLD_CURRENT_TARGET while supported. Current brake is BRAKE;
   sensor P/D hold is not motor brake HOLD. Blocking wrapper defaults to zero output.
   Nonblocking target naturally transitions to sensor hold. P-only hold can sag.
8. Validate loaded hold/current/temperature and failure handling. Set hold_validated
   only after safe behavior and update ownership are demonstrated.
9. Repeat every calibrated height and descent with full load and multiple battery levels.
10. Qualify same-task concurrent travel using MECHANISM_CONCURRENCY_PLAN.md last.

Optional real bottom switch: assign unused ADI port, verify pressed polarity and
descent timeout before enabling switch-based return. Current command switch ADI C
is distinct from the unbound physical bottom limit. Do not confuse these functions.

| Step/test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| ____ | ____ | ____ | ____ | ____ | ____ |
