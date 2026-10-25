# Field-day autonomous checklist

MEASURE -> RUN -> RECORD -> TUNE. No row below is a recorded pass. Choose and record
physical acceptance criteria before qualifying a stage. Preserve failed runs. Clear
space for the full robot footprint, overshoot and mechanism travel. Use the same
physical start reference on every controlled repetition.

Date: ____ Operator: ____ Build/config: ____ Battery/load: ____
Acceptance criteria/reference: ____ Log files/photos: ____

## A. Hardware/preflight

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Inventory ports; resolve toggle P6/P19 versus P18/P19 | ____ | ____ | ____ | ____ | ____ |
| IMU calibration and required tracker reads | ____ | ____ | ____ | ____ | ____ |
| Mechanism motor connectivity and optional/unconfigured sensor snapshot | ____ | ____ | ____ | ____ | ____ |
| Stop/disable/mode-change and unplugged-device response | ____ | ____ | ____ | ____ | ____ |

## B. Odometry qualification

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Repeatable left/right/skills alignment and start readback | ____ | ____ | ____ | ____ | ____ |
| Manual forward/right/reverse push and pure turn | ____ | ____ | ____ | ____ | ____ |
| Tracker contact, scale and offsets after mechanical changes | ____ | ____ | ____ | ____ | ____ |
| Pose Recorder A, held A, counter-only reset and start chord rejection | ____ | ____ | ____ | ____ | ____ |

## C. Primitive movement qualification

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Short and long straight | ____ | ____ | ____ | ____ | ____ |
| Reverse straight and reverse go-to | ____ | ____ | ____ | ____ | ____ |
| 90 right, 90 left and 180 turn | ____ | ____ | ____ | ____ | ____ |
| X-only, Y-only and diagonal go-to | ____ | ____ | ____ | ____ | ____ |
| Return to origin, two point and square | ____ | ____ | ____ | ____ | ____ |

## D. Chaining qualification

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Two-move CHAIN versus SETTLE comparison | ____ | ____ | ____ | ____ | ____ |
| Reverse chaining | ____ | ____ | ____ | ____ | ____ |
| Small turn drive-turn-drive A/B diagnostic | ____ | ____ | ____ | ____ | ____ |
| Final settle, required event failure, deadline and disable | ____ | ____ | ____ | ____ | ____ |

## E. Pure Pursuit qualification if used

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Existing PURE_PURSUIT_BASIC final waypoint/exit trace | ____ | ____ | ____ | ____ | ____ |
| Forward/reverse path under load and chosen speed | ____ | ____ | ____ | ____ | ____ |
| Interference, deadline and final accuracy | ____ | ____ | ____ | ____ | ____ |

## F. Distance-sensor calibration

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Assign unused ports; reject P18/P19 and active devices | ____ | ____ | ____ | ____ | ____ |
| Measure local offsets/orientation, wall coordinate and pair baseline | ____ | ____ | ____ | ____ | ____ |
| Range, angle, residual and stationary gate characterization | ____ | ____ | ____ | ____ | ____ |
| Reject unplugged/no-return/occluded/wrong-wall readings | ____ | ____ | ____ | ____ | ____ |
| Disabled-by-default application: measured single-axis before/after | ____ | ____ | ____ | ____ | ____ |
| Pair heading estimate sign and geometry (report only) | ____ | ____ | ____ | ____ | ____ |

## G. Cascade/lift commissioning

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Motor cooperation and sensor direction | ____ | ____ | ____ | ____ | ____ |
| HOME zero and lower string-safe bound | ____ | ____ | ____ | ____ | ____ |
| CLEARANCE, SCORE, MAX_SAFE measurements | ____ | ____ | ____ | ____ | ____ |
| Limits in both directions and bottom switch if installed | ____ | ____ | ____ | ____ | ____ |
| Target timing/accuracy and STOP behavior | ____ | ____ | ____ | ____ | ____ |
| Supported HOLD then loaded hold/repeatability | ____ | ____ | ____ | ____ | ____ |
| Missed update, sensor fault, mode change and concurrent travel | ____ | ____ | ____ | ____ | ____ |

## H. Block 1 segment tests

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| B1_APPROACH | ____ | ____ | ____ | ____ | ____ |
| B1_FINAL_ALIGN | ____ | ____ | ____ | ____ | ____ |
| B1_SCORE | ____ | ____ | ____ | ____ | ____ |
| B1_EXIT | ____ | ____ | ____ | ____ | ____ |
| Expected-start rejection and final error logging | ____ | ____ | ____ | ____ | ____ |

## I. Block 1 integration

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Full start-to-exit 1-object prefix | ____ | ____ | ____ | ____ | ____ |
| Repeated same-start loaded scoring | ____ | ____ | ____ | ____ | ____ |
| Confirm configured thresholds before marking prefix qualified | ____ | ____ | ____ | ____ | ____ |

## J. Block 2

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| B2_PICKUP_APPROACH / B2_PICKUP | ____ | ____ | ____ | ____ | ____ |
| B2_RETURN / B2_FINAL_ALIGN | ____ | ____ | ____ | ____ | ____ |
| B2_SCORE / B2_EXIT | ____ | ____ | ____ | ____ | ____ |
| Full unchanged B1 plus B2 prefix | ____ | ____ | ____ | ____ | ____ |

## K. Block 3

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| B3_PICKUP_APPROACH / B3_PICKUP | ____ | ____ | ____ | ____ | ____ |
| B3_RETURN / B3_FINAL_ALIGN | ____ | ____ | ____ | ____ | ____ |
| B3_SCORE / B3_EXIT | ____ | ____ | ____ | ____ | ____ |
| Full proven B1+B2 plus B3 prefix | ____ | ____ | ____ | ____ | ____ |

## L. Block 4

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| B4_PICKUP_APPROACH / B4_PICKUP | ____ | ____ | ____ | ____ | ____ |
| B4_RETURN / B4_FINAL_ALIGN | ____ | ____ | ____ | ____ | ____ |
| B4_SCORE / B4_EXIT | ____ | ____ | ____ | ____ | ____ |
| Full proven prefix plus B4 | ____ | ____ | ____ | ____ | ____ |

## M. Optimization

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| One transit speed or chain change at a time | ____ | ____ | ____ | ____ | ____ |
| Precision settling/correction comparison | ____ | ____ | ____ | ____ | ____ |
| One mechanism overlap at a time after concurrency qualification | ____ | ____ | ____ | ____ | ____ |
| Full-prefix repeatability after every change | ____ | ____ | ____ | ____ | ____ |

## N. Stress testing

| Test | Target | Actual | Result | Notes | Successful repetitions |
|---|---|---|---|---|---|
| Exact start repeated and multiple consecutive runs | ____ | ____ | ____ | ____ | ____ |
| Measured shifted start and heading variation | ____ | ____ | ____ | ____ | ____ |
| Different battery charge and full mechanism load | ____ | ____ | ____ | ____ | ____ |
| Game-object placement variation | ____ | ____ | ____ | ____ | ____ |
| Stop/timeout/failure injection and complete logs | ____ | ____ | ____ | ____ | ____ |
| Skills start and reuse qualification | ____ | ____ | ____ | ____ | ____ |


