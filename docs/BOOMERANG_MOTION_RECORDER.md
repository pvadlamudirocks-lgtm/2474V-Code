> Schema 4 update: output lifecycle, ordering, stationary replay and SD instructions are superseded by BOOMERANG_TRANSPORT_AUDIT.md.

# Boomerang recorder audit and schema 3 retrieval (2026-11-16)

## Selected test
Only custom_auton_test arms ID 10001, calls nav_go_to_pose(19.94,6.93,56.83,100),
then requests the full report. RUN_CUSTOM_AUTON_TEST=true, RUN_DEV_TEST=false,
RUN_ROUTE_SEGMENT_TEST=false. The claw call remains commented. Autonomous entry/
exit still perform existing mechanism-stop cleanup and nav_set_start(0,0,0).
No recorder task runs unless explicitly armed. A failed arm does NOT cancel motion.
No mechanism command, navigation setting, or route selection was changed here.

## Save the next supervised trial
In PowerShell, from this project directory, BEFORE starting the trial:
    pros terminal --output "$env:USERPROFILE\Downloads\2474V-boom-100-trial01.txt"

Use a NEW filename for each trial. This is the installed CLI's documented --output
option, not --logfile (which logs CLI messages). Start the already selected autonomous
using the existing enabled-autonomous procedure. Do not run another movement while
the report prints. Wait for BOOM_TRACE_END before ending terminal capture.
After closing the terminal, validate the saved file:
    node tests/verify_boom_report.cjs "$env:USERPROFILE\Downloads\2474V-boom-100-trial01.txt"

Send the WHOLE file, including startup and BOOM_TRACE_BEGIN through BOOM_TRACE_END.
Do not copy only visible scrollback. The validator checks the first complete report;
use one trial per file. Keep decoding warnings too. Missing END, rows, or checksum
failure means the received trace is incomplete/corrupted; do not infer missing
movement phases were never captured. This procedure has NOT been hardware validated.
Do not enable --raw or --hex for the normal CSV test: those change CLI decoding.
The installed help also offers --backend solo if diagnosing a bridge problem later;
no evidence yet identifies the backend as the cause.

## Data and exact source map
All line numbers refer to the audited source; subsequent edits can shift them.

| Capability | Source | Status and interpretation |
|---|---|---|
| Trial, target/speed, start/final pose, direction, capture uptime | src/boomerang_motion_trace.cpp:265,368,389 | Recorded. ID is caller label; ID + uptime distinguishes trials within a boot. Not globally unique across reboot. Build timestamp/schema also emitted. |
| Drive L/R, heading, turn, odom angular, Boomerang PID | src/boomerang_motion_trace.cpp:65 | Public readbacks before dispatch. Added previously omitted drive L/R constants. |
| Active XY/angular PID and slew | src/boomerang_motion_trace.cpp:98 | First successfully observed controller tick; per-row angular gains. Not guaranteed first physical tick. |
| Lookahead/dlead/distance/bias/turn behavior and configured slew | src/boomerang_motion_trace.cpp:65 | Live pre-dispatch readbacks, now consistently labeled. Not post-dispatch introspection. |
| Actual per-tick slew limit, raw/scaled PID, bias, target plane, flipped, native active target | src/boomerang_motion_trace.cpp:98,439 | Direct existing PathControlFrame fields; newly exported scaled/bias/target values. Native target heading is NOT public coordinates. |
| Public X/Y/H, target errors, wrapped heading error, controller mode | src/boomerang_motion_trace.cpp:98,439 | Cached controller pose during motion; read-only pose before/after. Control age exported; sequential device sampling is not simultaneous. |
| Requested L/R vs individual measured RPM/mV/mA | src/boomerang_motion_trace.cpp:121,239,439 | Requested native integer command arguments; never derived from RPM. Pre/post commands are NaN, since stop write is not observed by this callback. Motor readbacks use PROS reversal conventions. |
| IMU heading/rotation/status/calibration/errno, P5/P7 installation/config/readings/errno, battery mV | src/boomerang_motion_trace.cpp:121 | Existing supported getters, no resets. Valid-mask and NaN sentinel handling. Per-motor individual getter errno unavailable; validity is sentinel/finite based. |
| Peak current/RPM, command steps/difference/reversals, RPM delta reversals | src/boomerang_motion_trace.cpp:337 | Sampled metrics only. Added steering-difference delta reversals and rows reaching requested speed. Not proof of physical oscillation or actual voltage saturation. |
| Dispatch, wait return, stop request/return, RPM-rest window | src/boomerang_motion_trace.cpp:288,368 | Wait return != success. Native completed flag != independently timestamped final exit. Rest: all four abs RPM<3 for >=100ms, sample gaps<=90ms. Not chassis rest/slip/acceleration. |
| Native XY/angular final exits | src/boomerang_motion_trace.cpp:298; src/motion_exit_trace.cpp:255 | Direct published native result. Pose trace repaired to use it when completed; cancellation with no completed result retains unobserved/Running semantics. |
| Capacity/lifecycle/cancellation | include/boomerang_trace_data.hpp:4; src/boomerang_motion_trace.cpp:153,181,304 | Fixed 450 rows, no overwrite; append overflow counted. Cancellation/end flags survive try-lock contention. |
| Sampling gaps/loss/invalid values | src/boomerang_motion_trace.cpp:153,368 | Nominal30ms, actual dt/read time, missed-slot ESTIMATE, separate sampler/hook skips, invalid/stale rows. Skips != exact lost controller ticks. |
| Report integrity | include/boomerang_report_writer.hpp; src/boomerang_motion_trace.cpp:439 | Numbered rows, phase counts, recorded/emitted counts, FNV-1a CSV checksum, ASCII/bounds checks, local write failure. No host receipt acknowledgement. |
| Future stage events | src/boomerang_motion_trace.cpp:313 | Existing bounded16events and overflow count; no actions. |

## Confirmed output defects and repairs
Previous output emitted the whole report without pacing, composed rows from several
printf calls, lacked row numbers/emitted counts and did not check output errors.
This made loss hard to locate. It does NOT prove the observed decoder warnings
were caused by throughput. The code iterated rows 0..count-1, with an explicit
pre-dispatch row, so late copied rows alone do not prove initial capture loss.

Schema3 assembles ASCII lines in fixed4096-byte storage and transmits at most192
bytes before fflush and a10ms yield, ONLY after stopping. Row fragments are assembled
before transmission. Formatting overflow/non-ASCII/output errors fail reporting;
frozen data is retained after local failure. The best-effort failure line may itself
be lost. No global serial lock or control-task lock is introduced. Mode changes
abort further report chunks. Successful fwrite/fflush is not delivery confirmation.
The checksum covers exact CSV data rows plus LF, not configuration text. Validator
normalizes CRLF. It detects missing/reordered rows, field-count errors and changed
data. Capture-health/truncation must still be reviewed independently of checksum PASS.

Reporting is synchronous when explicitly requested and can take several seconds;
maximum-size output may exceed the remaining enabled-autonomous period. A PROS task
termination can interrupt it. This patch does not extend autonomous time or bypass
mode transitions. No automatic reporting, motion retries, or additional test added.

## Exit discrepancy
pose_exit_trace_begin initialized both states to RUNNING. Its observation depended
on --wrap intercepting PID exit calls. Same-object calls in the monolithic library
can bypass that interception. The active patched native wait already publishes its
actual evaluated XY/angular results after completion (exit_conditions.cpp:188).
drive_stop does not clear this result. pose_exit_trace_finish now reads that result,
without calling PID calculations/exit checks again or changing their timers.
No independent exact native-exit timestamp is claimed; wait-return is the observable
boundary. An interrupted native wait leaves completed=false.

## Remaining uncertainty
The original full raw file is unavailable, so serial transport loss, CLI decoding,
copy truncation and task interruption cannot be separated retrospectively. No
malformed Boomerang printf argument types were found by strict compiler checking.
Native completion and startup/cleanup also print; mode transitions may interleave
their output with reporting. No additional periodic terminal writer is selected by
the custom route; Brain display writes are separate. Existing native wait output is
unchanged and can occur before the explicit stop; recorder output never does.

LIFT STARTUP ZERO FAILED is a separate startup/sensor result (main.cpp:260), not a
recorder diagnosis. No lift code or calibration changed. The diagnostic itself has
no mechanism actuation; program initialization and stop cleanup remain unchanged.

No claim is made about the cause of violent motion, slip, acceleration, braking
torque, or physical stopping. Capture timeout12s and tail1s limit RECORDING, not
movement. The recorder is NOT an independent motor watchdog. Existing controller
stop/cancellation remains unchanged.

All recorder mutex takes are take(0). Hardware reads occur in the low-priority
sampler (plus initial snapshot), not the motor callback. Reporting holds no recorder
mutex. Unexpected sampler termination can strand the recorder until restart, but
cannot make the controller wait for that mutex. Hardware getter latency and sampling
overhead still need physical verification.

## Validation
- tests/verify_boom_report.cjs --self-test: executed in Node, source-format/header
  arity plus450-row/NaN fixtures and missing/duplicate/corrupted/truncated reports.
- Seven C++ tests: ARM syntax compilation, including static_assert lifecycle,
  capacity, cancellation/timeout and math checks. Not robot runtime execution.
- boomerang_report_writer_test.cpp has host-runtime assertions for line assembly,
  checksum, failed sink, oversize formatting and invalid bytes. Compiled only:
  no native host C++ compiler/emulator was available.
- Strict recorder -Wformat=2 -Werror=format passes. An unrelated existing DRIVE_DIAG
  format warning in motion_exit_trace.cpp (%u vs unsigned long through_stop_ms)
  remains outside this Boomerang change; normal PROS build succeeds.
- Fresh PROS compilation/link and monolith.bin generation must accompany delivery.
No physical tests, upload, commit or push performed.
