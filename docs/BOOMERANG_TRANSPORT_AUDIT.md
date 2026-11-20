# Stationary transport test and SD export — schema 4

This supersedes the schema-3 output/lifecycle instructions in BOOMERANG_MOTION_RECORDER.md.
No powered test is requested. No robot was run or uploaded by this patch.

## What ordered=0 means
The old source tested r.ms <= previous.ms. Thus either an EQUAL timestamp or a
backward timestamp set ordered=0. This was not a row-index check and the printf
argument type was correct. Row0 is captured synchronously; the sampler starts
immediately and can record row1 in that same millisecond. Equal timestamps do not
mean reordered rows. This is a confirmed overly strict validation defect, not proof
of a capture reorder. The unavailable early rows prevent determining whether this
particular historical trace had equal or backward timestamps.
Schema4 accepts nondecreasing time, keeps consecutive row indices, and prints:
BOOM_ORDER nondecreasing, equal_timestamps, backward_timestamps,
first_backward_row, first_equal_row, backward_pair_ms.
Backward time still fails. Timestamps are not modified to hide the issue.
The offline validator uses the same nondecreasing rule.

## Serial/framing audit
Primary sources inspected:
- https://github.com/purduesigbots/pros-cli/blob/3.5.6/pros/serial/devices/vex/v5_user_device.py
- https://github.com/purduesigbots/pros/blob/4.1.1/src/system/dev/ser_driver.c
- https://github.com/purduesigbots/pros/blob/4.1.1/src/common/cobs.c
- https://github.com/purduesigbots/pros/blob/4.1.1/src/system/dev/ser_daemon.c
- https://github.com/purduesigbots/pros/blob/4.1.1/src/system/dev/usd_driver.c

CLI splits messages on NUL, then calls cobs.decode; that exception generates the
reported warning BEFORE payload text decoding. Unicode is not an explanation for
that warning. Long ASCII payloads are legal COBS. Packet boundaries are low-level
write boundaries, not CSV newline boundaries. Newlib buffering can split/coalesce
application writes. The recorder writes at most192 bytes then flushes; pacing is
unchanged and is NOT claimed to repair a framing defect.

The upstream4.1.1 serial driver prefixes stream ID, encodes COBS, appends NUL and
uses a write mutex. Normal concurrent stdout/stderr calls should not interleave
bytes INSIDE an encoded frame. Application lines can still interleave between
writes. Native completion, startup, shutdown, and the PROS banner are other writers;
the selected custom route does not start another periodic terminal telemetry task.
No project serctl/fdctl code disables COBS.

The driver queues through a2047-byte stream buffer. ser_output_write converts the
send count to bool; ser_write_r returns requested length for any nonzero acceptance.
The downstream flush also has an explicit dropped-data warning when the VEX write
returns fewer bytes. Therefore successful application writes are not a host-delivery
guarantee. These are concrete possible loss paths, NOT evidence that either happened
during this test. Cable/USB/bridge loss, truncation, raw/COBS mismatch, or another
lower-level defect remain hypotheses without the raw frame hex and full capture.
No kernel, CLI, global buffering, or framing settings were changed.

## Synthetic-only build (OFF by default)
Edit ONLY include/boomerang_trace_options.hpp:
    BOOM_TRACE_SYNTHETIC_ONLY = true;
    BOOM_TRACE_EXPORT_SD = true; // optional; false tests terminal alone

Build with pros make. No upload is performed by this task. If the user later chooses
to run this communication-only build, start it while competition-disabled. initialize()
starts the fixture in a low-priority task and returns before IMU/lift/mechanism initialization. autonomous(),
opcontrol(), disabled() and competition_initialize() return immediately in this build.
It never invokes the selected Boomerang or any mechanism command. No device readings
are used to fabricate the fixture; all values are explicitly synthetic.
71 rows exercise the SAME renderer, long rows, NaN values, header, checksum/footer,
and intentional equal timestamps at rows0/1. Expected order: nondecreasing=1,
equal_timestamps=1, backward_timestamps=0. ID71001; source synthetic=1.
The fixture does not start if the program is enabled or the controller mode is active.
Restarting the program repeats the stationary fixture; no movement is chained.

Set SYNTHETIC_ONLY back to false to restore the unchanged normal callbacks.
Do NOT run the powered route just to test reporting. Existing target/speed and custom
selection remain19.94,6.93,56.83 /100 and RUN_CUSTOM_AUTON_TEST=true in the normal build.

## Independent outputs and retained data
Normal explicit post-movement report() uses EXPORT_SD (default false) plus terminal.
boomerang_motion_trace_export(true,false): terminal only.
boomerang_motion_trace_export(false,true): SD only.
boomerang_motion_trace_export(true,true): both (SD first).
These are callable ONLY against a ready frozen capture and disabled drive controller.
No I/O occurs in the motor callback or sampler. A separate low-priority task owns I/O.
There is no navigation/recorder mutex held across I/O, and mode cleanup never joins it.
Cancellation requests an atomic abort; the worker stops between writes and closes SD.
A syscall already in progress cannot be forcibly interrupted safely. PROS/VEX driver
latency, shared serial locks and power loss remain outside application timing guarantees.
Existing motor stop calls occur before recorder cancellation. Control tasks do not
acquire an I/O lock added by this patch. Do not manually delete the output worker.

The explicit caller waits up to30s after movement, then requests abort. Competition
callback termination does not intentionally delete the independent owner. A failure
or timeout never starts/retries movement. BOOM_OUTPUT reports terminal and SD
attempted/success/error/row count independently. output_status() exposes the same
results; EBUSY means the output job is still running.
Data is ALWAYS retained after output, including one-destination failures.
boomerang_motion_trace_discard() explicitly releases it for a later capture.
Do not discard a real trace until it has been retrieved. Restart/power loss loses RAM.
Single diagnostic API caller only; no concurrent replay/rearm/export callers.

## SD path, checks, retrieval
Card is optional. pros::usd::is_installed()==1 is required; absent card reports ENODEV.
Supported standard fopen/fwrite/fflush/fclose operations use:
    /usd/boom-<trial_id>-<capture_start_uptime_ms>.txt
On the removable card this is the ROOT file:
    boom-<trial_id>-<capture_start_uptime_ms>.txt
For synthetic ID71001 the prefix is boom-71001-.
Files open in append-binary mode, never truncate an existing trace. Repeated filenames
across reboots append another complete report. Validator handles every BEGIN block
independently and flags incomplete old reports instead of silently ignoring them.
Open, full-write count, flush and close return values are checked. SD failure does
not prevent a separate terminal attempt, and terminal failure cannot erase SD/RAM.

Important kernel limitation:4.1.1 calls vexFileSync without returning its status and
usd_close_r returns0 after vexFileClose. Checking fflush/fclose is necessary but
cannot prove physical durability. Ejected-card, full-card, power-loss and hardware
latency tests have NOT been run. File integrity must be verified after retrieval.

After the stationary job finishes and the program is stopped, remove the card and
copy the entire file to the PC. Do not remove a card during writing.
From this project folder:
    node tests/verify_boom_report.cjs "C:\path\to\boom-71001-....txt"
Validate the terminal capture separately:
    pros terminal --output "$env:USERPROFILE\Downloads\boom-synthetic-terminal.txt"
Start terminal BEFORE starting the stationary program; wait for BOOM_TRACE_END and
BOOM_OUTPUT. Preserve all decoder warnings. Do not use --raw/--hex as an unexamined fix.
Keep both files for checksum/row comparison. Matching complete SD but corrupted
terminal narrows the fault to after formatting; it does not identify the exact
framing/USB layer. A failed SD checksum needs investigation before powered testing.

## Software validation
Node tests execute source-derived CSV/header arity, long-row71/450-row fixtures,
NaN fields, equal timestamps, backward/missing/duplicate/corrupt/incomplete reports,
and appended multi-report files. Generated71-row files are read back and validated.
COBS offline model tests boundary lengths, ASCII/UTF-8 payloads and missing-byte
decoder rejection. This is not a test of the installed kernel/CLI or USB.
All available C++ tests are compile/static-assert checked; host runtime C++ and robot
runtime are not available. The synthetic Brain code has not been executed.
Fresh pros make compilation/link and monolith.bin generation accompany delivery.
