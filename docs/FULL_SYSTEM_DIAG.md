# FULL_SYSTEM_DIAG — passive sensor reporting

FULL_SYSTEM_DIAG remains selectable through ACTIVE_DEV_TEST in src/main.cpp.
It now performs passive reporting only. Powered profile selectors, button approvals,
motor-bench commands, controlled-motion commands, and their control-task hooks were removed.
It cannot transition into powered stages and does not configure ordinary autonomous movement.

Keep the robot stationary for the 2-second health capture. After a 5-second countdown,
physically rotate it clockwise approximately 90 degrees during the 8-second manual
capture window, then hold still. Samples are buffered at 50 ms intervals; results print
through the PROS Brain Terminal, with a compact final summary before optional history.
B or competition disable cancels. No button approval is required.

Sensor validity, raw tracking readings, IMU rotation, pose, motor encoders, RPM,
voltage readback and current are still sampled; the compact report prioritizes sensor results. Approximate physical motion does not
prove geometry or navigation accuracy; powered stages report SKIPPED.

## Compact terminal output

The completed manual-window summary prints before any trace history. It includes
IMU rotation delta, horizontal/vertical raw centidegree and distance changes,
start/end odometry, window validity and actual screening/qualification results.
A sensor screen PASS is not physical geometry qualification. Interrupted or
invalid windows cannot pass; powered checks remain SKIPPED.

`include/full_system_diag_limits.hpp`: `detailed_trace=false` is the default.
Setting it true prints at most `trace_max_rows=8` evenly selected manual samples,
including first/last, after the summary. Rows are paced by at least
`trace_print_interval_ms=100`. Sensor acquisition remains 50 ms; the health,
countdown and manual windows remain 2, 5 and 8 seconds respectively.
Report lines are bounded to 511 characters, paced 20 ms, and output stops on a
reported write/format error. There is no retry loop or forced flush.

Transport limitation: this does not guarantee delivery or a hard write deadline
when the host/USB connection is broken. Official PROS 4.1.1 ser_driver.c uses
unbounded waits on blocking writes; its ser_ctl nonblocking flag changes a local
copy rather than persisting the flag. No kernel/serial driver workaround or global
stdio change was introduced. Excessive output can create backpressure; it does
not establish the cause of missing aa55 headers or invalid Windows handles.
Source: https://github.com/purduesigbots/pros/blob/4.1.1/src/system/dev/ser_driver.c

Open one PROS Brain Terminal connection before starting the passive diagnostic.
Record from FULL_SYSTEM_DIAG START through DIAGNOSTIC SUMMARY END. Keep still
for health, then rotate clockwise approximately 90 degrees during the manual
window. If transport fails, capture the host error and whether SUMMARY END was
visible; do not interpret missing output as a sensor failure or passing test.
