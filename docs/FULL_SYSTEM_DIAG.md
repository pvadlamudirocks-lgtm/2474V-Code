# Automatic FULL_SYSTEM_DIAG profiles

This guide supersedes the earlier A/X/Y confirmation workflow. Nothing has been uploaded or physically tested by Codex.

## Selection

Edit `include/full_system_diag_limits.hpp`:

```cpp
inline constexpr Profile profile=Profile::PASSIVE;
inline constexpr bool motor_bench_physically_verified=false;
inline constexpr bool controlled_motion_enabled=false;
inline constexpr bool tracker_geometry_physically_verified=false;
```

`src/main.cpp` already selects `DevTest::FULL_SYSTEM_DIAG`, with RUN_DEV_TEST=true, RUN_CUSTOM_AUTON_TEST=false and RUN_ROUTE_SEGMENT_TEST=false. Those user settings were preserved. Rebuild after changing the compile-time profile. The currently built profile is PASSIVE.

## PASSIVE (default)

1. Enable autonomous with the robot stationary on the floor. Stage A automatically captures **2 seconds** of hardware/sensor health.
2. Wait while the buffered health report prints. A preparation baseline is recorded automatically. The Brain LCD and terminal display a **5-second countdown**. Keep still during it.
3. At **GO**, manually rotate the robot clockwise approximately **90 degrees** during the **8-second measurement window**, minimizing translation. Finish within the window and hold still. No X/A/Y input is needed. Start/end samples bracket the window; sensors and pose are never reset.
4. The manual report and summary print, then the diagnostic exits. C/D/E are skipped. There is no transition into a powered profile.

Acquisition/countdown time is nominally **15 seconds**, plus sensor-call, stop-handshake, scheduling and terminal-printing time. The Brain LCD is the timing cue, not a stopwatch started at autonomous enable. Use a supervised autonomous test mode that allows enough time for the whole session; a 15-second competition period can truncate it.

PASSIVE does not install the powered hook, call a navigation movement, or issue a nonzero motor command. It requests zero through the existing owner at entry/cleanup. This is a software-path guarantee, not a claim that the robot cannot roll or that firmware/hardware faults are impossible.

## MOTOR_BENCH (separate build selection)

Select `Profile::MOTOR_BENCH` only with the robot already securely supported, all drive wheels clear of floor/supports/cables/hands. There is **no button approval** after autonomous starts.

- Stage A: **2 seconds**, stationary. Manual stage B is skipped; do not rotate the stand.
- Stage C: four individual motors in the actual configured left/right vector order, followed by the left group and right group. Each action has a **5-second countdown**, a **2,000 mV pulse capped at 250 ms**, then stop acknowledgement and a rest check. The existing caller fallback is 350 ms; the owner independently ends the pulse at 250 ms.
- Each pre/post rest check requires 100 ms of quiet feedback and allows at most **2 seconds**. Stop acknowledgement is bounded by the existing **100 ms** limit. Reports print after each stopped action. Nominal countdown/pulse total is **31.5 seconds** for six actions, plus health, settling and reporting.
- Failure/cancel stops the session; later actions do not run. The profile ends after bench testing and never enters floor tests.

Record each motor's observed physical direction, mounting/coupling behavior and actual stop response separately. A sensor-response PASS is not physical direction verification. A zero-commanded coupled partner motor's existing brake mode can resist an individual low-voltage pulse; failure does not prove a broken motor. The software never sets the physical-verification flag automatically.

## CONTROLLED_MOTION (blocked by default)

Selecting the profile alone does not enable motion. Both `motor_bench_physically_verified` and `controlled_motion_enabled` must be deliberately set true in a future build after physical bench/cancel/disable verification. They remain false in the delivered code.

When explicitly authorized/configured, this profile runs health for 2 seconds, skips manual/stand stages, then on a clear floor separately counts down **5 seconds** before CW15, CCW15 and FRONT3-inch movements at speed30. Each uses the existing **1,500 ms** owner deadline, stop acknowledgement and pre/post rest checks. A failed segment ends the session. Nominal countdown plus maximum movement time is **19.5 seconds**, plus health/rest/printing.

The optional final +Y3-inch point test stays skipped unless `tracker_geometry_physically_verified` is independently set true as well. If enabled, it has another 5-second countdown, explicitly resets the local pose to (0,0,0), verifies readback, and uses the same 1,500 ms movement deadline. No geometry is certified from an approximate manual rotation.

## Cancellation, evidence and status

Competition disable/mode changes retain the existing owner/lifecycle stop path. **B** is an optional cancel when a controller is connected; a disconnected controller does not block passive progression. No A, X or Y is read. Keep an independent competition-disable control accessible for powered testing. Deadlines run in the existing owner independently of the blocking navigation caller; they remain scheduling/API-dependent, not hard real-time guarantees. Actual stop latency requires hardware measurement. Forced competition-task deletion can truncate reports.

Passive samples are collected at nominal **50 ms** intervals (20 Hz before read overhead), powered samples at nominal **20 ms** intervals. Fixed buffers hold 200 complete rows. Each row groups sensor and motor data in one acquisition cycle with a start timestamp and **read_span_ms**. Hardware reads are sequential, not simultaneous hardware-triggered samples; the span exposes acquisition skew. No high-frequency terminal output occurs during capture. Start/end snapshots, raw tracker deltas, converted distances, IMU heading/unwrapped rotation, calculated pose displacement, validity, motor feedback, PID/command values and exits are reported after each stage/action.

PASS means the named numerical screening criterion was met, not full physical qualification. Validity failures are FAIL; untested stages are SKIPPED. Approximate manual motion cannot independently verify a 90-degree scale or tracker geometry, so those physical conclusions remain INCONCLUSIVE. Missing/invalid samples cannot become a PASS.

## Capture terminal output

Build with `pros make`. Upload only when the team deliberately chooses to do so; no automatic upload is performed. Open the PROS serial terminal before enabling autonomous:

```powershell
pros terminal --output output/full-system-auto/robot-session.txt
```

If needed, supply the actual detected Brain port, e.g. `pros terminal COM7 --output output/full-system-auto/robot-session.txt` with COM7 replaced. This redirects output to the file; follow the Brain LCD timing cues. Save the full START-to-END report and record physical observations alongside it. No END marker means the session was incomplete, not passed.
