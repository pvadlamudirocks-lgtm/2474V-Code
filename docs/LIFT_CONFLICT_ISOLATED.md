# Lift Conflict: Brain display diagnostic

## Current readiness

Prepared for supervised diagnostic use; physical behavior is unverified. STOPPING_MARGIN_DEG=2.0 is a provisional early-cutoff reserve, NOT a proven stopping-distance bound. Outward power stops at +3 or -3 startup-relative P16 degrees; +/-5 is only a software diagnostic envelope. Additional movement after zero, including beyond +/-5, is possible. Operator support and physical clearance in both directions are required. No bottom-limit sensor is configured or required.

The existing sole motor writer, 1000 mV cap, 1-second activation, released-button arming, and sensor/current/temperature/stall/write-fault protections remain. Neither kernel/device latency nor physical rest is guaranteed by software. Do not keep holding if the lift shakes or resists.

## Mouse-only workflow

1. Open C:\Users\vadla\Downloads\2474V Startup Recovery\diagnostics\lift_conflict\brain_project in a separate VS Code window.
2. Open project.pros using its PROS Project editor. Set Upload: Program Slot to an available slot 1-7. Check the Brain first; uploading replaces that slot. No slot was preselected in the project. Do NOT upload without deliberately selecting one.
3. Leave after-upload action as none. Click the PROS Build & Upload action. This builds src/main.cpp here, which includes the existing isolated diagnostic source; it does not build normal robot src files.
4. Manually open Lift Conflict on the Brain with no competition connection. Keep the lift independently supported and hands clear.
5. Release L1/L2. Read the startup-relative P16 value and valid motor readings. Tap SUPPORTED + CLEAR only after verifying clearance in both directions. This is NOT physical homing and performs no encoder reset.
6. Read the UNVERIFIED margin warning. After confirmation and released buttons, healthy conditions allow READY; no manual P16 numbers are needed.
7. READY allows one held L1 UP or L2 DOWN activation. Release, both buttons, or change directly between directions stops. Release-to-rearm, 1000 mV and 1-second cap remain. Never keep holding a shaking or stalled mechanism.
8. Read live state/stop reason at the top. Results remain after stopping; tap the bottom to switch to raw encoder/status/current-peak details. Photograph both pages and report direction, reason, each motor RPM/current/encoder delta and P16 delta.

## Envelope and ownership

First valid P16 read during initialization is the fixed reference for the whole process. Relative range is -5 to +5 degrees; button presses, clearance confirmation and mode transitions never re-zero it. The policy rejects nonfinite/nonpositive/too-large margin configuration; the selected finite 2-degree reserve is explicitly unverified. Outward command cutoff occurs at +3 or -3 degrees; opposite-direction motion still needs release/rearm. Outside +/-5 latches a fault. Invalid feedback and motor-write faults stay latched. No automatic reversal, voltage increase or restart.

Only the existing control task writes P13/P14/P15. Screen and optional terminal logger use nonblocking snapshot acquisition; no screen/serial operation occurs in the motor-owner task. Clearance is bound to the current lifecycle generation. A screen touch cannot energize a motor. Controller release and guards are checked on the next control iteration; instantaneous stopping is not guaranteed.

P13 TOP negative UP, P14 LEFT positive UP, P15 RIGHT negative UP are unchanged. No normal competition code is modified. LIFT_MOTOR_SAFETY_LOCKOUT remains true. Normal application, chassis, mechanisms and navigation are not linked into the isolated executable.

## Telemetry and interpretation

Brain: RPM signed, current mA, temperature Celsius, encoder displacement counts, P16 relative degrees, requested test voltage mV. Test voltage on a retained result is the requested activation polarity; current live zero acknowledgement is shown separately and is not proof of physical rest. Final summary readings precede the final zero command. STALE means the snapshot is old: stop the program independently.

MOTOR DISCONNECTED reports connection failure. P13/P14/P15 SUSPECT requires one stationary encoder with high peak current while both other encoders moved; it identifies an outlier, not proven wrong polarity. Unexpected P16 direction/travel is reported explicitly. Otherwise INSUFFICIENT DATA is intentional, including all-three stall. Compare encoder signs from safely assisted known upward travel against powered command signs to investigate polarity; high current alone cannot prove it.

Terminal printing is optional (TERMINAL_TELEMETRY=false by default), and never required. Logger retains a bounded 128-frame rolling RAM history; overwritten frames are counted internally. Pending end summary and Brain last-result snapshot are retained separately. Data is volatile, not saved across restart. Queue pressure still stops movement. No claim of serial reliability or physical timing qualification.

## Build and files

brain_project/project.pros has its own name and upload options, with after=none and no slot selected. It reuses the parent installed kernel headers/libraries and common.mk in place; keep it inside Startup Recovery. It is a separate application project, not a separately relocatable SDK copy. Its normal pros make writes brain_project/bin/monolith.bin. The original alternate Makefile excludes brain_project to prevent duplicate source compilation.

Host tests: tests/lift_conflict_policy_test.cpp and tests/lift_conflict_writer_test.cjs. They cover simulated safeguards, not hardware timing. Brain rendering, touch behavior, upload and physical motor behavior have not been tested.
