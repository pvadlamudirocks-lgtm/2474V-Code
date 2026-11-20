# Phase 1 native EZ output safety — 2026-11-17

Workspace: C:\Users\vadla\Downloads\2474V Startup Recovery

Scope: native archive-member replacements only. No upload, robot execution, commit, or push.

## Sources and archive construction

New drive.cpp, set_odom_pid.cpp, and slew.cpp in patches/EZ-Template were imported from the EZ-Template v3.2.2 tag, preserving its MPL notice:
- https://github.com/EZ-Robotics/EZ-Template/blob/v3.2.2/src/EZ-Template/drive/drive.cpp
- https://github.com/EZ-Robotics/EZ-Template/blob/v3.2.2/src/EZ-Template/drive/set_pid/set_odom_pid.cpp
- https://github.com/EZ-Robotics/EZ-Template/blob/v3.2.2/src/EZ-Template/slew.cpp

Makefile compiles and replaces matching .cpp.o members with ar r, then ar s. It does not put duplicate native definitions into src or add linker wrappers. Existing PID.cpp and exit_conditions.cpp replacements remain in place and are byte-identical to the baseline archive members. An active monolithic-link map is generated at bin/phase1-link.map.

Drive and slew headers/object layouts were not changed. Original and rebuilt DWARF report Drive=5968 bytes, slew=80 bytes; all 170 Drive and 9 slew data-member offsets match the baseline DWARF; drive.cpp retains all 45 original strong definitions and slew.cpp all 12. Fifteen untouched archive members match byte-for-byte.

## Changes

- set_odom_pid.cpp: validates public vector inputs before indexing/preprocessing, raw paths before size-2/back access, finite movement/pose fields, direction, turn behavior and speed; replaces end()->target with back().target. Nested void setters cannot install a new drive mode after a fault. Raw PTP checks its PP index and finite turn reference. Invalid inputs request cancellation through the existing owner; they do not acquire navigation_lock recursively or create another motor writer.
- slew.cpp: validates input and calculated ramp state, including overflow and zero denominators. Invalid state returns zero, records a typed persistent fault, and leaves a sticky-invalid internal slope until a valid initialize. speed_max_set cannot make that invalid ramp return full speed. Disabled, valid slew still returns its requested limit. Valid lower-speed-than-minimum behavior is preserved. Iteration retains the native one-tick transition at the ramp endpoint and bounds its output to [0,max_speed].
- pid_tasks.cpp: uses the existing emergency-stop latch and owner task. The first typed fault is retained until autonomous_begin. Owner-detected faults issue drive_set(0,0) immediately, disabling native PID. Setter-detected faults request that stop on the next owner service, rather than writing motors from the setter. Shared-slew consumers check faults before output; finite checks prevent invalid float-to-int conversion. PTP/PP indices and pose state are checked. The PTP mixer retains native finite vector scaling, adds finite checks, and limits both sides to min(requested speed, valid slew limit) before integer conversion.
- drive.cpp: final integer +/-127 bound; extra requested-speed boundary in PTP/PP; active-mode fault forces zero. Existing motor reversal/PTO logic is retained. An explicit zero stop is allowed during the original 1500 ms startup power inhibit.
- motion_output_safety.hpp: hardware-independent production mixer, validators, integer boundary, first-fault latch and read-only fault getter declaration. Fault codes: None=0, Path=1, Slew=2, State=3, Output=4.

No PID gains, dlead, route targets, odometry calibration, nav_go_to_pose, mechanism sources or diagnostic frame layouts changed. No diagnostic values are overwritten with fabricated valid readings. Existing output-frame fields remain calculated/submitted outputs; rejected ticks return before emitting a normal controller frame. The typed fault is available through ez::motion_output_fault_get(); reporting integration is not added here.

## Zero-distance / heading-only limitation

Valid slew-disabled zero-distance calls retain their original controller path; a valid zero-speed request stays zero. An enabled ramp with target==current cannot define a travel direction and previously divided by zero. It now explicitly latches Slew and requests a stop, rather than falling back to full speed. This includes heading-only requests passed through a translation-based enabled slew ramp. No turn is silently substituted. Native Boomerang heading-only semantics are NOT newly qualified or repaired by this patch. A dedicated heading-only policy would require separate review.

## Offline verification

Executed on Windows with checksum-verified portable Zig 0.13.0 (temporary installation):

    zig c++ -std=c++17 -O1 -g -Itests/phase1_host -Iinclude tests/motion_output_safety_test.cpp patches/EZ-Template/slew.cpp -o <temporary-test.exe>
    <temporary-test.exe>

The actual production slew.cpp is compiled with its unchanged class declaration. The small host headers provide only the unused PROS include and sign utility. The production mixer, path validators, boundary and latch are tested directly. Hardware APIs and the complete RTOS controller task are not simulated.

PASS: 100000 native finite-formula regressions; forward/reverse/equal-min-max/disabled slew; zero-distance enabled rejection; valid zero-speed; NaN/+Infinity/-Infinity in inputs; calculated overflow/zero denominator; sticky-invalid slew after speed changes; angular output 487.584; requested caps 0/30/55/70/100/127; unequal limit/request; INT_MIN/MAX boundary; empty/singleton/invalid paths; first-fault persistence and explicit reset.

Existing tests:
- auton_math_test: runtime PASS.
- boomerang_report_writer_test: runtime PASS.
- auton_auto_clamp_test, geometry_turn_deadline_test, full_system_diag_test: compile-time assertions PASS with host compiler.
- boomerang_trace_test, telemetry_stability_test: compile-time assertions PASS with project ARM compiler.
- verify_boom_report.cjs --self-test and boomerang_transport_test.cjs: PASS (software fixtures/models, not physical serial validation).

## Build and linked-code verification

Generated bin and .d directories were removed after validating their absolute paths. Ordinary pros make then freshly compiled all active project sources and native replacements and linked bin/monolith.elf / bin/monolith.bin. Clean robot build: PASS, no compiler/linker warnings or errors.

Run:

    node tests/verify_phase1_link.cjs <ARM-toolchain-bin> [baseline-EZ-Template.a]

PASS: each replacement member occurs once; extracted bytes equal freshly compiled objects; each checked native symbol occurs once in archive and final ELF; map identifies correct member; final disassembly contains fault calls in PTP, slew iteration and private_drive_set; 15 unrelated members unchanged.

The verification must use ar x, not ar p: this Windows ar translates LF bytes on stdout, corrupting binary comparisons. Initial verification failures from ar p were resolved by binary extraction; no archive-member mismatch remains.

Tooling issues encountered and resolved: PROS CLI rejected a directly supplied -B option; ordinary make was used after cleaning. Attempting to link a static-assert-only test without main failed; it was correctly compiled as an assertion test. Initial Zig bundled-libunwind compilation emitted dllexport-on-redeclaration warnings in toolchain sources; project safety tests subsequently compiled and ran successfully. These were not robot compiler warnings. No outstanding failed tests.

## Remaining qualification

No powered or hardware tests were performed. Offline checks do not prove real sensor validity, physical braking, task scheduling/stop latency, or motor-voltage readback. No new deadline/watchdog has been added, and nav_go_to_pose wrapper safety improvements remain a separate patch. Invalid setter stop latency depends on the existing motor-owner task running. Active PID faults are sticky for the autonomous run; recovery is the existing explicit autonomous_begin lifecycle, not automatic retry. Driver behavior for valid commands remains unchanged.

This is a candidate for separate controlled hardware validation of the native-output correction, not a safety sign-off for arbitrary autonomous routes or speed-100 operation.

