# Phase 1 corrections 2 and 3 — 2026-11-17

Workspace: C:\Users\vadla\Downloads\2474V Startup Recovery

Only autonomous fault suppression and Boomerang geometry validation are changed. No robot upload or powered test is authorized/performed. Correction 1 (owner-unavailable stopping) and correction 4 (slew initialization) are deferred.

## Correction 2: explicit driver-output permission

The motor boundary no longer treats native DISABLE as sufficient proof of driver control. A latched output fault permits nonzero output only when all of these hold: the driver-entry cleanup hook has granted permission, native PID is disabled, competition state is not autonomous, and competition state is not disabled.

Permission starts false. autonomous_begin revokes it before clearing the previous run's fault. Publishing ANY native output fault revokes permission before publishing the fault, including a manually dispatched test that skipped autonomous_begin and left competition flags in driver mode. Changing native mode with drive_set cannot grant permission. Neither normal autonomous completion nor an ordinary drive_stop grants permission.

The only production grant is driver_control_ready(), called immediately after auton::lifecycle_stop("opcontrol") returns. That cleanup already stops PID and disarms diagnostics through nav_cancel_sequence/drive_stop. Granting permission does not clear the recorded output fault or emergency latch. Ordinary driver commands remain exempt after this proper handoff. Fault-free command behavior, motor reversals, PTO handling and requested-speed limits are unchanged. Zero commands remain possible while faulted.

All current custom/development/selector dispatches are under autonomous(), which calls autonomous_begin. Physical Brain menu/competition flag behavior was not tested. Manually invoking autonomous with non-autonomous competition flags does NOT create a standalone enabled mode: existing autonomous_cancelled checks still cancel native movement. The new permission state only closes the fault bypass; it does not weaken those interlocks.

## Correction 3: checked geometry, same valid mathematics

The production boomerang_geometry helper receives the existing native distance_to_point and vector_off_point functions. It validates target/current pose, direction, finite dlead in its documented [0,1] range, finite nonnegative maximum carrot distance and lookahead. It validates the computed distance and distance*dlead before maximum limiting. It validates the projected carrot before the near-target substitution can conceal invalid arithmetic. Zero distance, zero maximum, zero lookahead and dlead endpoints retain their finite native calculations.

The distance formula, multiplication/clamp/direction order, projection function and lookahead/2 threshold are unchanged. A single pose snapshot is used per geometry calculation. The speed-70 target and dlead=.50 configuration are unchanged. No slew initialization, tuning, navigation wrapper, recorder, tracking geometry, motor wiring, or mechanism source is changed.

On geometry rejection boomerang_task publishes a State fault, calls drive_set(0,0), and returns before PTP calculation. This is the existing motor owner under navigation_mutex. drive_set disables native mode without reacquiring that mutex, then uses the existing final motor writer. No additional writer/task or archive member is introduced.

pid_wait compatibility was checked: the existing wait_service_tick returns false on autonomous_cancelled, and native PTP/PP waits check it after their existing delays. Publishing emergency cancellation before the zero command ensures the wait can return even if it already entered its PP branch. Cancellation is not reported as native successful completion. Device calls, RTOS scheduling and actual stop latency remain unverified physically.

## Actual stopping guarantees and unresolved limits

- Fault detected in the running motor owner: zero is commanded through drive_set before that controller function returns; native mode becomes DISABLE and subsequent faulted autonomous writes are suppressed.
- Fault detected in a setter: the existing atomic request remains deferred. A previous applied command can persist until the owner or another existing safe stop path executes. No unconditional immediate stopping is claimed if the owner has not started, is unavailable, or cannot acquire navigation_mutex.
- The software issues a zero command; it does not prove the motor API accepted it, physical rest, or braking distance.
- The newly added permission bookkeeping uses atomics only and does not redesign navigation locking. Driver takeover relies on the existing PROS lifecycle assumption that the prior autonomous task is terminated before cleanup/reuse. It is not an adversarial concurrent-autonomous/driver executor.
- Slew's current bounded initialization remains exactly unchanged, including its previously documented first-iteration difference and enabled-zero-travel rejection.
- No new independent movement deadline or nav_go_to_pose safety repair is added.

## Offline tests

1. tests/native_corrections23_test.cpp uses production permission/geometry helpers. All 80 gate states; manual-autonomous flag mismatch; fault revocation; driver handoff without clearing the fault; zero commands; invalid/NaN/Infinity geometry; finite arithmetic overflow; invalid projection before near-target override; forward/reverse and threshold edges. 100000 random valid geometries match the previous formula exactly, plus the actual (19.94,6.93,56.83), dlead=.50, max=16, lookahead=7 fixture.
2. tests/native_corrections23_harness.cjs extracts the actual current function bodies for the motor boundary, drive_set, boomerang_task, fault publication, driver permission lifecycle, cancellation and native wait-service hook. It compiles and executes those bodies against fake motors/competition APIs. Tests cover direct-drive fault bypass, manual tests with driver flags, normal driver takeover, owner-issued stop, no PTP after rejection, and wait-service cancellation. This is not a full RTOS/PROS simulation.
3. Existing motion_output_safety_test.cpp executes the unchanged slew and prior mixer tests, including 100000 finite regressions.
4. Existing recorder verification and COBS transport model tests are rerun; they do not represent physical serial testing.

Commands (portable host Zig may be replaced by a compatible host C++ compiler for the .cpp tests):

    zig c++ -std=c++17 -O1 -Iinclude tests/native_corrections23_test.cpp -o <temporary-test.exe>
    <temporary-test.exe>
    node tests/native_corrections23_harness.cjs <path-to-zig.exe>
    zig c++ -std=c++17 -O1 -g -Itests/phase1_host -Iinclude tests/motion_output_safety_test.cpp patches/EZ-Template/slew.cpp -o <temporary-old-test.exe>
    <temporary-old-test.exe>
    node tests/verify_boom_report.cjs --self-test
    node tests/boomerang_transport_test.cjs

Link verification uses the existing member-replacement process. tests/verify_phase1_link.cjs additionally verifies the new lifecycle symbols, Boomerang fault path, the final motor-boundary authorization call, and cleanup-before-permission ordering in linked opcontrol. Disassembly line endings are normalized so function-body checks cannot accidentally search later functions.

## Files in this correction

Modified: src/main.cpp (one driver-entry hook), include/EZ-Template/navigation_hooks.hpp (two declarations), include/EZ-Template/motion_output_safety.hpp (permission state/predicate and checked geometry helper), patches/EZ-Template/drive.cpp (fault gate), patches/EZ-Template/pid_tasks.cpp (permission lifecycle and geometry rejection), tests/verify_phase1_link.cjs, firmware/EZ-Template.a (rebuilt by existing recipe).

Created: tests/native_corrections23_test.cpp, tests/native_corrections23_harness.cjs, docs/PHASE1_CORRECTIONS_2_3.md. Generated bin/.d files are rebuilt. All other preexisting source/test files are checked against pre-edit hashes.

## Observed final results

All listed offline tests passed. The actual-function harness also verified that geometry rejection does not proceed to ptp_task, direct nonzero commands remain suppressed after fault, normal driver takeover preserves the recorded fault, and the actual wait_service_tick returns false.

Clean pros make passed through monolith.elf linking and monolith.bin generation with no compiler/linker warnings or errors. No code was changed to repair build errors. The preexisting AUTON ENTERED line has trailing whitespace flagged by git diff --check; it is byte-identical to the pre-edit line and was left untouched.

Archive/ELF verification passed: six existing replacement members match the newly compiled objects; 15 untouched members are byte-identical to the pre-correction archive; the final boundary reads lifecycle authorization; linked opcontrol calls lifecycle_stop before driver_control_ready. No extra native archive member or class-layout change was introduced.

Final ELF SHA-256: 8d384832c1eaf22bd8881e611092e4cfecf236fe4471d81abf3e49334533b6d6

Verified ELF addresses: private_drive_set 0x0380b238; boomerang_task 0x0380f4b0; driver_control_ready 0x0380df40; driver_output_authorized 0x0380df1c; motion_output_fault 0x0380dea8.

The pre-edit hash comparison found exactly the five intended production file changes, plus the linked-code verifier change. Slew source, Drive/slew class declarations, PID settings, autonomous commands/targets (apart from the separate opcontrol handoff line in main.cpp), nav_go_to_pose/navigation, recorder and mechanism files remain unchanged.

Further review is recommended before hardware validation, specifically the explicit driver permission handoff and the accepted deferred-stop limit. These results qualify software checks for corrections 2/3 only; they do not establish physical stopping, timing safety, or a complete resolution of Phase 1's other audit findings.
