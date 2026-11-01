# 2474V local audit snapshot — 2026-11-01

This branch captures the existing Dell project's complete current source state for independent review. Robot control code was not changed to prepare publication. Earlier documentation is historical and may describe old selections or hardware; actual source is authoritative.

## Current selection and review entry points

- `src/main.cpp`: RUN_DEV_TEST=true, ACTIVE_DEV_TEST=FULL_SYSTEM_DIAG; Custom/Route Segment overrides=false. Existing custom autonomous contents, controller mappings and hardware definitions are preserved.
- `include/full_system_diag_limits.hpp`: PASSIVE profile; physical bench verification, controlled motion enable and tracker geometry verification flags all false.
- `src/full_system_diag.cpp`: automatic passive windows, separate bench and blocked controlled-motion profiles. [Operation and timings](FULL_SYSTEM_DIAG.md).
- `src/navigation.cpp`, `src/nav_sequence.cpp`, `src/auton_route.cpp`, `src/auton_actions.cpp`: existing movement interfaces, ownership, fault handling and route helpers.
- `patches/EZ-Template/`: existing project patches. `pid_tasks.cpp` owns PID execution, bounded movement supervision and diagnostic servicing; `exit_conditions.cpp` services wait cancellation. `PID.cpp` retains the pre-existing local correction. This is a patched EZ distribution, not an assertion of stock-library behavior.
- `src/motion_exit_trace.cpp` and Makefile: exact native exit wrappers and hot/cold linkage.
- `src/subsystems.cpp`, `include/subsystems.hpp`: includes the existing unfinished mechanism work, green lift cartridge configuration and single-P19 toggle test. These were preserved, not newly commissioned by publishing.
- `src/nav_diagnostics.cpp`: tracker validity and latched application-level sensor faults. Native odometry invalid-reading behavior is not claimed repaired.

Physical root-cause investigations and qualification remain incomplete. Do not treat successful compilation, a software PASS, or this publication as proof of reliable autonomous movement or safe physical stopping. No PID tuning or further autonomous debugging was performed for publication.

## Reproduce the build

Use PROS CLI with its GNU Arm Embedded toolchain on PATH (the audit machine uses PROS CLI 3.5.6 and arm-none-eabi-g++ 14.3.1). Run from the repository root:

```sh
pros make
```

Installed project dependencies are included: PROS kernel 4.1.1, EZ-Template 3.2.2 plus the existing patches, liblvgl 8.3.8 and OkapiLib 5.0.0, their headers, static archives and linker scripts. `firmware/*.a` are required build inputs; do not confuse them with generated robot-upload binaries. The existing Makefile rebuilds selected EZ archive members from `patches/` when dependencies change, then links hot/cold images and native-exit wrappers.

Template `location` fields in `project.pros` are normalized to `.` to remove the former machine-specific template-cache paths. Versions, system-file lists, target and upload metadata are unchanged. Build uses the checked-in headers and archives. Do not run a template upgrade/reinstall as part of a read-only audit; it could replace local patches.

`bin/`, `.d/`, local terminal logs, investigation output, editor/machine folders, old diagnostic backup copies and redundant ZIP exports are excluded. The original working copies of excluded files were retained locally. No firmware upload is part of build verification.

## Applicable tests

Compile-time assertions (execute as compiler checks):

```sh
arm-none-eabi-g++ -std=gnu++20 -I include -fsyntax-only tests/full_system_diag_test.cpp
arm-none-eabi-g++ -std=gnu++20 -I include -fsyntax-only tests/movement_limits_test.cpp
arm-none-eabi-g++ -std=gnu++20 -I include -fsyntax-only tests/telemetry_stability_test.cpp
```

The diagnostic test intentionally asserts the delivered PASSIVE/disabled defaults. Deliberately commissioning another profile requires revisiting that default-selection assertion; the profile permission matrix remains independently tested.

With a host C++ compiler, compile and execute `tests/auton_math_test.cpp` using `-std=c++20 -I include`. An ARM syntax check alone does not execute those runtime assertions. With Python 3 available:

```sh
python -m unittest discover -s tools -p "test_*.py"
```

## Publication verification results

- `pros make`: exit 0; existing outputs were current (`Nothing to be done for quick`). This publication check was incremental, not a new clean rebuild.
- Compile-time assertions passed: FULL_SYSTEM_DIAG 32, movement limits 10, telemetry stability 51 (**93 total**).
- `auton_math_test.cpp`: ARM syntax compilation passed; runtime assertions were not executed because no host C++ compiler was available on PATH.
- Python analysis unit tests: not run; the WindowsApps Python alias could not execute. This is an environment limitation, not a passing test result.
- Hash comparison confirmed all existing files under `src`, `include`, `patches`, `tests` and `tools` were unchanged by packaging.
- FULL_SYSTEM_DIAG remains PASSIVE; powered enable/physical-verification flags remain false.

Hardware direction, calibration, deadlines under scheduler/API load, disable response and actual stopping require physical testing. No robot firmware upload or physical test occurred during publication.
