# Team 2474V non-field autonomous development report

## A. Files changed

Existing working files extended in this task:

- src/main.cpp: separate route test mode, recorder hook, mode/lift display, optional
  preflight and lifecycle cleanup. Existing custom test and button mappings retained.
- src/autons.cpp: permanent selector functions now call inert configured-prefix entry.
- include/subsystems.hpp and src/subsystems.cpp: preflight, calibrated preset/hold
  APIs, disabled measured soft-limit support and uncalibrated lower-limit fix.
- include/nav_sequence.hpp and src/nav_sequence.cpp: active ownership query and
  mode/sensor/active-mechanism-fault guards; no chain tuning or exit threshold changes.
- include/nav_diagnostics.hpp and src/nav_diagnostics.cpp: reusable silent sensor check.
  These two files were already untracked on arrival and are not newly invented files.

New implementation files:

- include/auton_math.hpp, include/auton_log.hpp, src/auton_log.cpp
- include/auton_route.hpp, src/auton_route.cpp, include/route_config.hpp
- include/pose_recorder.hpp, src/pose_recorder.cpp
- include/wall_correction.hpp, src/wall_correction.cpp
- include/lift_calibration.hpp
- tools/analyze_auton_runs.py, tools/test_analyze_auton_runs.py
- tests/auton_math_test.cpp

New documentation:

- docs/CURRENT_AUTON_ARCHITECTURE.md
- docs/AUTON_SOFTWARE_GUIDE.md
- docs/AUTON_START_ALIGNMENT.md
- docs/NAVIGATION_QUALIFICATION.md
- docs/DISTANCE_CORRECTION.md
- docs/MECHANISM_CONCURRENCY_PLAN.md
- docs/MECHANISM_QUALIFICATION.md
- docs/FIELD_DAY_AUTON_CHECKLIST.md
- docs/FOUR_OBJECT_AUTON_ROADMAP.md
- docs/AUTON_REPEATABILITY_PLAN.md
- docs/LOCALIZATION_FUTURE.md
- docs/AUTON_READINESS.md
- docs/NON_FIELD_AUTON_REPORT.md

Other files shown by git status were already dirty/untracked on arrival, including
navigation.cpp, navigation.hpp, the EZ archive, project.pros, earlier audit documents
and the user ZIP. They were not reset or treated as this task's changes. Build output
is in ignored bin/. No commit, push or upload was performed.

## B. Software completed

Audited current source first; retained the FRONT frame and trusted primitives.
Added pose capture/count reset/explicit configured start reset, independent segment
tests, shared B1-B4 names, readable callback/prefix structure, optional checkpoint
validation, motion classification, preconditions, explicit failure categories,
fixed-buffer CSV events, standard-library PC statistics, conservative distance math,
stationary single-axis application behind gates, lift commissioning structure and
field documentation. Permanent selector entries and Skills reuse remain compatible.

The only runnable example segment is SNAPSHOT, which does not command motion.
No route coordinates, sensor assignments, lift angles or new speed defaults were
invented. Unconfigured routes report NOT_CONFIGURED and stop.

## C. Prepared but disabled

- Pose Recorder: enabled=false; explicit start record also unconfigured.
- Route segment development dispatch: false; SNAPSHOT selected for first activation.
- Real left/right/safe/Skills routes: enabled=false, starts/stages unset.
- Advanced chaining/sequence mechanism events: existing opt-in gates remain false.
- Distance application: false; every new sensor unconfigured; paired heading report only.
- New lift limit enforcement: false; sign/home/max unset. Presets individually unset.
- Blocking lift HOLD_CURRENT_TARGET: rejected until hold_validated is true.
- Background lift task: not created or activated. Existing same-task service preserved.

The original RUN_CUSTOM_AUTON_TEST=true remains active, including its existing
395 ms toggle test. This is not a competition-ready selection.

## D. Exact physical values still needed

See the explicit value inventory in AUTON_READINESS.md: physical start references,
all route phase poses/headings, drive/turn speeds, deadlines, arrival tolerances,
mechanism dwell/power/duration, lift direction/home/clearance/score/max/string-safe
bounds, target/hold tuning and distance mount/wall/range/angle/residual/rest gates.
The values must come from measurement; the repository intentionally cannot run a
fabricated four-object route without them.

## E. Exact hardware assignments still needed

Resolve toggle wiring: repository P6/P19 is preserved; request background says
P18/P19. Reserve P18/P19 from distance assignment either way. Assign actual unused
distance ports/mounts. Assign an optional physical bottom-switch ADI port/polarity.
Decide whether arm P17 / unbound feedback and flip ADI B remain on the final robot.
No current drivetrain/tracker/IMU/mechanism port or reversal was changed.

## F. Field-day tests

FIELD_DAY_AUTON_CHECKLIST.md contains A-N in requested order, with target, actual,
result, notes and successful-repetition blanks for every listed physical test.
Start with inventory/preflight and reproducible placement, then odometry/primitives,
chains/optional PP, distance, lift, individual B1 stages and integrated prefixes.
Optimization and stress tests follow reliable four-object behavior, not precede it.

## G. Navigation still unqualified

Existing user evidence of basic movement success is retained without new claims.
New route-level deadlines, sensor-stop paths and physical error tolerances need
qualification. Chain handoff/small-turn/reverse behavior and PP paths need field
traces. Pose-Boomerang remains BLOCKED for competition / PHYSICAL DIAGNOSTIC REQUIRED.
No statically proven wrapper/units/coordinate defect explained the short stop, so
its implementation and physical tuning were left unchanged.

## H. Mechanisms still unqualified

Final cascade geometry/direction/reference/limits/target/hold/loading and concurrency;
flex power/retention and scoring timing; actual toggle wiring and route duration;
claw polarity/grip after changes; any retained arm/flip. Current configuration
availability is distinguished from connected hardware and from successful actuation.

## I. Architectural risks discovered

1. Ordinary blocking navigation does not service mechanisms. A latched lift output
   could continue while its cooperative checks are starved. Session refuses active
   lift/arm control before ordinary motion; same-task NavSequence is the gated overlap
   path. No new background writer was introduced.
2. Sequence point moves use injected PP whereas trusted three-argument point moves
   use the ordinary native command. Their qualification cannot be assumed identical.
3. PROS may terminate autonomous without C++ unwinding. Lifecycle cleanup supplements
   existing RAII, flushes interrupted logs after stop, and never dereferences the old
   Session stack. Board power loss still cannot guarantee logs or software cleanup.
4. The old distance module has conflicting placeholder devices and guessed gates.
   It stays excluded. A range cannot identify a wall versus a game object; callers
   must supply physically unambiguous geometry even after software gating passes.
5. Existing lift lower clamping used zero despite uncalibrated limits. Fixed by
   requiring the existing calibration gate. New measured full-output soft limits
   are separate from legacy feedback MIN/MAX; commission in the same frame and keep
   them consistent. Unknown limits remain disabled, not imaginary protection.
6. Hold is powered P/D with inherited unvalidated gains, not a gravity-compensated
   or motor brake HOLD guarantee. Hold needs continued service and loaded validation.
7. Event write acceptance does not guarantee later completion. Checkpoint feedback
   validators and required events must match the real mechanism. Sequence now detects
   an actively controlled lift/arm transitioning to Fault during its service loop.
8. Bounded logs can overflow, explicitly reported; labels require static lifetime.
   Odom statistics are not independent physical truth. Retain surveyed results.

## J. Build and software verification

- make quick -j4: successful embedded compile/link in the installed PROS toolchain.
  MSYS required running outside the filesystem sandbox to create its Windows kernel
  runtime object. No upload step was used.
- All new embedded feature files compile, including disabled recorder/correction/
  route/lift features. Disabled-by-default behavior remains a runtime/physical test,
  not a claim that every hardware path was exercised by compilation.
- Python analyzer: five standard-library unit tests passed (circular wrap, opposite
  headings, empty/singleton data, quoted CSV/repeated runs/error norms, unverified counts).
- Pure C++ geometry/mirroring/direction-limit test compiles with -Wall -Wextra -Werror
  using ARM g++. Runtime assertions were NOT executed: no host C++ compiler was found.
  Instructions for a native host test are in AUTON_SOFTWARE_GUIDE.md.
- git diff --check passes. Current diff and status reviewed; unrelated initial edits
  preserved. No physical validation or competition reliability claim is made.

## K. Intentionally not implemented

No fake competition/Skills route, automatic mirroring, guessed distance hardware,
automatic paired-heading application, speculative Boomerang tuning, global PID or
offset change, background mechanism ownership redesign, Monte Carlo/EKF, LemLib
migration, Brain file writer, robot upload or commit. These are either explicitly
out of scope or depend on physical evidence. Stage callback recipes must still be
filled with the team's measured travel/pickup/scoring choices; all scheduling,
validation, logging, qualification and abort infrastructure is available for them.

The requested final feature/status/blocker/next-test table is AUTON_READINESS.md.
