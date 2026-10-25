# Repeatability and stress plan

Engineering goal: increase robustness and identify sensitivity. No route can be
claimed to never fail. Establish acceptance criteria before collecting qualification
results; keep successful repetitions and failed/aborted runs, not just best runs.

| Condition | Controlled comparison | Record |
|---|---|---|
| Exact same start | repeat standardized placement | surveyed start/end, every stage result, elapsed time |
| Shifted start | deliberately vary one translation component | measured offset and resulting signed endpoint error |
| Heading variation | vary heading while holding position | measured heading variation and heading/XY endpoint change |
| Battery charge | repeat same route at different charge levels | battery state, loaded behavior, duration and motor faults |
| Consecutive runs | run repeatedly without selectively correcting results | run index, reset procedure, heat and accumulated mechanical change |
| Object placement | vary within realistic observed placement envelope | measured placement, pickup result and downstream changes |
| Full mechanism load | compare actual game load and unloaded qualification | lift sag, drive changes, retention, current/temperature |

Analyze by route, segment and motion with tools/analyze_auton_runs.py. Use different
capture filenames/config notes for intentional condition changes. Mean signed error
indicates potential systematic bias; deviations/ranges reveal spread. Circular heading
statistics avoid a false 180-degree mean for readings near 0/360. Odometry-only errors
cannot prove field accuracy: pair logs with independent endpoint measurements/video.
Record correlated errors (battery/load/contact) before changing PID. Change one cause
at a time and requalify the entire proven prefix after each material change.

| Test | Target | Actual | Result | Notes/condition | Successful repetitions |
|---|---|---|---|---|---|
| ____ | ____ | ____ | ____ | ____ | ____ |
