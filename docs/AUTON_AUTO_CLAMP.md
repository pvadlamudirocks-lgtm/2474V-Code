# Claw distance sensing and automatic close

## Shipped state
CLAW_DISTANCE_PORT=0. Driver and autonomous automatic closing remain false;
all range/polarity calibration gates remain false, distances and arming timeouts
remain zero. No route arms auto-clamp. V7 lift and navigation behavior are unchanged.
ENABLE_CLAW_DISTANCE_AUTO_ARM is retained only for source compatibility and has
no actuator effect; the legacy arm_move_score detection action was removed.

## Passive measurement
Assign CLAW_DISTANCE_PORT only after verifying actual wiring and unused port
ownership (P19 belongs to Toggle). Keep both auto flags false. The shared accessor
never constructs port zero. ENABLE_CLAW_DISTANCE_DIAGNOSTIC=true prints from the
existing driver mechanisms_print_debug path about every 100 ms, before that
function's separate 500-ms mechanism-log throttle. It does not print inside
navigation wait callbacks. No new task, screen writer, actuator call or sensor
reset is used by passive sampling. Ordinary robot startup/manual controls are
still active; this is not an isolated passive firmware mode. In particular,
existing startup writes claw OFF and initializes other mechanisms.

Build/upload manually, start supported at established lift home, enter driver
control, keep controls released, and open PROS terminal. Expect CLAW_DISTANCE
port=0 status=UNCONFIGURED before port assignment. After assignment, VALID lines
contain millimeters and confidence or NA (confidence is unavailable <=200 mm).
NO_OBJECT, DISCONNECTED, READ_ERROR, INVALID and LOW_CONFIDENCE are distinct.
No numeric distance is printed for invalid/no-object results. errno is included.
Brain Devices -> sensor -> millimeters remains a no-program alternative.

## Configuration (include/subsystems.hpp)
Driver: ENABLE_DRIVER_AUTO_CLAMP, CLAW_DISTANCE_CALIBRATED,
CLAW_AUTO_CLAMP_POLARITY_VERIFIED, CLAW_DETECT_MIN_MM/MAX_MM,
CLAW_DETECT_CONSISTENT_READINGS (3), CLAW_DISTANCE_SAMPLE_MS (20),
CLAW_DISTANCE_FRESHNESS_MS (100), CLAW_AUTO_CLAMP_TIMEOUT_MS (0).
Shared: CLAW_DETECT_REMOVAL_READINGS (3), CLAW_DETECT_MIN_CONFIDENCE (20),
CLAW_DISTANCE_PORT, CLAW_OPEN_OUTPUT (false; preserve until physical verification).
Diagnostic: ENABLE_CLAW_DISTANCE_DIAGNOSTIC, CLAW_DISTANCE_DIAGNOSTIC_MS (100).
Autonomous: ENABLE_AUTON_AUTO_CLAMP, AUTON_AUTO_CLAMP_RANGE_CALIBRATED,
AUTON_AUTO_CLAMP_POLARITY_VERIFIED, AUTON_AUTO_CLAMP_MIN_MM/MAX_MM,
AUTON_AUTO_CLAMP_READINGS/SAMPLE_MS/FRESHNESS_MS/TIMEOUT_MS (all zero).

Ranges and timeouts must be measured/chosen before enabling. Config requires
positive min, max>=min and <9999, >=2 entry/removal readings, positive sample
interval, freshness>=sample interval, finite timeout>freshness and <2^31 ms,
and enough timeout for the configured entry readings. Choose enough time for
both removal and entry. Values in tests are synthetic fixtures, not calibration.

## Behavior and ownership
One detector and one sampler serve the currently selected mode. Driver arming
starts on an enabled driver session or explicit claw command; autonomous requires
an explicit successful auton_auto_clamp_arm() in an enabled autonomous session.
Every new arm/manual override requires multiple consecutive valid OUT-OF-RANGE
readings, then consecutive valid IN-RANGE readings. No-object (9999), missing
sensor, read errors, low confidence or stale samples never count as removal;
they revoke the removal qualification. Close-range confidence is not required.
A no-object background alone therefore cannot rearm: a measurable out-of-range
background/target is required. This conservative behavior must be tested at the
actual mount before enabling.

One successful event calls existing claw_close() once, with existing polarity,
return-home, disabled and autonomous-cancellation checks. An already commanded
closed claw produces ALREADY_COMMANDED_CLOSED without an output call. Results
mean command state only, never confirmed physical closure/capture. Failure
latches detector Fault until restart; no automatic retries. Driver timeout does
not automatically restart; an explicit claw command or mode change begins a new
window, again requiring removal. After a successful close, manual opening starts
a new removal-qualified window. Autonomous requires explicit rearming.

Explicit claw commands revoke automatic intent before attempting a nonblocking
claw gate. Busy requests return false rather than queue; driver R1 already reports
write failure. Automatic close reenters the same gate only on the same task.
Cleanup invalidates an atomic generation without waiting for the gate. Mode,
cancellation, generation and deadline are checked after sampling and before close.
A mode switch after the last software check cannot undo an already-issued ADI
command. There is no claim of instantaneous pneumatic stopping.

mechanisms_update -> update_object_detection -> claw_distance::update is the
active integration. Existing NavSequence wait service calls mechanisms_update;
not every legacy movement does. Do not arm around an unserviced movement.
stop_all_mechanisms and mechanisms_initialize revoke arming. No route is changed.
No new interlock with unrelated mechanisms is claimed; callers must choose a
physically safe pickup window. Existing claw_set rejects closing during return-home.

## Validation and physical qualification
Host tests cover classification, removal/entry boundaries, initial object,
manual override, one-shot, disconnected/no-object/stale readings, mode changes,
timeout, cancellation, output failure and wraparound. They simulate APIs; they
do not establish device latency, sensor accuracy, pressure, actual polarity,
closure timing or object capture. PROS has no hardware timestamp here: freshness
measures software sampling age, not proof that plausible sensor values changed.
Use stationary supervised pneumatic testing before any moving approach. Verify
clear beam, mounting, OFF/open and ON/close, release/manual override, and behavior
with the object removed. Keep hands clear and secure/support the mechanism.
