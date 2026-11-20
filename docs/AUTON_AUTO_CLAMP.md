# Autonomous object-region clamp (disabled pending rebuild)

No current route arms this feature. It detects an object in a calibrated region;
it cannot identify a Pin or confirm physical capture. A successful trigger records
one successful semantic close command, not physical closure.

## Required configuration in include/subsystems.hpp
- CLAW_DISTANCE_PORT: verified unused Smart Port (currently 0); the existing
  active_smart_ports_unique static assertion covers it.
- AUTON_AUTO_CLAMP_MIN_MM / MAX_MM: measured object-region bounds (currently 0).
- AUTON_AUTO_CLAMP_READINGS: at least 2 distinct timed samples (currently 0).
- AUTON_AUTO_CLAMP_SAMPLE_MS: measured supported sensor sampling interval (0).
- AUTON_AUTO_CLAMP_FRESHNESS_MS: maximum sample gap/read duration (0).
- AUTON_AUTO_CLAMP_TIMEOUT_MS: finite pickup window, greater than freshness (0).
- AUTON_AUTO_CLAMP_RANGE_CALIBRATED: false until range/mounting are qualified.
- AUTON_AUTO_CLAMP_POLARITY_VERIFIED: false until semantic open/close are verified.
- ENABLE_AUTON_AUTO_CLAMP: false until supervised qualification.
Keep ENABLE_CLAW_DISTANCE_AUTO_ARM false: this is independent of the old arm
detection path, and auto-clamp rejects operation if the old path is enabled.

Current software maps claw_open()/claw_set(true) to CLAW_OPEN_OUTPUT=false (OFF)
and claw_close()/claw_set(false) to ON. Existing route comments contradict this
physical interpretation. Do not invert polarity based on comments. Verify it on
the rebuilt claw before setting the new verification gate.
Commanded open is not position feedback.

## Future integration sketch (comments only; NOT a runnable route)
// Verify physical semantic open/close first.
// Prepare claw using claw_open(); abort the pickup if it fails.
// Create the existing NavSequence scope before arming (it performs cleanup).
// if (!auton_auto_clamp_arm()) stop/recover without attempting the pickup.
// Approach using a qualified existing NavSequence movement with its existing
// timeout/cancellation safeguards. Its wait service calls mechanisms_update(),
// which services auto-clamp. No coordinates or new movement commands here.
// Check auton_auto_clamp_triggered(), auton_auto_clamp_state(), and
// auton_auto_clamp_reason() after the motion has stopped.
// If not triggered, stop/recover; do not assume capture or automatically retry.
// auton_auto_clamp_disarm() before leaving the pickup region.

A NavSequence finish/abort, autonomous cleanup, disabled, and opcontrol cleanup
all use stop_all_mechanisms(), which revokes arming without opening the claw.
Explicit claw commands also revoke arming. arm/update belong to the same
autonomous caller. Nonblocking mutex acquisition serializes those calls;
lifecycle disarm atomically revokes arming. Mode/cancellation is checked again
after sensor reads and immediately before the existing claw_close API.

Not every legacy blocking navigation wrapper calls mechanisms_update().
Do not arm around an unserviced legacy movement. No global wait hook was replaced.
Outside a serviced sequence, the caller must update regularly; a late update
times out before commanding closure. This is cooperative, not an independent
watchdog. Cancellation cannot undo a pneumatic command already issued.

The detector requires the lift disabled, no lift fault, no arm movement/hold,
no homing, and no active flex or Toggle output (invalid voltage reads reject).
It does not start/stop those mechanisms or interfere with drivetrain output.
No console IO or delay is added by the update itself. Existing claw API failure
logging/cancellation behavior remains intact.

Invalid/out-of-range samples reset the count; missing sensor faults an armed
pickup. Timeout disarms without closure. A close write is attempted once;
failure latches Fault until program restart. Disarm preserves terminal results
for inspection. Explicit rearming clears a prior success/timeout result.
Freshness measures software read timing; PROS provides no per-reading hardware
timestamp, so a stuck-but-plausible sensor value cannot be ruled out in software.

## Qualification
1. With feature disabled, verify wiring, port ownership, pneumatic polarity, and
   mount stability. Keep hands out of the claw and support mechanisms.
2. Collect sensor readings with/without the intended object at pickup angles;
   choose bounds, sampling, debounce, freshness and finite timeout from evidence.
3. Confirm semantic open/close and safe stationary mechanism conditions.
4. Enable only in a supervised, explicitly armed stationary autonomous test.
   Verify absent sensor, out-of-range objects, noise, timeout, cancellation,
   disable, driver transition, and one-shot success. Do not test on a person.
5. Only then integrate a separately reviewed serviced approach. Physical testing
   is still required; no hardware qualification is claimed by the build/tests.