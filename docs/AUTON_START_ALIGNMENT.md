# Repeatable physical start reference

Coordinates are meaningful only after reproducing the same physical frame. Keep
FRONT fixed; at heading zero FRONT points +Y and robot right is +X. Use inches.
The tested setup example aligns the vertical tracking wheel exactly to a puzzle
mat seam/line. This is a placement procedure, never an odometry math offset.

| Start | Physical X reference | Physical Y reference | FRONT heading reference | Start XYH in code | Picture/file reference |
|---|---|---|---|---|---|
| Left | ____ | ____ | ____ | ____ | ____ |
| Right | ____ | ____ | ____ | ____ | ____ |
| Skills | ____ | ____ | ____ | ____ | ____ |

1. Inspect trackers/contact and calibrate IMU with robot still.
2. Place using documented robot feature and field feature, including which edge
   of the seam is used. Record wheel-center versus chassis-edge reference.
3. Align FRONT independently of XY; avoid using a loose mechanism as the reference.
4. Initialize the configured start once, using START_POSE or recorder DOWN+Y.
5. Verify displayed X/Y/H and IMU before driving. Resetting software cannot fix
   incorrect physical placement. Never re-zero at a pickup to conceal accumulated error.
6. Capture each target, name the segment, record mechanism/load and photograph it.
7. Return physically to this reference for every repetition; record deliberate
   perturbations separately. Re-measure after chassis/tracker/mechanism changes.

Operator: ____ Date: ____ Reference repeatability observation: ____
