# Future localization boundary

Current architecture: tracking-wheel odometry plus IMU provides nav_pose_get;
stationary, gated wall readings may occasionally correct one translation axis.
Routes request a named checkpoint correction and receive a result; they should
not read raw sensors or depend on a particular estimator.

A future estimator could sit between sensor sampling and this public pose/correction
boundary, publishing quality/age and explicit acceptance results. It would need one
pose writer, defined task synchronization, timestamps and a failure fallback before
activation. No EKF, Monte Carlo localization, external localization library or LemLib
migration is included. Preserve the existing canonical FRONT convention and EZ motion
engine regardless of future experiments.
