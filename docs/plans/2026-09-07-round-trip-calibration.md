# LiDAR-referenced round-trip speed calibration

The robot profile enables a 3 cm outward and return measurement, followed by an
independent repeat using the measured forward and reverse command gains. A
fresh rear-clear signal is required throughout, as are the existing raw range,
hazard, map and TF gates. Estop is never automatically released.

Each leg has a 7 s timeout and total execution is bounded at 35 s. A 600 ms
stationary interval permits filtered range readings to settle. Candidate gains
outside 0.75 to 1.25 fail; they are never silently clamped. Completion requires
return within 6 mm by LiDAR and 8 mm by wheel odometry, straightness, map/wheel
agreement, and a corrected repeat within 6 mm of nominal integrated distance.

Verified gains are atomically saved and published with readiness. Safety applies
forward/reverse gains only with ready and fresh gain messages; reset/failure
deactivates them. These are low-speed command corrections, not wheel geometry,
TF extrinsics or an independent absolute map calibration. Boot revalidates.

Tests model a 0.92 actuator response, both return trips, interrupted control,
stalls, rear-clearance loss and persisted correction completion.
