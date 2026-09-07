# Directional clearance for short calibration

Use the measured LiDAR mount translation and chassis circumradius to calculate the front and rear bumper cone extents independently. Preserve the configured front stand-off, hysteresis, blind-zone protection and conservative fallback when the transform is unavailable. Do not interpret an unspecified 7 cm as a safe sensor-origin threshold.

The safety node publishes its actual active thresholds and nearest forward/rear ranges. Startup calibration consumes fresh telemetry instead of assuming a separate fixed 20 cm clearance. Reserve the selected outbound stroke plus stopping/return-error allowance. Select 2?4 cm within available room; keep the established minimum motion-evidence distance, two repeated round trips, odometry/LiDAR/map agreement and failure behavior.

The dashboard reports available and required front/rear distances, the selected stroke, and the runtime stop limits. All distances are explicitly LiDAR-origin distances. Camera remains observation-only. IR, IMU, emergency-stop, stale-sensor and safety-gate checks remain authoritative.

Validation: pure geometry and clearance boundary tests; isolated ROS tests for actual limit publication, stale telemetry rejection and narrow-space round-trip selection; dashboard field tests; commit before SSH deployment; real browser calibration attempt with actual result recorded separately from automated test success.
