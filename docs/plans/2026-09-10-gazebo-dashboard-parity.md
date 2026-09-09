# Gazebo and dashboard processing path

Isolated Gazebo, the dashboard, and the robot use the same command chain and the same safety-fused gauges.

- `enable_simulation_scans()` is the only opt-in for GPU lidar. Production still rejects sim-time `/scan`.
- Isolated worlds mount the GPU lidar at yaw=π so scan 0 is the rear, matching C1. TF nose is π.
- `web_node` always publishes teleop on `/cmd_vel_raw`. Gauges wait for `/safety/*`; they do not fill F/L/R from raw `/scan`.
- `/robot/evidence_scope` labels synthetic IR/IMU/US/camera. It does not change the decision path.
- `tools/gz/test_planning.sh` remains a planner driver without safety. Production-identical dashboard is `run_calibration_mapping.sh`.

Wheel contact and independent sensors remain HOLD.
