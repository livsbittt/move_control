# Automatic startup calibration

Stationary sensor checks automatically advance to one bounded motion validation
when emergency stop is explicitly released and all existing safety gates pass.
No separate validate-motion click is needed. The normal path is collecting,
waiting_motion (with the actual blocking reason), validating_motion, ready.
The existing 0.008 m/s, 4 s / 4 cm limits and sensor agreement checks remain.
Failure or abort is terminal until retry; readiness loss never starts new motion.
The result is persisted before readiness is published. Boot never releases estop.

The dashboard hides the manual motion button in automatic mode and explains that
releasing estop allows the sequence to advance. Manual mode remains available via
calibration_auto_motion=false. Floor sensing is unchanged.

Validation covers automatic progression through saved readiness without a motion
command, emergency stop waiting, and no automatic retry after abort or failure.
