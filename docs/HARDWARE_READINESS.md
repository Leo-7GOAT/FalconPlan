# Hardware readiness — PENDING_HARDWARE

Interface status: **Hardware-Ready / Pending Physical Validation**. R550 manufacturer protocol, steering geometry, control limits and actuator calibration are unknown. No CAN/serial frame, physical E-stop or Pi HIL result is invented.

`hal/base.py`: get_state, apply_command, step. VehicleState(x,y,psi,v) uses rear axle, right-handed world/map, meters, CCW radians, signed forward m/s. VehicleCommand(delta,a): road-wheel radians and m/s². Timestamps remain transport envelopes in one monotonic clock domain; simulator/mock/replay/hardware provenance must be explicit.

Safety evaluates timestamps before dispatch. Timeout, geofence, speed, reverse, collision and software E-stop latch STOPPED; manual commands still obey limits. Reset requires explicit acknowledgement, stationary state and valid geofence; the next evaluation checks freshness. Emergency steering/braking bypass queued ordinary commands. This is a software request, not proof of physical stopping.

## Tools and first physical week

Use [commissioning tools](../tools/hardware/README.md) for encoder/IMU/command/receipt recording, step identification, measured latency, fault injection, offline replay and tested release staging/atomic rollback. Unknown fields remain null; hardware is disabled by default. CARLA step identification only calibrates the simulator.

1. Obtain verified wiring, voltage, steering type, device command/acknowledgement and disconnect semantics. Check independent physical E-stop and power isolation.
2. Check polarity/ground/fuse, lift wheels, verify signs and encoder/IMU frames with logs before closed-loop movement.
3. Measure wheelbase, steering angle/rate/saturation, deadzone, delay, step response and braking. Fit parameters with uncertainty and retain raw samples.
4. Verify stale data, lost command/communications, manual takeover, watchdog/geofence and explicit recovery at low energy.
5. Execute low-speed straight/circle tracking, compare model residuals and measure CPU/RSS/jitter/latency on the actual Pi before expanding the operational envelope.

Release selection starts no actuator process. The hardware executive must own startup interlocks and safe shutdown. Vehicle-specific driver, wiring and on-target HIL remain PENDING_HARDWARE.
