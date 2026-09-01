# FalconPlan architecture and contracts

## Layering

```text
traffic / road state
    -> prediction and risk metrics
    -> behavior FSM + IDM + MOBIL
    -> lane/speed/horizon objective
    -> Frenet lattice generation
    -> feasibility, World projection, collision, and cost filters
    -> selected World-frame trajectory
    -> future closed-loop tracking
    -> VehicleHAL -> simulator or mock-hardware adapter
```

Algorithm code consumes project-owned state types. Backend conversion stays
inside `VehicleHAL` adapters; planning code does not import a simulator or
hardware transport.

## Coordinate conventions

- World: right-handed planar frame; x/y in m, yaw in rad, positive yaw is CCW.
- Body: origin at the rear-axle reference point; x forward and y left.
- Frenet: s is reference-line arc length, d is left-positive displacement, and
  `d_prime = dd/ds`.
- Vehicle: `(x, y, psi)` is the rear-axle World pose and `v` is longitudinal
  speed. `delta` is road-wheel steering, not steering-wheel angle.

World/Body transforms are rigid SE(2) transforms. World/Frenet conversion
projects to the continuous reference spline and rejects local singularities.

## Behavior contracts

`BehaviorPlanner` combines current and predicted TTC/THW risk, IDM
longitudinal acceleration, MOBIL lane-change evaluation, and a hysteretic FSM.
The result is a behavior-level decision; it is not itself a geometric or
trajectory plan.

## Frenet lattice contracts

`FrenetLatticePlanner` receives initial Frenet derivatives, lane centers,
terminal-speed samples, duration samples, a desired lateral/speed objective,
and circular obstacles. It returns a selected `FrenetTrajectory`, a cost
breakdown, and candidate counts from each filter stage.

The pipeline is:

1. sample lateral targets near configured lane centers;
2. construct quintic lateral and quartic longitudinal trajectories;
3. enforce Frenet speed, acceleration, and jerk limits;
4. project each survivor to World `(x, y, yaw)` and estimate curvature;
5. enforce World curvature limits;
6. perform swept-segment collision checks using vehicle radius, obstacle radius,
   and safety margin;
7. rank the remaining candidates by weighted objective cost.

If any hard-filter stage produces no candidates, the planner raises a clear
`RuntimeError`; it does not return a colliding or infeasible fallback.

## Vehicle model and HAL

The rear-axle kinematic bicycle model is:

```text
x_dot   = v cos(psi)
y_dot   = v sin(psi)
psi_dot = v tan(delta) / wheel_base
v_dot   = a
```

Euler and RK4 integration are available. Commands are saturated before
integration. `SimulatorAdapter` advances the shared model directly;
`MockHardwareAdapter` maps the same command to mutually exclusive normalized
throttle/brake channels and a bounded CAN-like steering payload before
advancing a local plant.

## Current limitations

- Static circular obstacles only in W04 planning; no time-indexed prediction or
  online replanning loop.
- Kinematic point/reference model; no tire-force, slip, delay, or dynamic
  bicycle model.
- No implemented closed-loop tracker for the selected trajectory.
- No production CAN transport, watchdog, fault state, or safety case.
