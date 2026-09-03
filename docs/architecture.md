# W01/W02 architecture and contracts

## Dependency rule

Upper-level behavior, planning, and control code receives project-owned state
types and a `VehicleHAL`. It must not import a simulator API or hardware driver.
Backend-specific conversion remains inside an adapter.

```text
coordinate_system + vehicle_model
              |
future behavior / planning / control
              |
          VehicleHAL
          /        \
 SimulatorAdapter  MockHardwareAdapter
```

`SimulatorAdapter` advances the shared kinematic model directly.
`MockHardwareAdapter` first maps the same bounded `VehicleCommand` to mock
actuator channels, records a CAN-like payload, then advances a local plant.
This is a contract test double, not a claim of real vehicle integration.

## Coordinate conventions

- World: right-handed planar frame; x/y in m, yaw in rad, positive yaw is CCW.
- Body: origin at the vehicle reference point; x forward and y left.
- Frenet: s is reference-line arc length in m; d is left-positive normal
  displacement in m; `d_prime` is dimensionless `dd/ds`.
- Vehicle: `(x, y, psi)` is the rear-axle pose in World and `v` is longitudinal
  speed in m/s. `delta` is road-wheel steering in rad, not steering-wheel angle.

World/Body transforms are rigid SE(2) point transforms. World/Frenet conversion
projects to the continuous reference spline, uses
`d'=(1-kappa*d)tan(psi-theta_r)`, and rejects the local Frenet singularity.

## Vehicle model

The W02 rear-axle kinematic bicycle equations are:

```text
x_dot   = v cos(psi)
y_dot   = v sin(psi)
psi_dot = v tan(delta) / wheel_base
v_dot   = a
```

Both explicit Euler and classical RK4 are implemented. Commands are saturated
before integration and the resulting speed is constrained to configured
limits. This is intentionally a kinematic model; tire forces, slip, actuator
delay, and a dynamic bicycle model belong to later scope.

## HAL channel semantics

`VehicleHAL.get_state()` returns the current project-owned `VehicleState`.
`apply_command()` stores a `VehicleCommand`; `step(dt)` advances one positive
duration and returns the new state.

For the mock hardware backend:

- steering is bounded road-wheel angle in rad;
- throttle is normalized to `[0, 1]` for non-negative acceleration;
- brake is normalized to `[0, 1]` for negative acceleration magnitude;
- throttle and brake are mutually exclusive;
- each accepted command appends one dictionary to `can_tx_log`.

The dictionary is deliberately CAN-like rather than a real DBC/frame encoding.
Real bus transport, timestamps, acknowledgements, watchdogs, and safety states
are not claimed in W02.
