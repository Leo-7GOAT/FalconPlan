# FalconPlan

FalconPlan is a compact autonomous-driving planning and control portfolio
project. The accepted W01/W02 scope establishes trustworthy coordinate,
vehicle-model, and hardware-abstraction foundations before behavior, planning,
and closed-loop control are added.

## W01/W02 capabilities

- World frame: x forward in the map, y left, yaw counter-clockwise.
- Body frame: x forward, y left, origin at the vehicle reference point.
- Frenet frame: s along the reference line, d positive to its left, and
  `d_prime = dd/ds`.
- Smooth approximately arc-length-parameterized reference line with continuous
  World/Frenet projection and singularity guards.
- Rear-axle kinematic bicycle model with steering, acceleration, braking, and
  speed limits.
- Real Euler and fourth-order Runge-Kutta integration.
- One `VehicleHAL` contract with switchable simulator and mock-hardware
  backends. The mock maps acceleration to mutually exclusive normalized
  throttle/brake channels and records a bounded road-wheel steering CAN-like
  payload.
- Unit, numerical, integration, invalid-input, and adapter-switch tests.

All positions are metres, time is seconds, speed is m/s, acceleration is
m/s^2, and angles/angular rates are radians and rad/s.

## Architecture

```text
World / VehicleState
        |
future Behavior / Planning
        |
future Control
        |
VehicleHAL
   |-------------------|
SimulatorAdapter   MockHardwareAdapter
   |                   |
Kinematic model    actuator/CAN semantics + local plant
```

Algorithm code depends on `VehicleHAL`, never on a specific simulator or
hardware transport. See `docs/architecture.md` for interface and frame details.

## Repository layout

```text
coordinate_system/  coordinate states, reference line, and transforms
vehicle_model/      state/command/parameters and kinematic bicycle model
hal/                backend-neutral vehicle interface and two adapters
demos/              focused coordinate, model, and adapter examples
tests/              W01/W02 acceptance evidence
docs/               architecture and scope boundaries
```

## Install, test, and run

Python 3.9 or newer is supported.

```bash
python -m pip install -e ".[dev]"
python -m pytest tests -v
python -m demos.coordinate_demo
python -m demos.adapter_switch_demo
```

The adapter demo prints `PASS` for both backends and verifies that identical
upper-level code produces the same state transition.

## Acceptance boundary

W01/W02 cover the engineering foundation above. W03 will introduce rule-based
behavior work. FSM, TTC/THW, CV/CA prediction, IDM, MOBIL, trajectory planning,
path tracking, CARLA/ROS2, and real CAN I/O are intentionally not implemented
or claimed here.
