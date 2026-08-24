# FalconPlan

A personal planning and control stack built from classical vehicle models,
coordinate systems, and hardware abstraction.

## Current status

The repository currently contains the vehicle-side foundations used by later
planning and control work:

- World / Body / Frenet coordinate transforms
- smooth reference line construction and queries
- kinematic bicycle model
- Euler and RK4 integration
- steering, acceleration, braking, and speed constraints
- simulator and mock hardware adapters
- mock steering, throttle, brake, and CAN output
- automated unit and integration tests

Behavior, trajectory planning, and closed-loop control algorithms are not part
of the current implementation.

## Structure

```text
FalconPlan/
|-- coordinate_system/  # coordinate states, transforms, and reference line
|-- vehicle_model/      # vehicle state, commands, parameters, and dynamics
|-- hal/                # common vehicle interface and backend adapters
|-- demos/              # small runnable examples
|-- tests/              # unit and integration tests
`-- docs/               # architecture notes
```

`coordinate_system` owns the World, Body, and Frenet representations.

`vehicle_model` owns the kinematic bicycle state transition and vehicle
constraints.

`hal` exposes the same command and state interface for simulator and mock
hardware backends.

## Quick start

Run the test suite from the repository root:

```bash
python -m pytest
```

Run two small examples:

```bash
python -m demos.coordinate_demo
python -m demos.adapter_switch_demo
```

## Roadmap

- behavior decision
- trajectory planning
- path tracking
- optimization-based control
- CARLA / ROS2 / HIL

