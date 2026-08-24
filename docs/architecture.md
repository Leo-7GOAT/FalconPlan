# Architecture

FalconPlan currently covers coordinate handling, a classical vehicle model,
and a small hardware abstraction layer. Planning and control modules will be
added in later stages.

## Dependency direction

```text
Coordinate System
       |
       v
Vehicle Model
       |
       v
HAL
       |
       v
future Behavior / Planning / Control
```

The lower layers provide shared state and execution interfaces. Future modules
should depend on those interfaces instead of selecting a simulator or hardware
backend directly.

## Coordinate System

`coordinate_system` defines World, Body, and Frenet data types. It includes
point transforms, a smooth reference line, and conversions between world and
Frenet vehicle states.

The coordinate layer does not make behavior or trajectory decisions.

## Vehicle Model

`vehicle_model` defines `VehicleState`, `VehicleCommand`, and `VehicleParams`.
`KinematicBicycleModel` advances the state with Euler or RK4 integration and
applies the configured steering, acceleration, braking, and speed limits.

The model is shared by the current backends so their state transitions use the
same equations.

## Hardware abstraction layer

```text
VehicleHAL
|-- SimulatorAdapter
`-- MockHardwareAdapter
```

`VehicleHAL` defines the command, step, and state interface.

`SimulatorAdapter` connects that interface to the local vehicle model.

`MockHardwareAdapter` maps commands to mock steering, throttle, and brake
signals, records a mock CAN frame, and advances a local plant model.

## Future modules

Behavior, planning, and control will sit above the existing layers. They are
roadmap items rather than current modules. Simulator-specific, ROS2, and HIL
integration can be added as HAL backends without changing their public command
and state boundary.

