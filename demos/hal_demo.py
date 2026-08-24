import math

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)

from hal import (
    SimulatorAdapter
)


params = VehicleParams(
    wheel_base=2.8,
    max_steer=math.radians(35),
    max_accel=3.0,
    max_decel=6.0,
    max_speed=40.0,
    min_speed=0.0
)

model = KinematicBicycleModel(
    params
)

initial_state = VehicleState(
    x=0.0,
    y=0.0,
    psi=0.0,
    v=10.0
)

vehicle = SimulatorAdapter(
    model=model,
    initial_state=initial_state
)

command = VehicleCommand(
    delta=math.radians(10),
    a=0.0
)

vehicle.apply_command(
    command
)

for i in range(10):

    state = vehicle.step(
        dt=0.1
    )

    print(
        f"step={i + 1}, "
        f"x={state.x:.3f}, "
        f"y={state.y:.3f}, "
        f"psi={math.degrees(state.psi):.3f} deg, "
        f"v={state.v:.3f}"
    )