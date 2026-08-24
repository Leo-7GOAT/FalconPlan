import math

from models import (
    VehicleState,
    VehicleCommand,
    VehicleParams
)

from kinematic_bicycle import (
    KinematicBicycleModel
)


state = VehicleState(
    x=0.0,
    y=0.0,
    psi=0.0,
    v=10.0
)

command = VehicleCommand(
    delta=math.radians(10),
    a=0.0
)

params = VehicleParams(
    wheel_base=2.8,
    max_steer=math.radians(35),
    max_accel=3.0,
    max_decel=6.0,
    max_speed=40.0,
    min_speed=0.0
)

model = KinematicBicycleModel(params)


next_state = model.step(
    state,
    command,
    dt=0.5
)

print("Default RK4:", next_state)


next_state = model.step(
    state,
    command,
    dt=0.5,
    method="euler"
)

print("Euler:", next_state)