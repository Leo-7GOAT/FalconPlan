import math
import matplotlib.pyplot as plt

from coordinate_system import(
    ReferenceLine,
    FrenetTransformer,
    FrenetState,
    WorldState,
)

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)

def world_to_vehicle_state(
        world_state: WorldState,
        v:float
) -> VehicleState:

    return VehicleState(
        x=world_state.x,
        y=world_state.y,
        psi=world_state.theta,
        v=v
    )
def vehicle_to_world_state(
        vehicle_state: VehicleState
) -> WorldState:

    return WorldState(
        x=vehicle_state.x,
        y=vehicle_state.y,
        theta=vehicle_state.psi
    )

reference_points = [
    (0.0, 0.0),
    (5.0, 1.0),
    (10.0, 4.0),
    (15.0, 8.0),
    (20.0, 10.0),
]

reference_line = ReferenceLine(
    reference_points
)

transformer = FrenetTransformer(
    reference_line
)

frenet_state = FrenetState(
    s=10.0,
    d=0.0,
    d_prime=0.0
)

world_state = transformer.frenet_to_world(
    frenet_state
)

print("初始 Frenet:")
print(frenet_state)

print("\n转换后的 World:")
print(world_state)

vehicle_state = world_to_vehicle_state(
    world_state,
    v=5.0
)

print("\n转换后的 VehicleState:")
print(vehicle_state)

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

command = VehicleCommand(
    delta=0.0,
    a=0.0
)

next_vehicle_state = model.step(
    vehicle_state,
    command,
    dt=0.1
)

print("\n运动后的 VehicleState:")
print(next_vehicle_state)

next_world_state = vehicle_to_world_state(
    next_vehicle_state
)

next_frenet_state = transformer.world_to_frenet(
    next_world_state
)

print("\n运动后的 WorldState:")
print(next_world_state)

print("\n运动后的 FrenetState:")
print(next_frenet_state)

T = 2.0
dt = 0.1
steps = int(T / dt)

current_state = vehicle_state

xs = [current_state.x]
ys = [current_state.y]

times = [0.0]
d_history = [0.0]

for i in range(steps):

    current_state = model.step(
        current_state,
        command,
        dt=dt
    )

    current_world = vehicle_to_world_state(
        current_state
    )

    current_frenet = transformer.world_to_frenet(
        current_world
    )

    xs.append(current_state.x)
    ys.append(current_state.y)

    times.append((i + 1) * dt)
    d_history.append(current_frenet.d)

    print(
        f"t={(i + 1) * dt:.1f}s, "
        f"s={current_frenet.s:.3f}, "
        f"d={current_frenet.d:.3f}, "
        f"d'={current_frenet.d_prime:.3f}"
    )

    ref_x = [
        p[0] for p in reference_points
    ]

    ref_y = [
        p[1] for p in reference_points
    ]

    plt.figure()

    plt.plot(
        ref_x,
        ref_y,
        "o-",
        label="Reference"
    )

    plt.plot(
        xs,
        ys,
        label="Vehicle"
    )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.grid()

    plt.show()

    plt.figure()

    plt.plot(
        times,
        d_history
    )

    plt.xlabel("time [s]")
    plt.ylabel("lateral error d [m]")
    plt.grid()

    plt.show()