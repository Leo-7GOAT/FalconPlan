import math

import matplotlib.pyplot as plt
import numpy as np

from control.stanley import (
    StanleyController,
)

from vehicle_model.kinematic_bicycle import (
    KinematicBicycleModel,
)

from vehicle_model.models import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
)


# ============================================================
# Vehicle
# ============================================================

params = VehicleParams(
    wheel_base=2.8,

    max_steer=0.5,

    max_accel=3.0,
    max_decel=6.0,

    max_speed=40.0,
    min_speed=0.0,
)


model = KinematicBicycleModel(
    params=params,
)


# ============================================================
# Stanley controller
# ============================================================

stanley = StanleyController(
    k=2.0,
    softening=1.0,

    max_steer=params.max_steer,
)


# ============================================================
# Straight reference trajectory
#
# y = 0
# yaw = 0
# ============================================================

trajectory_x = np.linspace(
    0.0,
    150.0,
    301,
).tolist()

trajectory_y = [
    0.0
    for _ in trajectory_x
]

trajectory_yaw = [
    0.0
    for _ in trajectory_x
]


# ============================================================
# Initial vehicle state
#
# Vehicle starts 2 m to the right of path.
# ============================================================

state = VehicleState(
    x=0.0,
    y=-2.0,

    psi=0.0,

    v=10.0,
)


dt = 0.05
total_time = 10.0

steps = int(
    total_time / dt
)


# ============================================================
# Logs
# ============================================================

times = []

vehicle_x = []
vehicle_y = []

cross_track_errors = []
heading_errors = []

steering_commands = []


# ============================================================
# Closed-loop tracking
# ============================================================

for k in range(
    steps + 1
):

    t = k * dt

    # --------------------------------------------------------
    # 1. Compute tracking errors
    # --------------------------------------------------------

    (
        nearest_index,
        heading_error,
        cross_track_error,
    ) = stanley.compute_errors(
        state=state,

        trajectory_x=trajectory_x,
        trajectory_y=trajectory_y,
        trajectory_yaw=trajectory_yaw,

        wheel_base=params.wheel_base,
    )

    # --------------------------------------------------------
    # 2. Stanley steering
    # --------------------------------------------------------

    steering = stanley.update(
        heading_error=heading_error,
        cross_track_error=cross_track_error,

        speed=state.v,
    )

    # --------------------------------------------------------
    # 3. Vehicle command
    #
    # This demo only tests lateral control.
    # Longitudinal acceleration stays zero.
    # --------------------------------------------------------

    command = VehicleCommand(
        delta=steering,
        a=0.0,
    )

    # --------------------------------------------------------
    # 4. Vehicle dynamics
    # --------------------------------------------------------

    state = model.step(
        state=state,
        command=command,

        dt=dt,
        method="rk4",
    )

    # --------------------------------------------------------
    # 5. Logs
    # --------------------------------------------------------

    times.append(
        t
    )

    vehicle_x.append(
        state.x
    )

    vehicle_y.append(
        state.y
    )

    cross_track_errors.append(
        cross_track_error
    )

    heading_errors.append(
        heading_error
    )

    steering_commands.append(
        steering
    )


# ============================================================
# Result
# ============================================================

print(
    "final x:",
    state.x,
)

print(
    "final y:",
    state.y,
)

print(
    "final yaw [deg]:",
    math.degrees(
        state.psi
    ),
)

print(
    "final cross-track error:",
    cross_track_errors[-1],
)

print(
    "max steering [deg]:",
    max(
        abs(
            math.degrees(delta)
        )
        for delta in steering_commands
    ),
)


# ============================================================
# Vehicle trajectory
# ============================================================

plt.figure(
    figsize=(11, 6)
)

plt.plot(
    trajectory_x,
    trajectory_y,

    linestyle="--",
    label="Target path",
)

plt.plot(
    vehicle_x,
    vehicle_y,

    linewidth=2.5,
    label="Vehicle trajectory",
)

plt.scatter(
    [0.0],
    [-2.0],

    label="Initial vehicle position",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "Stanley Straight Path Tracking"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# Cross-track error
# ============================================================

plt.figure()

plt.plot(
    times,
    cross_track_errors,
)

plt.axhline(
    0.0,
    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Cross-track error [m]"
)

plt.title(
    "Stanley Cross-Track Error"
)

plt.grid()

plt.show()


# ============================================================
# Steering command
# ============================================================

plt.figure()

plt.plot(
    times,
    [
        math.degrees(delta)
        for delta in steering_commands
    ],
)

plt.axhline(
    math.degrees(
        params.max_steer
    ),

    linestyle="--",
    label="Steering limit",
)

plt.axhline(
    -math.degrees(
        params.max_steer
    ),

    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering angle [deg]"
)

plt.title(
    "Stanley Steering Command"
)

plt.grid()
plt.legend()

plt.show()