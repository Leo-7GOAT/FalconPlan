import math

import matplotlib.pyplot as plt
import numpy as np

from control.pid import PIDController
from control.stanley import StanleyController

from coordinate_system.reference_line import ReferenceLine

from planning.collision import CollisionParams
from planning.constraints import TrajectoryConstraints
from planning.cost import TrajectoryCostWeights

from planning.lattice_planner import (
    FrenetLatticePlanner,
    FrenetPlanningInput,
)

from planning.sampling import LateralSamplingConfig
from planning.world_constraints import WorldTrajectoryConstraints

from vehicle_model.kinematic_bicycle import (
    KinematicBicycleModel,
)

from vehicle_model.models import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
)


# ============================================================
# 1. W04 Reference Road
# ============================================================

reference_path = [
    (0.0, 0.0),
    (25.0, 0.0),
    (50.0, 3.0),
    (75.0, 10.0),
    (100.0, 20.0),
    (130.0, 30.0),
]

reference_line = ReferenceLine(
    reference_path=reference_path,
)


# ============================================================
# 2. W04 Planner
# ============================================================

planner = FrenetLatticePlanner(
    reference_line=reference_line,

    trajectory_constraints=TrajectoryConstraints(
        max_speed=30.0,
        max_longitudinal_accel=3.0,
        max_longitudinal_jerk=5.0,
        max_lateral_accel=2.5,
        max_lateral_jerk=8.0,
    ),

    world_constraints=WorldTrajectoryConstraints(
        max_curvature=0.03,
    ),

    collision_params=CollisionParams(
        vehicle_radius=1.0,
        safety_margin=0.3,
    ),

    cost_weights=TrajectoryCostWeights(),

    lateral_sampling=LateralSamplingConfig(
        offsets=(
            -0.5,
            0.0,
            0.5,
        )
    ),

    dt=0.1,
)


planning_input = FrenetPlanningInput(
    s0=0.0,
    s_d0=20.0,
    s_dd0=0.0,

    d0=0.0,
    d_d0=0.0,
    d_dd0=0.0,

    lane_centers=[
        0.0,
        3.5,
    ],

    target_speed_values=[
        23.0,
        25.0,
        27.0,
    ],

    duration_values=[
        2.5,
        3.0,
        3.5,
        4.0,
    ],

    desired_d=3.5,
    desired_speed=25.0,

    obstacles=[],
)


planning_result = planner.plan(
    planning_input
)

target = planning_result.best


print(
    "W04 best:",
    f"dT={target.target_d:.1f}",
    f"vT={target.target_speed:.1f}",
    f"T={target.duration:.1f}",
)


# ============================================================
# 3. W02 Vehicle Model
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
# 4. W05 Controllers
# ============================================================

pid = PIDController(
    Kp=0.8,
    Ki=0.1,
    Kd=0.05,

    output_min=-params.max_decel,
    output_max=params.max_accel,
)


stanley = StanleyController(
    k=2.0,
    softening=1.0,

    max_steer=params.max_steer,
)


# ============================================================
# 5. Initial State
#
# Deliberately start 1.5 m to the right of W04 trajectory.
# ============================================================

initial_offset = 1.5

initial_yaw = target.yaw[0]


state = VehicleState(
    x=(
        target.x[0]
        + initial_offset
        * math.sin(initial_yaw)
    ),

    y=(
        target.y[0]
        - initial_offset
        * math.cos(initial_yaw)
    ),

    psi=initial_yaw,

    # Start from W04 planned initial speed.
    v=target.s_d[0],
)


# ============================================================
# 6. Simulation
# ============================================================

dt = 0.05

max_time = 8.0

max_steps = int(
    max_time / dt
)


# ============================================================
# Logs
# ============================================================

times = []

vehicle_x = []
vehicle_y = []

actual_speeds = []
target_speeds = []

speed_errors = []
cross_track_errors = []
heading_errors = []

steering_commands = []
acceleration_commands = []

nearest_indices = []


# ============================================================
# 7. Closed-loop Control
# ============================================================

for k in range(
    max_steps + 1
):

    t = k * dt

    # --------------------------------------------------------
    # A. Lateral tracking errors
    # --------------------------------------------------------

    (
        nearest_index,
        heading_error,
        cross_track_error,
    ) = stanley.compute_errors(
        state=state,

        trajectory_x=target.x,
        trajectory_y=target.y,
        trajectory_yaw=target.yaw,

        wheel_base=params.wheel_base,
    )

    # --------------------------------------------------------
    # B. Target speed from W04 trajectory
    # --------------------------------------------------------

    target_speed = target.s_d[
        nearest_index
    ]

    target_acceleration = target.s_dd[
        nearest_index
    ]


    # --------------------------------------------------------
    # C. Longitudinal PID
    # --------------------------------------------------------

    acceleration = pid.update(
        target=target_speed,
        measurement=state.v,
        dt=dt,
        feedforward=target_acceleration,
    )

    # --------------------------------------------------------
    # D. Lateral Stanley
    # --------------------------------------------------------

    steering = stanley.update(
        heading_error=heading_error,
        cross_track_error=cross_track_error,
        speed=state.v,
    )

    # --------------------------------------------------------
    # E. Combined vehicle command
    # --------------------------------------------------------

    command = VehicleCommand(
        delta=steering,
        a=acceleration,
    )

    # --------------------------------------------------------
    # F. Vehicle dynamics
    # --------------------------------------------------------

    state = model.step(
        state=state,
        command=command,
        dt=dt,
        method="rk4",
    )

    # --------------------------------------------------------
    # G. Logs
    # --------------------------------------------------------

    times.append(t)

    vehicle_x.append(
        state.x
    )

    vehicle_y.append(
        state.y
    )

    actual_speeds.append(
        state.v
    )

    target_speeds.append(
        target_speed
    )

    speed_errors.append(
        target_speed - state.v
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

    acceleration_commands.append(
        acceleration
    )

    nearest_indices.append(
        nearest_index
    )

    # --------------------------------------------------------
    # H. End of current trajectory
    # --------------------------------------------------------

    if nearest_index >= (
        len(target.x) - 2
    ):
        break


# ============================================================
# 8. Metrics
# ============================================================

cte_array = np.asarray(
    cross_track_errors,
    dtype=float,
)

speed_error_array = np.asarray(
    speed_errors,
    dtype=float,
)


cte_rmse = float(
    np.sqrt(
        np.mean(
            cte_array ** 2
        )
    )
)

speed_rmse = float(
    np.sqrt(
        np.mean(
            speed_error_array ** 2
        )
    )
)


max_cte = float(
    np.max(
        np.abs(
            cte_array
        )
    )
)

max_speed_error = float(
    np.max(
        np.abs(
            speed_error_array
        )
    )
)


max_steering_deg = max(
    abs(
        math.degrees(delta)
    )
    for delta in steering_commands
)


max_acceleration = max(
    acceleration_commands
)

min_acceleration = min(
    acceleration_commands
)


# ============================================================
# 9. Result
# ============================================================

print()
print("=" * 70)
print("PID + STANLEY W04 TRACKING RESULT")
print("=" * 70)

print(
    "simulation time:",
    times[-1],
)

print(
    "final nearest index:",
    nearest_indices[-1],
    "/",
    len(target.x) - 1,
)

print()

print(
    "cross-track RMSE:",
    cte_rmse,
)

print(
    "max |cross-track error|:",
    max_cte,
)

print(
    "final cross-track error:",
    cross_track_errors[-1],
)

print()

print(
    "speed RMSE:",
    speed_rmse,
)

print(
    "max |speed error|:",
    max_speed_error,
)

print(
    "final speed:",
    state.v,
)

print(
    "final target speed:",
    target_speeds[-1],
)

print()

print(
    "max steering [deg]:",
    max_steering_deg,
)

print(
    "max acceleration:",
    max_acceleration,
)

print(
    "min acceleration:",
    min_acceleration,
)

print()

print(
    "final vehicle state:"
)

print(state)


# ============================================================
# 10. World Tracking Plot
# ============================================================

plt.figure(
    figsize=(11, 7)
)

plt.plot(
    target.x,
    target.y,
    linestyle="--",
    linewidth=2.0,
    label="W04 best trajectory",
)

plt.plot(
    vehicle_x,
    vehicle_y,
    linewidth=2.5,
    label="PID + Stanley vehicle",
)

plt.scatter(
    [vehicle_x[0]],
    [vehicle_y[0]],
    label="Initial vehicle position",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W05 - PID + Stanley Tracking"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 11. Speed Tracking
# ============================================================

plt.figure()

plt.plot(
    times,
    target_speeds,
    linestyle="--",
    label="W04 target speed",
)

plt.plot(
    times,
    actual_speeds,
    label="Actual speed",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Speed [m/s]"
)

plt.title(
    "W04 Speed Profile Tracking"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 12. Cross-track Error
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
    "PID + Stanley Cross-Track Error"
)

plt.grid()

plt.show()


# ============================================================
# 13. Steering Command
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
    "Steering [deg]"
)

plt.title(
    "Stanley Steering Command"
)

plt.grid()

plt.show()


# ============================================================
# 14. Acceleration Command
# ============================================================

plt.figure()

plt.plot(
    times,
    acceleration_commands,
)

plt.axhline(
    params.max_accel,
    linestyle="--",
)

plt.axhline(
    -params.max_decel,
    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Acceleration [m/s^2]"
)

plt.title(
    "PID Acceleration Command"
)

plt.grid()

plt.show()