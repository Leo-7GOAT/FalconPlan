import math

import matplotlib.pyplot as plt
import numpy as np

from control.pid import PIDController
from control.lqr import LQRController

from coordinate_system.reference_line import (
    ReferenceLine,
)

from planning.collision import (
    CollisionParams,
)

from planning.constraints import (
    TrajectoryConstraints,
)

from planning.cost import (
    TrajectoryCostWeights,
)

from planning.lattice_planner import (
    FrenetLatticePlanner,
    FrenetPlanningInput,
)

from planning.sampling import (
    LateralSamplingConfig,
)

from planning.world_constraints import (
    WorldTrajectoryConstraints,
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
# Tracking geometry helper
#
# Same sign convention as Stanley:
#
# heading_error = path_yaw - vehicle_yaw
#
# cross_track_error > 0:
# target path is on the LEFT side of vehicle
# ============================================================

def normalize_angle(
    angle: float,
) -> float:

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def compute_tracking_errors(
    state: VehicleState,
    trajectory_x,
    trajectory_y,
    trajectory_yaw,
    wheel_base: float,
):

    if len(trajectory_x) == 0:
        raise ValueError(
            "trajectory must not be empty"
        )

    if not (
        len(trajectory_x)
        == len(trajectory_y)
        == len(trajectory_yaw)
    ):
        raise ValueError(
            "trajectory lengths must match"
        )

    # --------------------------------------------------------
    # Front axle position
    # --------------------------------------------------------

    front_x = (
        state.x
        + wheel_base
        * math.cos(state.psi)
    )

    front_y = (
        state.y
        + wheel_base
        * math.sin(state.psi)
    )

    # --------------------------------------------------------
    # Nearest trajectory point
    # --------------------------------------------------------

    nearest_index = min(
        range(len(trajectory_x)),
        key=lambda i: (
            (trajectory_x[i] - front_x) ** 2
            + (trajectory_y[i] - front_y) ** 2
        ),
    )

    path_x = trajectory_x[
        nearest_index
    ]

    path_y = trajectory_y[
        nearest_index
    ]

    path_yaw = trajectory_yaw[
        nearest_index
    ]

    # --------------------------------------------------------
    # Heading error
    #
    # e_psi = psi_path - psi_vehicle
    # --------------------------------------------------------

    heading_error = normalize_angle(
        path_yaw - state.psi
    )

    # --------------------------------------------------------
    # Signed cross-track error
    #
    # Left normal:
    #
    # n = (-sin(path_yaw), cos(path_yaw))
    #
    # e_y = (P_path - P_front) dot n
    # --------------------------------------------------------

    cross_track_error = (
        (path_x - front_x)
        * (-math.sin(path_yaw))
        +
        (path_y - front_y)
        * math.cos(path_yaw)
    )

    return (
        nearest_index,
        heading_error,
        cross_track_error,
    )


# ============================================================
# 1. W04 reference road
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
# 2. W04 Frenet Lattice Planner
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
# 3. W02 vehicle model
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
# 4. W05 longitudinal controller
#
# W04 acceleration feedforward
# +
# PID speed feedback
# ============================================================

pid = PIDController(
    Kp=0.8,
    Ki=0.1,
    Kd=0.05,

    output_min=-params.max_decel,
    output_max=params.max_accel,
)


# ============================================================
# 5. W05 LQR lateral controller
# ============================================================

lqr = LQRController(
    q_cross_track=1.0,
    q_heading=1.0,

    r_steer=10.0,

    max_steer=params.max_steer,
)


# ============================================================
# 6. Initial vehicle state
#
# Same experiment as Stanley:
# deliberately start 1.5 m to the RIGHT of target trajectory.
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

    v=target.s_d[0],
)


# ============================================================
# 7. Simulation settings
# ============================================================

dt = 0.05

max_time = 8.0

max_steps = int(
    max_time / dt
)


# ============================================================
# 8. Logs
# ============================================================

times = []

vehicle_x = []
vehicle_y = []

target_speeds = []
actual_speeds = []

cross_track_errors = []
heading_errors = []
speed_errors = []

steering_commands = []
acceleration_commands = []

nearest_indices = []


# ============================================================
# 9. Closed-loop simulation
# ============================================================

for k in range(
    max_steps + 1
):

    t = k * dt

    # --------------------------------------------------------
    # A. Compute lateral tracking errors
    # --------------------------------------------------------

    (
        nearest_index,
        heading_error,
        cross_track_error,
    ) = compute_tracking_errors(
        state=state,

        trajectory_x=target.x,
        trajectory_y=target.y,
        trajectory_yaw=target.yaw,

        wheel_base=params.wheel_base,
    )

    # --------------------------------------------------------
    # B. Read W04 longitudinal references
    # --------------------------------------------------------

    target_speed = target.s_d[
        nearest_index
    ]

    target_acceleration = target.s_dd[
        nearest_index
    ]

    # --------------------------------------------------------
    # C. Longitudinal:
    #
    # acceleration feedforward + PID feedback
    # --------------------------------------------------------

    acceleration = pid.update(
        target=target_speed,

        measurement=state.v,

        dt=dt,

        feedforward=target_acceleration,
    )

    # --------------------------------------------------------
    # D. Lateral:
    #
    # curvature feedforward + LQR feedback
    # --------------------------------------------------------

    reference_curvature = target.curvature[
        nearest_index
    ]

    steering = lqr.update(
        cross_track_error=cross_track_error,

        heading_error=heading_error,

        speed=state.v,

        wheel_base=params.wheel_base,

        dt=dt,

        reference_curvature=reference_curvature,
    )

    # --------------------------------------------------------
    # E. Combine longitudinal + lateral command
    # --------------------------------------------------------

    command = VehicleCommand(
        delta=steering,
        a=acceleration,
    )

    # --------------------------------------------------------
    # F. W02 vehicle model
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

    times.append(
        t
    )

    vehicle_x.append(
        state.x
    )

    vehicle_y.append(
        state.y
    )

    target_speeds.append(
        target_speed
    )

    actual_speeds.append(
        state.v
    )

    cross_track_errors.append(
        cross_track_error
    )

    heading_errors.append(
        heading_error
    )

    speed_errors.append(
        target_speed
        - state.v
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
    # H. End of current W04 trajectory
    # --------------------------------------------------------

    if nearest_index >= (
        len(target.x) - 2
    ):
        break


# ============================================================
# 10. Metrics
# ============================================================

cte_array = np.asarray(
    cross_track_errors,
    dtype=float,
)

heading_array = np.asarray(
    heading_errors,
    dtype=float,
)

speed_error_array = np.asarray(
    speed_errors,
    dtype=float,
)

steering_array = np.asarray(
    steering_commands,
    dtype=float,
)


cte_rmse = float(
    np.sqrt(
        np.mean(
            cte_array ** 2
        )
    )
)


heading_rmse = float(
    np.sqrt(
        np.mean(
            heading_array ** 2
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


max_heading_error_deg = float(
    np.max(
        np.abs(
            np.degrees(
                heading_array
            )
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


max_steering_deg = float(
    np.max(
        np.abs(
            np.degrees(
                steering_array
            )
        )
    )
)


# Steering smoothness baseline:
#
# mean absolute change between consecutive commands
if len(
    steering_array
) > 1:

    steering_smoothness = float(
        np.mean(
            np.abs(
                np.diff(
                    steering_array
                )
            )
        )
    )

else:
    steering_smoothness = 0.0


# ============================================================
# 11. Print result
# ============================================================

print()
print("=" * 70)
print("LQR + PID W04 TRACKING RESULT")
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
    "heading RMSE [deg]:",
    math.degrees(
        heading_rmse
    ),
)

print(
    "max |heading error| [deg]:",
    max_heading_error_deg,
)

print(
    "final heading error [deg]:",
    math.degrees(
        heading_errors[-1]
    ),
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
    "steering smoothness "
    "[mean |delta_k-delta_(k-1)| rad]:",
    steering_smoothness,
)

print()

print(
    "max acceleration:",
    max(
        acceleration_commands
    ),
)

print(
    "min acceleration:",
    min(
        acceleration_commands
    ),
)

print()

print(
    "final vehicle state:"
)

print(state)


# ============================================================
# 12. World trajectory
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

    label="LQR + PID vehicle",
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
    "FalconPlan W05 - LQR + PID Tracking"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 13. Speed tracking
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
    "LQR + PID - Speed Profile Tracking"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 14. Cross-track error
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
    "LQR Cross-Track Error"
)

plt.grid()

plt.show()


# ============================================================
# 15. Heading error
# ============================================================

plt.figure()

plt.plot(
    times,
    [
        math.degrees(error)
        for error in heading_errors
    ],
)

plt.axhline(
    0.0,
    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Heading error [deg]"
)

plt.title(
    "LQR Heading Error"
)

plt.grid()

plt.show()


# ============================================================
# 16. Steering command
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
    "LQR Steering Command"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 17. Acceleration command
# ============================================================

plt.figure()

plt.plot(
    times,
    acceleration_commands,
)

plt.axhline(
    params.max_accel,

    linestyle="--",
    label="Max acceleration",
)

plt.axhline(
    -params.max_decel,

    linestyle="--",
    label="Max deceleration",
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
plt.legend()

plt.show()