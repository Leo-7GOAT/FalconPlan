import math

import matplotlib.pyplot as plt
import numpy as np

from control.stanley import (
    StanleyController,
)

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


print("W04 best trajectory")
print(
    f"dT={target.target_d:.1f}, "
    f"vT={target.target_speed:.1f}, "
    f"T={target.duration:.1f}"
)


# ============================================================
# 3. W02 vehicle model
# ============================================================

vehicle_params = VehicleParams(
    wheel_base=2.8,

    max_steer=0.5,

    max_accel=3.0,
    max_decel=6.0,

    max_speed=40.0,
    min_speed=0.0,
)


vehicle_model = KinematicBicycleModel(
    params=vehicle_params,
)


# ============================================================
# 4. W05 Stanley controller
# ============================================================

stanley = StanleyController(
    k=2.0,
    softening=1.0,

    max_steer=vehicle_params.max_steer,
)


# ============================================================
# 5. Initial vehicle state
#
# Put vehicle 1.5 m to the RIGHT of planned trajectory.
#
# Path left normal:
#
#     n = (-sin(yaw), cos(yaw))
#
# therefore right direction:
#
#    -n = (sin(yaw), -cos(yaw))
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

    # Lateral-only experiment.
    # Keep speed constant.
    v=15.0,
)


# ============================================================
# 6. Simulation
# ============================================================

dt = 0.05
max_time = 10.0

max_steps = int(
    max_time / dt
)


times = []

vehicle_x = []
vehicle_y = []

cross_track_errors = []
heading_errors = []

steering_commands = []
nearest_indices = []


for k in range(
    max_steps + 1
):

    t = k * dt

    # --------------------------------------------------------
    # Tracking errors
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

        wheel_base=vehicle_params.wheel_base,
    )

    # --------------------------------------------------------
    # Stanley steering
    # --------------------------------------------------------

    steering = stanley.update(
        heading_error=heading_error,
        cross_track_error=cross_track_error,

        speed=state.v,
    )
    

    # --------------------------------------------------------
    # Vehicle command
    #
    # Longitudinal control is deliberately disabled here.
    # --------------------------------------------------------

    command = VehicleCommand(
        delta=steering,
        a=0.0,
    )

    # --------------------------------------------------------
    # Vehicle dynamics
    # --------------------------------------------------------

    state = vehicle_model.step(
        state=state,
        command=command,

        dt=dt,
        method="rk4",
    )

    # --------------------------------------------------------
    # Logs
    # --------------------------------------------------------

    times.append(t)

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

    nearest_indices.append(
        nearest_index
    )

    # --------------------------------------------------------
    # Stop after reaching the end of W04 trajectory.
    # --------------------------------------------------------

    if nearest_index >= (
        len(target.x) - 2
    ):
        break


# ============================================================
# 7. Metrics
# ============================================================

cte_array = np.array(
    cross_track_errors
)

rmse_cte = float(
    np.sqrt(
        np.mean(
            cte_array ** 2
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

final_cte = float(
    cross_track_errors[-1]
)

max_steering_deg = max(
    abs(
        math.degrees(delta)
    )
    for delta in steering_commands
)


print()
print("=" * 60)
print("STANLEY W04 TRACKING RESULT")
print("=" * 60)

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

print(
    "RMSE cross-track error:",
    rmse_cte,
)

print(
    "max |cross-track error|:",
    max_cte,
)

print(
    "final cross-track error:",
    final_cte,
)

print(
    "max steering [deg]:",
    max_steering_deg,
)

print(
    "final vehicle state:"
)

print(state)


# ============================================================
# 8. World trajectory
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

    label="Stanley tracked trajectory",
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
    "FalconPlan W05 - Stanley Tracking W04 Trajectory"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 9. Cross-track error
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
    "W04 Trajectory Cross-Track Error"
)

plt.grid()

plt.show()


# ============================================================
# 10. Heading error
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
    "W04 Trajectory Heading Error"
)

plt.grid()

plt.show()


# ============================================================
# 11. Steering command
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
        vehicle_params.max_steer
    ),
    linestyle="--",
)

plt.axhline(
    -math.degrees(
        vehicle_params.max_steer
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
    "Stanley Steering Command on W04 Trajectory"
)

plt.grid()

plt.show()