import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from control.lqr import LQRController
from control.metrics import (
    TrackingMetrics,
    compute_tracking_metrics,
)
from control.pid import PIDController
from control.stanley import StanleyController

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
    VehicleCommand,
    VehicleParams,
    VehicleState,
)


# ============================================================
# Benchmark result
# ============================================================

@dataclass
class ControllerRunResult:
    name: str

    metrics: TrackingMetrics

    times: list[float]

    vehicle_x: list[float]
    vehicle_y: list[float]

    cross_track_errors: list[float]
    heading_errors: list[float]

    steering_commands: list[float]

    target_speeds: list[float]
    actual_speeds: list[float]

    acceleration_commands: list[float]

    final_state: VehicleState


# ============================================================
# 1. Build W04 trajectory
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
# 2. Shared vehicle parameters
# ============================================================

params = VehicleParams(
    wheel_base=2.8,

    max_steer=0.5,

    max_accel=3.0,
    max_decel=6.0,

    max_speed=40.0,
    min_speed=0.0,
)


# ============================================================
# 3. Shared benchmark conditions
# ============================================================

dt = 0.05

max_time = 8.0

max_steps = int(
    max_time / dt
)


initial_offset = 1.5

initial_yaw = target.yaw[0]


initial_state = VehicleState(
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
# 4. Common tracking-error calculation
#
# Same geometry and sign convention for Stanley and LQR.
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
):

    # --------------------------------------------------------
    # Front axle
    # --------------------------------------------------------

    front_x = (
        state.x
        + params.wheel_base
        * math.cos(state.psi)
    )

    front_y = (
        state.y
        + params.wheel_base
        * math.sin(state.psi)
    )

    # --------------------------------------------------------
    # Nearest W04 trajectory point
    # --------------------------------------------------------

    nearest_index = min(
        range(len(target.x)),
        key=lambda i: (
            (target.x[i] - front_x) ** 2
            + (target.y[i] - front_y) ** 2
        ),
    )

    path_x = target.x[
        nearest_index
    ]

    path_y = target.y[
        nearest_index
    ]

    path_yaw = target.yaw[
        nearest_index
    ]

    # --------------------------------------------------------
    # Heading error
    #
    # e_psi = path - vehicle
    # --------------------------------------------------------

    heading_error = normalize_angle(
        path_yaw
        - state.psi
    )

    # --------------------------------------------------------
    # Signed cross-track error
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
# 5. Run one controller
# ============================================================

def run_controller(
    controller_name: str,
) -> ControllerRunResult:

    # --------------------------------------------------------
    # Every benchmark run gets its OWN fresh model/state/PID.
    #
    # Otherwise the second controller would inherit history.
    # --------------------------------------------------------

    model = KinematicBicycleModel(
        params=params,
    )

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

    lqr = LQRController(
        q_cross_track=1.0,
        q_heading=1.0,

        r_steer=10.0,

        max_steer=params.max_steer,
    )

    # Frozen dataclass, so create a fresh one.
    state = VehicleState(
        x=initial_state.x,
        y=initial_state.y,
        psi=initial_state.psi,
        v=initial_state.v,
    )

    # --------------------------------------------------------
    # Logs
    # --------------------------------------------------------

    times = []

    vehicle_x = []
    vehicle_y = []

    cross_track_errors = []
    heading_errors = []

    steering_commands = []

    target_speeds = []
    actual_speeds = []

    acceleration_commands = []


    # --------------------------------------------------------
    # Closed loop
    # --------------------------------------------------------

    for k in range(
        max_steps + 1
    ):

        t = k * dt

        (
            nearest_index,
            heading_error,
            cross_track_error,
        ) = compute_tracking_errors(
            state
        )

        # ====================================================
        # Shared longitudinal controller
        # ====================================================

        target_speed = target.s_d[
            nearest_index
        ]

        target_acceleration = target.s_dd[
            nearest_index
        ]

        acceleration = pid.update(
            target=target_speed,

            measurement=state.v,

            dt=dt,

            feedforward=target_acceleration,
        )

        # ====================================================
        # Only lateral controller changes
        # ====================================================

        if controller_name == "Stanley":

            steering = stanley.update(
                heading_error=heading_error,

                cross_track_error=(
                    cross_track_error
                ),

                speed=state.v,
            )

        elif controller_name == "LQR":

            reference_curvature = (
                target.curvature[
                    nearest_index
                ]
            )

            steering = lqr.update(
                cross_track_error=(
                    cross_track_error
                ),

                heading_error=(
                    heading_error
                ),

                speed=state.v,

                wheel_base=(
                    params.wheel_base
                ),

                dt=dt,

                reference_curvature=(
                    reference_curvature
                ),
            )

        else:

            raise ValueError(
                f"Unknown controller: "
                f"{controller_name}"
            )

        # ====================================================
        # Combined VehicleCommand
        # ====================================================

        command = VehicleCommand(
            delta=steering,
            a=acceleration,
        )

        state = model.step(
            state=state,

            command=command,

            dt=dt,

            method="rk4",
        )

        # ====================================================
        # Logs
        # ====================================================

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

        target_speeds.append(
            target_speed
        )

        actual_speeds.append(
            state.v
        )

        acceleration_commands.append(
            acceleration
        )

        # ====================================================
        # End of current W04 trajectory
        # ====================================================

        if nearest_index >= (
            len(target.x) - 2
        ):
            break


    # --------------------------------------------------------
    # Unified metrics
    # --------------------------------------------------------

    metrics = compute_tracking_metrics(
        cross_track_errors=(
            cross_track_errors
        ),

        heading_errors=(
            heading_errors
        ),

        steering_commands=(
            steering_commands
        ),

        dt=dt,
    )


    return ControllerRunResult(
        name=controller_name,

        metrics=metrics,

        times=times,

        vehicle_x=vehicle_x,
        vehicle_y=vehicle_y,

        cross_track_errors=(
            cross_track_errors
        ),

        heading_errors=(
            heading_errors
        ),

        steering_commands=(
            steering_commands
        ),

        target_speeds=(
            target_speeds
        ),

        actual_speeds=(
            actual_speeds
        ),

        acceleration_commands=(
            acceleration_commands
        ),

        final_state=state,
    )


# ============================================================
# 6. Run benchmark
# ============================================================

stanley_result = run_controller(
    "Stanley"
)

lqr_result = run_controller(
    "LQR"
)


# ============================================================
# 7. Print benchmark table
# ============================================================

print()
print("=" * 92)
print(
    "FALCONPLAN W05 - STANLEY VS LQR BENCHMARK"
)
print("=" * 92)

print(
    f"{'Metric':<32}"
    f"{'Stanley':>20}"
    f"{'LQR':>20}"
)

print("-" * 92)


def print_row(
    name,
    stanley_value,
    lqr_value,
):

    print(
        f"{name:<32}"
        f"{stanley_value:>20.6f}"
        f"{lqr_value:>20.6f}"
    )


print_row(
    "Cross-track RMSE [m]",

    stanley_result
    .metrics
    .cross_track_rmse,

    lqr_result
    .metrics
    .cross_track_rmse,
)


print_row(
    "Max |CTE| [m]",

    stanley_result
    .metrics
    .max_cross_track_error,

    lqr_result
    .metrics
    .max_cross_track_error,
)


print_row(
    "Final CTE [m]",

    stanley_result
    .metrics
    .final_cross_track_error,

    lqr_result
    .metrics
    .final_cross_track_error,
)


print_row(
    "Heading RMSE [deg]",

    math.degrees(
        stanley_result
        .metrics
        .heading_rmse
    ),

    math.degrees(
        lqr_result
        .metrics
        .heading_rmse
    ),
)


print_row(
    "Max heading error [deg]",

    math.degrees(
        stanley_result
        .metrics
        .max_heading_error
    ),

    math.degrees(
        lqr_result
        .metrics
        .max_heading_error
    ),
)


print_row(
    "Max steering [deg]",

    math.degrees(
        stanley_result
        .metrics
        .max_steering
    ),

    math.degrees(
        lqr_result
        .metrics
        .max_steering
    ),
)


print_row(
    "Steering-rate RMSE [deg/s]",

    math.degrees(
        stanley_result
        .metrics
        .steering_rate_rmse
    ),

    math.degrees(
        lqr_result
        .metrics
        .steering_rate_rmse
    ),
)


print_row(
    "Max steering rate [deg/s]",

    math.degrees(
        stanley_result
        .metrics
        .max_steering_rate
    ),

    math.degrees(
        lqr_result
        .metrics
        .max_steering_rate
    ),
)


# ============================================================
# 8. Improvement summary
# ============================================================

stanley_rmse = (
    stanley_result
    .metrics
    .cross_track_rmse
)

lqr_rmse = (
    lqr_result
    .metrics
    .cross_track_rmse
)


rmse_improvement = (
    (
        stanley_rmse
        - lqr_rmse
    )
    / stanley_rmse
    * 100.0
)


print()

print(
    "LQR cross-track RMSE improvement "
    f"vs Stanley: "
    f"{rmse_improvement:.2f}%"
)


# ============================================================
# 9. World trajectory comparison
# ============================================================

plt.figure(
    figsize=(11, 7)
)

plt.plot(
    target.x,
    target.y,

    linestyle="--",
    linewidth=2.0,

    label="W04 target",
)

plt.plot(
    stanley_result.vehicle_x,
    stanley_result.vehicle_y,

    linewidth=2.0,

    label="Stanley",
)

plt.plot(
    lqr_result.vehicle_x,
    lqr_result.vehicle_y,

    linewidth=2.0,

    label="LQR",
)

plt.scatter(
    [initial_state.x],
    [initial_state.y],

    label="Initial state",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W05 - Stanley vs LQR Tracking"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 10. Cross-track error comparison
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,
    stanley_result.cross_track_errors,

    label="Stanley",
)

plt.plot(
    lqr_result.times,
    lqr_result.cross_track_errors,

    label="LQR",
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
    "Cross-Track Error Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 11. Heading error comparison
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.heading_errors
    ),

    label="Stanley",
)

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.heading_errors
    ),

    label="LQR",
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
    "Heading Error Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 12. Steering comparison
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.steering_commands
    ),

    label="Stanley",
)

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.steering_commands
    ),

    label="LQR",
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
    "Steering angle [deg]"
)

plt.title(
    "Steering Command Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 13. Steering-rate comparison
# ============================================================

stanley_steering_rate = (
    np.diff(
        stanley_result.steering_commands
    )
    / dt
)

lqr_steering_rate = (
    np.diff(
        lqr_result.steering_commands
    )
    / dt
)


plt.figure()

plt.plot(
    stanley_result.times[1:],

    np.degrees(
        stanley_steering_rate
    ),

    label="Stanley",
)

plt.plot(
    lqr_result.times[1:],

    np.degrees(
        lqr_steering_rate
    ),

    label="LQR",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering rate [deg/s]"
)

plt.title(
    "Steering Smoothness Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 14. Speed sanity check
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,
    stanley_result.target_speeds,

    linestyle="--",
    label="W04 target speed",
)

plt.plot(
    stanley_result.times,
    stanley_result.actual_speeds,

    label="Stanley run",
)

plt.plot(
    lqr_result.times,
    lqr_result.actual_speeds,

    label="LQR run",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Speed [m/s]"
)

plt.title(
    "Longitudinal Control Sanity Check"
)

plt.grid()
plt.legend()

plt.show()