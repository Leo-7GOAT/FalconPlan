import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from control.actuator import SteeringActuator
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

    requested_steering: list[float]
    actual_steering: list[float]

    target_speeds: list[float]
    actual_speeds: list[float]

    acceleration_commands: list[float]

    final_state: VehicleState


# ============================================================
# 1. Build W04 target trajectory
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
# 2. Shared vehicle configuration
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
# 3. Shared actuator configuration
# ============================================================

max_steer_rate = math.radians(
    90.0
)


# ============================================================
# 4. Simulation settings
# ============================================================

dt = 0.05

max_time = 8.0

max_steps = int(
    max_time / dt
)


# ============================================================
# 5. Initial disturbance
#
# Both controllers start from exactly the same state.
# Vehicle starts 1.5 m to the RIGHT of W04 trajectory.
# ============================================================

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
# 6. Common geometry
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
    # Front axle position
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
    # e_psi = path_yaw - vehicle_yaw
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
# 7. Run one controller
# ============================================================

def run_controller(
    controller_name: str,
) -> ControllerRunResult:

    # --------------------------------------------------------
    # Fresh plant
    # --------------------------------------------------------

    model = KinematicBicycleModel(
        params=params,
    )

    # --------------------------------------------------------
    # Fresh longitudinal controller
    # --------------------------------------------------------

    pid = PIDController(
        Kp=0.8,
        Ki=0.1,
        Kd=0.05,

        output_min=-params.max_decel,
        output_max=params.max_accel,
    )

    # --------------------------------------------------------
    # Fresh lateral controllers
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # Fresh steering actuator
    #
    # Both competitors get exactly the same actuator.
    # --------------------------------------------------------

    steering_actuator = SteeringActuator(
        max_steer=params.max_steer,

        max_steer_rate=max_steer_rate,
    )

    # --------------------------------------------------------
    # Fresh initial state
    # --------------------------------------------------------

    state = VehicleState(
        x=initial_state.x,
        y=initial_state.y,

        psi=initial_state.psi,
        v=initial_state.v,
    )

    # ========================================================
    # Logs
    # ========================================================

    times = []

    vehicle_x = []
    vehicle_y = []

    cross_track_errors = []
    heading_errors = []

    requested_steering_log = []
    actual_steering_log = []

    target_speeds = []
    actual_speeds = []

    acceleration_commands = []


    # ========================================================
    # Closed-loop simulation
    # ========================================================

    for k in range(
        max_steps + 1
    ):

        t = k * dt

        # ----------------------------------------------------
        # A. Tracking errors
        # ----------------------------------------------------

        (
            nearest_index,
            heading_error,
            cross_track_error,
        ) = compute_tracking_errors(
            state
        )

        # ----------------------------------------------------
        # B. W04 longitudinal references
        # ----------------------------------------------------

        target_speed = target.s_d[
            nearest_index
        ]

        target_acceleration = target.s_dd[
            nearest_index
        ]

        # ----------------------------------------------------
        # C. Shared longitudinal controller
        #
        # feedforward + PID feedback
        # ----------------------------------------------------

        acceleration = pid.update(
            target=target_speed,

            measurement=state.v,

            dt=dt,

            feedforward=target_acceleration,
        )

        # ----------------------------------------------------
        # D. Requested steering
        #
        # This is what the ideal controller WANTS.
        # ----------------------------------------------------

        if controller_name == "Stanley":

            requested_steering = (
                stanley.update(
                    heading_error=heading_error,

                    cross_track_error=(
                        cross_track_error
                    ),

                    speed=state.v,
                )
            )

        elif controller_name == "LQR":

            reference_curvature = (
                target.curvature[
                    nearest_index
                ]
            )

            requested_steering = (
                lqr.update(
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
            )

        else:

            raise ValueError(
                f"Unknown controller: "
                f"{controller_name}"
            )

        # ----------------------------------------------------
        # E. Steering actuator
        #
        # requested_steering != actual_steering
        #
        # The actuator imposes:
        #
        # |delta(k+1) - delta(k)|
        # <= max_steer_rate * dt
        # ----------------------------------------------------

        actual_steering = (
            steering_actuator.update(
                target_steer=(
                    requested_steering
                ),

                dt=dt,
            )
        )

        # ----------------------------------------------------
        # F. Actual vehicle command
        # ----------------------------------------------------

        command = VehicleCommand(
            delta=actual_steering,
            a=acceleration,
        )

        # ----------------------------------------------------
        # G. W02 plant
        # ----------------------------------------------------

        state = model.step(
            state=state,

            command=command,

            dt=dt,

            method="rk4",
        )

        # ----------------------------------------------------
        # H. Logs
        # ----------------------------------------------------

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

        requested_steering_log.append(
            requested_steering
        )

        actual_steering_log.append(
            actual_steering
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

        # ----------------------------------------------------
        # I. End of W04 trajectory
        # ----------------------------------------------------

        if nearest_index >= (
            len(target.x) - 2
        ):
            break


    # ========================================================
    # Unified metrics
    #
    # IMPORTANT:
    #
    # We measure ACTUAL steering,
    # not requested steering.
    # ========================================================

    metrics = compute_tracking_metrics(
        cross_track_errors=(
            cross_track_errors
        ),

        heading_errors=(
            heading_errors
        ),

        steering_commands=(
            actual_steering_log
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

        requested_steering=(
            requested_steering_log
        ),

        actual_steering=(
            actual_steering_log
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
# 8. Run fair competition
# ============================================================

stanley_result = run_controller(
    "Stanley"
)

lqr_result = run_controller(
    "LQR"
)


# ============================================================
# 9. Benchmark table
# ============================================================

print()
print("=" * 100)

print(
    "FALCONPLAN W05 - STANLEY VS LQR "
    "WITH STEERING RATE LIMIT"
)

print("=" * 100)

print(
    f"Steering rate limit: "
    f"{math.degrees(max_steer_rate):.1f} deg/s"
)

print()

print(
    f"{'Metric':<36}"
    f"{'Stanley':>20}"
    f"{'LQR':>20}"
)

print("-" * 100)


def print_row(
    name,
    stanley_value,
    lqr_value,
):

    print(
        f"{name:<36}"
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
    "Max actual steering [deg]",

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
# 10. RMSE comparison
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


rmse_change = (
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
    f"{rmse_change:.2f}%"
)


# ============================================================
# 11. World trajectory comparison
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

    label="Stanley + actuator",
)

plt.plot(
    lqr_result.vehicle_x,
    lqr_result.vehicle_y,

    linewidth=2.0,

    label="LQR + actuator",
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
    "Stanley vs LQR with Steering Rate Limit"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 12. Cross-track error
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
    "Cross-Track Error with Steering Actuator"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 13. Requested vs actual steering: Stanley
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.requested_steering
    ),

    linestyle="--",

    label="Requested",
)

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.actual_steering
    ),

    label="Actual",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering [deg]"
)

plt.title(
    "Stanley - Requested vs Actual Steering"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 14. Requested vs actual steering: LQR
# ============================================================

plt.figure()

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.requested_steering
    ),

    linestyle="--",

    label="Requested",
)

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.actual_steering
    ),

    label="Actual",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering [deg]"
)

plt.title(
    "LQR - Requested vs Actual Steering"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 15. Actual steering comparison
# ============================================================

plt.figure()

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.actual_steering
    ),

    label="Stanley",
)

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.actual_steering
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
    "Actual steering [deg]"
)

plt.title(
    "Actual Steering Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 16. Actual steering-rate comparison
# ============================================================

stanley_rate = (
    np.diff(
        stanley_result.actual_steering
    )
    / dt
)

lqr_rate = (
    np.diff(
        lqr_result.actual_steering
    )
    / dt
)


plt.figure()

plt.plot(
    stanley_result.times[1:],

    np.degrees(
        stanley_rate
    ),

    label="Stanley",
)

plt.plot(
    lqr_result.times[1:],

    np.degrees(
        lqr_rate
    ),

    label="LQR",
)

plt.axhline(
    math.degrees(
        max_steer_rate
    ),

    linestyle="--",

    label="Rate limit",
)

plt.axhline(
    -math.degrees(
        max_steer_rate
    ),

    linestyle="--",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering rate [deg/s]"
)

plt.title(
    "Actual Steering Rate Comparison"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 17. Heading error comparison
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
# 18. Longitudinal sanity check
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