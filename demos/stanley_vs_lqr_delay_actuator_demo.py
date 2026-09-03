import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from control.actuator import SteeringActuator
from control.delay import SteeringCommandDelay
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
    delayed_steering: list[float]
    actual_steering: list[float]

    target_speeds: list[float]
    actual_speeds: list[float]

    acceleration_commands: list[float]

    final_state: VehicleState


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
# 2. W04 planner
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
# 3. Shared vehicle parameters
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
# 4. Actuator model
# ============================================================

dt = 0.05

steering_delay_seconds = 0.15

max_steer_rate = math.radians(
    90.0
)


# ============================================================
# 5. Benchmark initial state
# ============================================================

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

    heading_error = normalize_angle(
        path_yaw
        - state.psi
    )

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
    # Fresh vehicle model
    # --------------------------------------------------------

    model = KinematicBicycleModel(
        params=params,
    )

    # --------------------------------------------------------
    # Fresh PID
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
    # Fresh delay
    # --------------------------------------------------------

    steering_delay = SteeringCommandDelay(
        delay_seconds=steering_delay_seconds,
        dt=dt,
        initial_steer=0.0,
    )

    # --------------------------------------------------------
    # Fresh rate-limited actuator
    # --------------------------------------------------------

    steering_actuator = SteeringActuator(
        max_steer=params.max_steer,

        max_steer_rate=max_steer_rate,
    )

    # --------------------------------------------------------
    # Fresh state
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
    delayed_steering_log = []
    actual_steering_log = []

    target_speeds = []
    actual_speeds = []

    acceleration_commands = []


    # ========================================================
    # Closed loop
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
        # C. Longitudinal FF + PID
        # ----------------------------------------------------

        acceleration = pid.update(
            target=target_speed,

            measurement=state.v,

            dt=dt,

            feedforward=target_acceleration,
        )

        # ----------------------------------------------------
        # D. Ideal controller request
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
        # E. Communication / actuator delay
        # ----------------------------------------------------

        delayed_steering = (
            steering_delay.update(
                requested_steer=(
                    requested_steering
                )
            )
        )

        # ----------------------------------------------------
        # F. Physical steering rate limit
        # ----------------------------------------------------

        actual_steering = (
            steering_actuator.update(
                target_steer=(
                    delayed_steering
                ),

                dt=dt,
            )
        )

        # ----------------------------------------------------
        # G. Actual command reaching vehicle
        # ----------------------------------------------------

        command = VehicleCommand(
            delta=actual_steering,
            a=acceleration,
        )

        # ----------------------------------------------------
        # H. W02 vehicle dynamics
        # ----------------------------------------------------

        state = model.step(
            state=state,

            command=command,

            dt=dt,

            method="rk4",
        )

        # ----------------------------------------------------
        # I. Logs
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

        delayed_steering_log.append(
            delayed_steering
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
        # J. End of W04 trajectory
        # ----------------------------------------------------

        if nearest_index >= (
            len(target.x) - 2
        ):
            break


    # ========================================================
    # Metrics measure the ACTUAL steering reaching vehicle.
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

        delayed_steering=(
            delayed_steering_log
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
# 8. Run benchmark
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
print("=" * 104)

print(
    "FALCONPLAN W05 - STANLEY VS LQR "
    "WITH DELAY + STEERING RATE LIMIT"
)

print("=" * 104)

print(
    f"Steering delay     : "
    f"{steering_delay_seconds:.3f} s"
)

print(
    f"Steering rate limit: "
    f"{math.degrees(max_steer_rate):.1f} deg/s"
)

print()

print(
    f"{'Metric':<40}"
    f"{'Stanley':>20}"
    f"{'LQR':>20}"
)

print("-" * 104)


def print_row(
    name,
    stanley_value,
    lqr_value,
):

    print(
        f"{name:<40}"
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
# 10. Relative CTE RMSE result
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
# 11. World trajectories
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
    "Stanley vs LQR with Delay + Rate Limit"
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
    "Cross-Track Error with Delay + Rate Limit"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 13. Stanley steering pipeline
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
        stanley_result.delayed_steering
    ),

    label="Delayed",
)

plt.plot(
    stanley_result.times,

    np.degrees(
        stanley_result.actual_steering
    ),

    linewidth=2.0,

    label="Actual",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering [deg]"
)

plt.title(
    "Stanley Steering Pipeline"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 14. LQR steering pipeline
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
        lqr_result.delayed_steering
    ),

    label="Delayed",
)

plt.plot(
    lqr_result.times,

    np.degrees(
        lqr_result.actual_steering
    ),

    linewidth=2.0,

    label="Actual",
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Steering [deg]"
)

plt.title(
    "LQR Steering Pipeline"
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
# 16. Steering rate comparison
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
    "Actual Steering Rate"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 17. Heading error
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