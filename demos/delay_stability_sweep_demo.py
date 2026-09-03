import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from control.actuator import SteeringActuator
from control.delay import SteeringCommandDelay
from control.lqr import LQRController
from control.metrics import compute_tracking_metrics
from control.pid import PIDController
from control.stanley import StanleyController
from control.trajectory_tracker import TrajectoryTracker

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

from vehicle_model.kinematic_bicycle import KinematicBicycleModel
from vehicle_model.models import (
    VehicleCommand,
    VehicleParams,
    VehicleState,
)


# ============================================================
# Result
# ============================================================

@dataclass
class SweepResult:
    controller: str
    delay_seconds: float

    completed: bool

    cte_rmse: float
    max_cte: float
    final_cte: float

    heading_rmse_deg: float
    max_heading_error_deg: float

    max_steering_deg: float
    max_steering_rate_deg_s: float

    simulation_time: float

    times: list[float]
    cte_history: list[float]


# ============================================================
# 1. W04 trajectory
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
        offsets=(-0.5, 0.0, 0.5),
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
# 2. Shared parameters
# ============================================================

params = VehicleParams(
    wheel_base=2.8,

    max_steer=0.5,

    max_accel=3.0,
    max_decel=6.0,

    max_speed=40.0,
    min_speed=0.0,
)


dt = 0.05

max_time = 8.0

max_steps = int(
    max_time / dt
)


max_steer_rate = math.radians(
    90.0
)


delays = [
    0.00,
    0.05,
    0.10,
    0.15,
]


# ============================================================
# 3. Shared initial condition
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
# 4. One benchmark run
# ============================================================

def run_case(
    controller_name: str,
    delay_seconds: float,
) -> SweepResult:

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
    # IMPORTANT:
    # Monotonic trajectory tracker
    # --------------------------------------------------------

    tracker = TrajectoryTracker(
        trajectory_x=target.x,
        trajectory_y=target.y,
        trajectory_yaw=target.yaw,

        wheel_base=params.wheel_base,

        search_window=20,
    )

    # --------------------------------------------------------
    # Delay
    # --------------------------------------------------------

    delay = SteeringCommandDelay(
        delay_seconds=delay_seconds,

        dt=dt,

        initial_steer=0.0,
    )

    # --------------------------------------------------------
    # Rate-limited steering actuator
    # --------------------------------------------------------

    actuator = SteeringActuator(
        max_steer=params.max_steer,

        max_steer_rate=max_steer_rate,
    )

    # --------------------------------------------------------
    # Fresh vehicle state
    # --------------------------------------------------------

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

    cross_track_errors = []
    heading_errors = []

    actual_steering_commands = []

    completed = False


    # ========================================================
    # Closed loop
    # ========================================================

    for k in range(
        max_steps + 1
    ):

        t = k * dt

        # ----------------------------------------------------
        # A. Monotonic tracking
        # ----------------------------------------------------

        (
            nearest_index,
            heading_error,
            cross_track_error,
        ) = tracker.compute_errors(
            state
        )

        # ----------------------------------------------------
        # B. W04 speed / acceleration references
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
        # D. Lateral controller
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
                        target.curvature[
                            nearest_index
                        ]
                    ),
                )
            )

        else:

            raise ValueError(
                f"Unknown controller: "
                f"{controller_name}"
            )

        # ----------------------------------------------------
        # E. Delay
        # ----------------------------------------------------

        delayed_steering = delay.update(
            requested_steer=(
                requested_steering
            )
        )

        # ----------------------------------------------------
        # F. Rate limit + steering saturation
        # ----------------------------------------------------

        actual_steering = actuator.update(
            target_steer=(
                delayed_steering
            ),

            dt=dt,
        )

        # ----------------------------------------------------
        # G. Vehicle
        # ----------------------------------------------------

        command = VehicleCommand(
            delta=actual_steering,
            a=acceleration,
        )

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

        cross_track_errors.append(
            cross_track_error
        )

        heading_errors.append(
            heading_error
        )

        actual_steering_commands.append(
            actual_steering
        )

        # ----------------------------------------------------
        # I. End condition
        # ----------------------------------------------------

        if nearest_index >= (
            len(target.x) - 2
        ):

            completed = True

            break


    # ========================================================
    # Metrics
    # ========================================================

    metrics = compute_tracking_metrics(
        cross_track_errors=(
            cross_track_errors
        ),

        heading_errors=(
            heading_errors
        ),

        steering_commands=(
            actual_steering_commands
        ),

        dt=dt,
    )


    return SweepResult(
        controller=controller_name,

        delay_seconds=delay_seconds,

        completed=completed,

        cte_rmse=(
            metrics.cross_track_rmse
        ),

        max_cte=(
            metrics.max_cross_track_error
        ),

        final_cte=(
            metrics.final_cross_track_error
        ),

        heading_rmse_deg=math.degrees(
            metrics.heading_rmse
        ),

        max_heading_error_deg=math.degrees(
            metrics.max_heading_error
        ),

        max_steering_deg=math.degrees(
            metrics.max_steering
        ),

        max_steering_rate_deg_s=(
            math.degrees(
                metrics.max_steering_rate
            )
        ),

        simulation_time=times[-1],

        times=times,

        cte_history=(
            cross_track_errors
        ),
    )


# ============================================================
# 5. Run sweep
# ============================================================

results = []


for delay_seconds in delays:

    for controller_name in [
        "Stanley",
        "LQR",
    ]:

        print(
            f"Running "
            f"{controller_name}, "
            f"delay="
            f"{delay_seconds * 1000:.0f} ms"
        )

        result = run_case(
            controller_name=(
                controller_name
            ),

            delay_seconds=(
                delay_seconds
            ),
        )

        results.append(
            result
        )


# ============================================================
# 6. Print robustness table
# ============================================================

print()
print("=" * 130)

print(
    "FALCONPLAN W05 - "
    "STEERING DELAY ROBUSTNESS SWEEP"
)

print("=" * 130)

print(
    f"{'Controller':<12}"
    f"{'Delay [ms]':>12}"
    f"{'Completed':>12}"
    f"{'CTE RMSE [m]':>16}"
    f"{'Max CTE [m]':>14}"
    f"{'Final CTE [m]':>16}"
    f"{'Heading RMSE [deg]':>22}"
    f"{'Max Heading [deg]':>20}"
)

print("-" * 130)


for result in results:

    print(
        f"{result.controller:<12}"

        f"{result.delay_seconds * 1000:>12.0f}"

        f"{str(result.completed):>12}"

        f"{result.cte_rmse:>16.4f}"

        f"{result.max_cte:>14.4f}"

        f"{result.final_cte:>16.4f}"

        f"{result.heading_rmse_deg:>22.3f}"

        f"{result.max_heading_error_deg:>20.3f}"
    )


# ============================================================
# 7. CTE RMSE vs delay
# ============================================================

stanley_results = [
    result
    for result in results
    if result.controller == "Stanley"
]


lqr_results = [
    result
    for result in results
    if result.controller == "LQR"
]


delay_ms = [
    result.delay_seconds * 1000
    for result in stanley_results
]


plt.figure()

plt.plot(
    delay_ms,

    [
        result.cte_rmse
        for result in stanley_results
    ],

    marker="o",

    label="Stanley",
)

plt.plot(
    delay_ms,

    [
        result.cte_rmse
        for result in lqr_results
    ],

    marker="o",

    label="LQR",
)

plt.xlabel(
    "Steering delay [ms]"
)

plt.ylabel(
    "Cross-track RMSE [m]"
)

plt.title(
    "FalconPlan W05 - Delay Robustness"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 8. Maximum CTE vs delay
# ============================================================

plt.figure()

plt.plot(
    delay_ms,

    [
        result.max_cte
        for result in stanley_results
    ],

    marker="o",

    label="Stanley",
)

plt.plot(
    delay_ms,

    [
        result.max_cte
        for result in lqr_results
    ],

    marker="o",

    label="LQR",
)

plt.xlabel(
    "Steering delay [ms]"
)

plt.ylabel(
    "Max |cross-track error| [m]"
)

plt.title(
    "Maximum Tracking Error vs Steering Delay"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 9. Individual CTE histories
# ============================================================

for delay_seconds in delays:

    stanley_case = next(
        result
        for result in results
        if (
            result.controller
            == "Stanley"
            and result.delay_seconds
            == delay_seconds
        )
    )

    lqr_case = next(
        result
        for result in results
        if (
            result.controller
            == "LQR"
            and result.delay_seconds
            == delay_seconds
        )
    )

    plt.figure()

    plt.plot(
        stanley_case.times,
        stanley_case.cte_history,

        label="Stanley",
    )

    plt.plot(
        lqr_case.times,
        lqr_case.cte_history,

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
        "CTE - Steering Delay "
        f"{delay_seconds * 1000:.0f} ms"
    )

    plt.grid()
    plt.legend()

    plt.show()