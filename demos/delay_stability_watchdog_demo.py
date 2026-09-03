import math
from dataclasses import dataclass

import matplotlib.pyplot as plt

from control.actuator import SteeringActuator
from control.delay import SteeringCommandDelay
from control.lqr import LQRController
from control.metrics import compute_tracking_metrics
from control.pid import PIDController
from control.stability import (
    TrackingAcceptanceCriteria,
    evaluate_tracking_run,
)
from control.stanley import StanleyController
from control.trajectory_tracker import TrajectoryTracker
from control.watchdog import (
    TrackingWatchdog,
    WatchdogConfig,
)

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
    watchdog_tripped: bool
    accepted: bool

    failure_reason: str

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
# 2. Shared configuration
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
# 3. Acceptance + watchdog
# ============================================================

acceptance_criteria = (
    TrackingAcceptanceCriteria(
        max_cross_track_error=2.5,

        max_heading_error_deg=20.0,

        final_cross_track_error=0.5,
    )
)


watchdog_config = WatchdogConfig(
    max_cross_track_error=2.5,

    max_heading_error_deg=20.0,

    # 3 frames * 0.05 s = 0.15 s
    violation_frames=3,
)


# ============================================================
# 4. Initial disturbance
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
# 5. Run one case
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
    # Lateral controllers
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
    # Monotonic tracker
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
    # Physical actuator
    # --------------------------------------------------------

    actuator = SteeringActuator(
        max_steer=params.max_steer,

        max_steer_rate=max_steer_rate,
    )

    # --------------------------------------------------------
    # Runtime watchdog
    # --------------------------------------------------------

    watchdog = TrackingWatchdog(
        config=watchdog_config,
    )

    # --------------------------------------------------------
    # Initial state
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

    watchdog_tripped = False
    watchdog_reason = ""


    # ========================================================
    # Closed loop
    # ========================================================

    for k in range(
        max_steps + 1
    ):

        t = k * dt

        # ----------------------------------------------------
        # A. Tracking
        # ----------------------------------------------------

        (
            nearest_index,
            heading_error,
            cross_track_error,
        ) = tracker.compute_errors(
            state
        )

        # ----------------------------------------------------
        # B. Log current tracking state FIRST
        #
        # Therefore the violating frame is included in metrics.
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

        # ----------------------------------------------------
        # C. Runtime watchdog
        # ----------------------------------------------------

        watchdog_status = watchdog.update(
            cross_track_error=(
                cross_track_error
            ),

            heading_error=(
                heading_error
            ),
        )

        if watchdog_status.tripped:

            watchdog_tripped = True

            watchdog_reason = (
                watchdog_status.reason
            )

            print(
                f"[WATCHDOG] "
                f"{controller_name} "
                f"delay="
                f"{delay_seconds * 1000:.0f} ms "
                f"t={t:.2f}s: "
                f"{watchdog_reason}"
            )

            break

        # ----------------------------------------------------
        # D. Trajectory completed
        # ----------------------------------------------------

        if nearest_index >= (
            len(target.x) - 2
        ):

            completed = True

            break

        # ----------------------------------------------------
        # E. Longitudinal trajectory reference
        # ----------------------------------------------------

        target_speed = target.s_d[
            nearest_index
        ]

        target_acceleration = target.s_dd[
            nearest_index
        ]

        # ----------------------------------------------------
        # F. Longitudinal FF + PID
        # ----------------------------------------------------

        acceleration = pid.update(
            target=target_speed,

            measurement=state.v,

            dt=dt,

            feedforward=target_acceleration,
        )

        # ----------------------------------------------------
        # G. Lateral controller
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
        # H. Steering delay
        # ----------------------------------------------------

        delayed_steering = delay.update(
            requested_steer=(
                requested_steering
            )
        )

        # ----------------------------------------------------
        # I. Rate limit
        # ----------------------------------------------------

        actual_steering = actuator.update(
            target_steer=(
                delayed_steering
            ),

            dt=dt,
        )

        actual_steering_commands.append(
            actual_steering
        )

        # ----------------------------------------------------
        # J. Vehicle command
        # ----------------------------------------------------

        command = VehicleCommand(
            delta=actual_steering,
            a=acceleration,
        )

        # ----------------------------------------------------
        # K. Plant
        # ----------------------------------------------------

        state = model.step(
            state=state,

            command=command,

            dt=dt,

            method="rk4",
        )


    # ========================================================
    # Metrics
    #
    # There can be one more error sample than steering sample
    # because watchdog checks before another command is issued.
    # Align them before calling metrics.
    # ========================================================

    metric_length = min(
        len(cross_track_errors),
        len(heading_errors),
        len(actual_steering_commands),
    )

    if metric_length == 0:
        raise RuntimeError(
            "no valid tracking samples"
        )


    metrics = compute_tracking_metrics(
        cross_track_errors=(
            cross_track_errors[
                :metric_length
            ]
        ),

        heading_errors=(
            heading_errors[
                :metric_length
            ]
        ),

        steering_commands=(
            actual_steering_commands[
                :metric_length
            ]
        ),

        dt=dt,
    )


    # ========================================================
    # Offline final acceptance
    # ========================================================

    acceptance = evaluate_tracking_run(
        completed=completed,

        max_cross_track_error=(
            metrics.max_cross_track_error
        ),

        max_heading_error_deg=(
            math.degrees(
                metrics.max_heading_error
            )
        ),

        final_cross_track_error=(
            metrics.final_cross_track_error
        ),

        criteria=acceptance_criteria,
    )


    accepted = (
        acceptance.passed
        and not watchdog_tripped
    )


    if watchdog_tripped:

        failure_reason = (
            "WATCHDOG: "
            + watchdog_reason
        )

    elif not acceptance.passed:

        failure_reason = (
            acceptance.reason
        )

    else:

        failure_reason = (
            "tracking accepted"
        )


    return SweepResult(
        controller=controller_name,

        delay_seconds=delay_seconds,

        completed=completed,

        watchdog_tripped=(
            watchdog_tripped
        ),

        accepted=accepted,

        failure_reason=(
            failure_reason
        ),

        cte_rmse=(
            metrics.cross_track_rmse
        ),

        max_cte=(
            metrics.max_cross_track_error
        ),

        final_cte=(
            metrics.final_cross_track_error
        ),

        heading_rmse_deg=(
            math.degrees(
                metrics.heading_rmse
            )
        ),

        max_heading_error_deg=(
            math.degrees(
                metrics.max_heading_error
            )
        ),

        max_steering_deg=(
            math.degrees(
                metrics.max_steering
            )
        ),

        max_steering_rate_deg_s=(
            math.degrees(
                metrics.max_steering_rate
            )
        ),

        simulation_time=(
            times[-1]
        ),

        times=(
            times[:metric_length]
        ),

        cte_history=(
            cross_track_errors[
                :metric_length
            ]
        ),
    )


# ============================================================
# 6. Run all 8 cases
# ============================================================

results = []


for delay_seconds in delays:

    for controller_name in [
        "Stanley",
        "LQR",
    ]:

        print()

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
# 7. Final robustness table
# ============================================================

print()
print("=" * 160)

print(
    "FALCONPLAN W05 - "
    "DELAY ROBUSTNESS + WATCHDOG + ACCEPTANCE"
)

print("=" * 160)

print(
    f"{'Controller':<10}"
    f"{'Delay':>8}"
    f"{'Done':>8}"
    f"{'Watchdog':>11}"
    f"{'PASS':>8}"
    f"{'CTE RMSE':>12}"
    f"{'Max CTE':>12}"
    f"{'Head RMSE':>12}"
    f"{'Max Head':>12}"
    f"{'Time':>9}"
    f"  {'Reason'}"
)

print("-" * 160)


for result in results:

    print(
        f"{result.controller:<10}"

        f"{result.delay_seconds * 1000:>7.0f}"
        f"ms"

        f"{str(result.completed):>8}"

        f"{str(result.watchdog_tripped):>11}"

        f"{str(result.accepted):>8}"

        f"{result.cte_rmse:>12.3f}"

        f"{result.max_cte:>12.3f}"

        f"{result.heading_rmse_deg:>12.2f}"

        f"{result.max_heading_error_deg:>12.2f}"

        f"{result.simulation_time:>9.2f}"

        f"  {result.failure_reason}"
    )


# ============================================================
# 8. Determine maximum validated delay
# ============================================================

print()
print("=" * 80)
print("MAXIMUM VALIDATED DELAY")
print("=" * 80)


for controller_name in [
    "Stanley",
    "LQR",
]:

    controller_results = [
        result
        for result in results
        if (
            result.controller
            == controller_name
        )
    ]

    passed_delays = [
        result.delay_seconds
        for result in controller_results
        if result.accepted
    ]

    if passed_delays:

        max_validated_delay = max(
            passed_delays
        )

        print(
            f"{controller_name:<10}: "
            f"{max_validated_delay * 1000:.0f} ms"
        )

    else:

        print(
            f"{controller_name:<10}: "
            f"no validated delay"
        )


# ============================================================
# 9. RMSE vs delay
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


plt.figure()

plt.plot(
    [
        result.delay_seconds * 1000
        for result in stanley_results
    ],

    [
        result.cte_rmse
        for result in stanley_results
    ],

    marker="o",

    label="Stanley",
)

plt.plot(
    [
        result.delay_seconds * 1000
        for result in lqr_results
    ],

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
    "CTE RMSE [m]"
)

plt.title(
    "FalconPlan W05 - "
    "Validated Delay Robustness"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 10. CTE history by delay
# ============================================================

for delay_seconds in delays:

    stanley_case = next(
        result
        for result in results
        if (
            result.controller == "Stanley"
            and result.delay_seconds
            == delay_seconds
        )
    )

    lqr_case = next(
        result
        for result in results
        if (
            result.controller == "LQR"
            and result.delay_seconds
            == delay_seconds
        )
    )


    plt.figure()

    plt.plot(
        stanley_case.times,
        stanley_case.cte_history,

        label=(
            "Stanley "
            + (
                "PASS"
                if stanley_case.accepted
                else "FAIL"
            )
        ),
    )

    plt.plot(
        lqr_case.times,
        lqr_case.cte_history,

        label=(
            "LQR "
            + (
                "PASS"
                if lqr_case.accepted
                else "FAIL"
            )
        ),
    )

    plt.axhline(
        acceptance_criteria
        .max_cross_track_error,

        linestyle="--",
        label="CTE limit",
    )

    plt.axhline(
        -acceptance_criteria
        .max_cross_track_error,

        linestyle="--",
    )

    plt.xlabel(
        "Time [s]"
    )

    plt.ylabel(
        "Cross-track error [m]"
    )

    plt.title(
        "Watchdog CTE - "
        f"{delay_seconds * 1000:.0f} ms"
    )

    plt.grid()
    plt.legend()

    plt.show()