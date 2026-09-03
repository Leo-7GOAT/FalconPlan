import math

import matplotlib.pyplot as plt
import numpy as np

from control.control_stack import (
    VehicleControlStack,
)

from control.lateral_controller import (
    LQRLateralController,
    StanleyLateralController,
)

from control.metrics import (
    compute_tracking_metrics,
)

from control.models import (
    ControlTrajectory,
)

from control.pid import PIDController

from control.watchdog import (
    WatchdogConfig,
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
    VehicleParams,
    VehicleState,
)


# ============================================================
# 1. W04 planner
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

planner_target = (
    planning_result.best
)


trajectory = (
    ControlTrajectory
    .from_planner_trajectory(
        planner_target
    )
)


# ============================================================
# 2. Vehicle
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


# ============================================================
# 3. Run exactly the same upper-level code
# ============================================================

def run_controller(
    controller_name: str,
):

    if controller_name == "Stanley":

        lateral_controller = (
            StanleyLateralController(
                k=2.0,
                softening=1.0,

                max_steer=(
                    params.max_steer
                ),
            )
        )

    elif controller_name == "LQR":

        lateral_controller = (
            LQRLateralController(
                q_cross_track=1.0,
                q_heading=1.0,

                r_steer=10.0,

                max_steer=(
                    params.max_steer
                ),
            )
        )

    else:

        raise ValueError(
            controller_name
        )


    longitudinal_controller = (
        PIDController(
            Kp=0.8,
            Ki=0.1,
            Kd=0.05,

            output_min=-params.max_decel,
            output_max=params.max_accel,
        )
    )


    stack = VehicleControlStack(
        trajectory=trajectory,

        vehicle_params=params,

        lateral_controller=(
            lateral_controller
        ),

        longitudinal_controller=(
            longitudinal_controller
        ),

        dt=dt,

        # Baseline benchmark:
        # actuator rate limit enabled,
        # no steering delay.
        steering_delay_seconds=0.0,

        max_steer_rate=math.radians(
            90.0
        ),

        watchdog_config=WatchdogConfig(
            max_cross_track_error=2.5,

            max_heading_error_deg=20.0,

            violation_frames=3,
        ),
    )


    model = KinematicBicycleModel(
        params=params
    )


    initial_offset = 1.5

    initial_yaw = trajectory.yaw[0]


    state = VehicleState(
        x=(
            trajectory.x[0]
            + initial_offset
            * math.sin(initial_yaw)
        ),

        y=(
            trajectory.y[0]
            - initial_offset
            * math.cos(initial_yaw)
        ),

        psi=initial_yaw,

        v=trajectory.speed[0],
    )


    times = []

    vehicle_x = []
    vehicle_y = []

    cte = []
    heading = []
    steering = []


    for step_index in range(
        int(8.0 / dt) + 1
    ):

        t = (
            step_index
            * dt
        )

        # ====================================================
        # THIS IS THE WHOLE CONTROL SYSTEM NOW
        # ====================================================

        result = stack.step(
            state
        )

        # ----------------------------------------------------
        # Diagnostics
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

        cte.append(
            result.cross_track_error
        )

        heading.append(
            result.heading_error
        )

        steering.append(
            result.actual_steering
        )

        # ----------------------------------------------------
        # W04 trajectory complete
        # ----------------------------------------------------

        if result.trajectory_completed:

            break

        # ----------------------------------------------------
        # Apply unified VehicleCommand
        # ----------------------------------------------------

        state = model.step(
            state=state,

            command=result.command,

            dt=dt,

            method="rk4",
        )

        # ----------------------------------------------------
        # Watchdog fail-safe
        # ----------------------------------------------------

        if result.safe_stop:

            print(
                f"{controller_name}: "
                f"WATCHDOG TRIPPED "
                f"at t={t:.2f}s"
            )

            break


    metric_length = min(
        len(cte),
        len(heading),
        len(steering),
    )


    metrics = compute_tracking_metrics(
        cross_track_errors=(
            cte[:metric_length]
        ),

        heading_errors=(
            heading[:metric_length]
        ),

        steering_commands=(
            steering[:metric_length]
        ),

        dt=dt,
    )


    return {
        "name": controller_name,

        "state": state,

        "times": times,

        "x": vehicle_x,
        "y": vehicle_y,

        "cte": cte,

        "metrics": metrics,
    }


# ============================================================
# 4. Controller switch
# ============================================================

stanley = run_controller(
    "Stanley"
)

lqr = run_controller(
    "LQR"
)


# ============================================================
# 5. Result
# ============================================================

print()
print("=" * 76)
print("FALCONPLAN W05 - VEHICLE CONTROL STACK")
print("=" * 76)

print(
    f"{'Metric':<30}"
    f"{'Stanley':>18}"
    f"{'LQR':>18}"
)

print("-" * 76)


def row(
    name,
    s,
    l,
):

    print(
        f"{name:<30}"
        f"{s:>18.6f}"
        f"{l:>18.6f}"
    )


row(
    "CTE RMSE [m]",

    stanley["metrics"]
    .cross_track_rmse,

    lqr["metrics"]
    .cross_track_rmse,
)


row(
    "Max CTE [m]",

    stanley["metrics"]
    .max_cross_track_error,

    lqr["metrics"]
    .max_cross_track_error,
)


row(
    "Max steering [deg]",

    math.degrees(
        stanley["metrics"]
        .max_steering
    ),

    math.degrees(
        lqr["metrics"]
        .max_steering
    ),
)


# ============================================================
# 6. World trajectory
# ============================================================

plt.figure(
    figsize=(11, 7)
)

plt.plot(
    trajectory.x,
    trajectory.y,

    linestyle="--",

    label="W04 trajectory",
)

plt.plot(
    stanley["x"],
    stanley["y"],

    label="Stanley stack",
)

plt.plot(
    lqr["x"],
    lqr["y"],

    label="LQR stack",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W05 - Unified VehicleControlStack"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 7. CTE
# ============================================================

plt.figure()

plt.plot(
    stanley["times"],
    stanley["cte"],

    label="Stanley",
)

plt.plot(
    lqr["times"],
    lqr["cte"],

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
    "Unified Control Stack - CTE"
)

plt.grid()
plt.legend()

plt.show()