
import matplotlib.pyplot as plt

from planning.speed_corridor import build_speed_corridor
from planning.speed_profile import speed_profile_from_dp_plan

from planning.speed_qp import (
    SpeedQPConstraints,
    SpeedQPWeights,
    build_speed_qp_problem,
    solve_speed_qp,
)

from planning.speed_validation import (
    is_speed_profile_st_safe,
)

from planning.st_dp import (
    DPCostWeights,
    DPSpeedPlanner,
)

from planning.st_graph import (
    DynamicObstacle,
    build_constant_velocity_st_boundary,
)

from planning.st_grid import build_st_grid
from planning.st_transition import STKinematicLimits


def main():

    # ====================================================
    # 1. Scenario
    # ====================================================

    initial_speed = 15.0
    desired_speed = 15.0

    horizon = 6.0
    max_s = 90.0

    max_accel = 3.0
    max_decel = 6.0
    max_jerk = 4.0

    # ====================================================
    # 2. Dynamic obstacle
    # ====================================================

    obstacle = DynamicObstacle(
        obstacle_id="slow_front_vehicle",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )

    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=horizon,
        dt=0.1,
    )

    # ====================================================
    # 3. W06 ST grid
    # ====================================================

    grid = build_st_grid(
        horizon=horizon,
        dt=1.0,
        max_s=max_s,
        ds=5.0,
    )

    limits = STKinematicLimits(
        min_speed=0.0,
        max_speed=25.0,
        max_accel=max_accel,
        max_decel=max_decel,
    )

    # ====================================================
    # 4. W06 DP
    # ====================================================

    dp_planner = DPSpeedPlanner(
        grid=grid,
        boundaries=[boundary],
        limits=limits,
        initial_speed=initial_speed,
        desired_speed=desired_speed,
        weights=DPCostWeights(
            speed_error=1.0,
            acceleration=0.5,
        ),
        edge_sample_dt=0.02,
    )

    dp_plan = dp_planner.plan()

    dp_profile = speed_profile_from_dp_plan(
        plan=dp_plan,
        initial_speed=initial_speed,
        initial_acceleration=0.0,
    )

    # ====================================================
    # 5. W07 convex ST corridor
    #
    # A larger numerical clearance than the initial
    # 0.05 m example is used here because cubic
    # interpolation can deviate between grid nodes.
    # ====================================================

    corridor = build_speed_corridor(
        reference_profile=dp_profile,
        boundaries=[boundary],
        min_s=0.0,
        max_s=max_s,
        boundary_epsilon=0.20,
    )

    # ====================================================
    # 6. W07 QP
    # ====================================================

    qp_constraints = SpeedQPConstraints(
        min_speed=0.0,
        max_speed=25.0,
        max_accel=max_accel,
        max_decel=max_decel,
        max_jerk=max_jerk,
    )

    qp_problem = build_speed_qp_problem(
        reference_profile=dp_profile,
        corridor=corridor,
        constraints=qp_constraints,
        weights=SpeedQPWeights(
            position_tracking=1.0,
            speed_tracking=1.0,
            acceleration=0.3,
            jerk=8.0,
        ),
    )

    qp_solution = solve_speed_qp(
        problem=qp_problem,
        reference_profile=dp_profile,
    )

    qp_profile = qp_solution.profile

    # ====================================================
    # 7. Validation
    # ====================================================

    qp_jerks = [
        (
            qp_profile.points[i + 1].a
            - qp_profile.points[i].a
        )
        /
        (
            qp_profile.points[i + 1].t
            - qp_profile.points[i].t
        )
        for i in range(len(qp_profile.points) - 1)
    ]

    max_abs_jerk = max(abs(j) for j in qp_jerks)

    st_safe = is_speed_profile_st_safe(
        profile=qp_profile,
        boundaries=[boundary],
        sample_dt=0.01,
    )

    max_acc = max(qp_profile.accelerations)
    min_acc = min(qp_profile.accelerations)

    # ====================================================
    # 8. Console report
    # ====================================================

    print()
    print("=" * 90)
    print("FALCONPLAN W07 - DP + QP SPEED OPTIMIZATION")
    print("=" * 90)

    print(
        f"{'t':>6}"
        f"{'DP s':>10}"
        f"{'QP s':>10}"
        f"{'DP v':>10}"
        f"{'QP v':>10}"
        f"{'DP a':>10}"
        f"{'QP a':>10}"
    )

    print("-" * 90)

    for dp, qp in zip(
        dp_profile.points,
        qp_profile.points,
    ):
        print(
            f"{dp.t:>6.2f}"
            f"{dp.s:>10.2f}"
            f"{qp.s:>10.2f}"
            f"{dp.v:>10.2f}"
            f"{qp.v:>10.2f}"
            f"{dp.a:>10.2f}"
            f"{qp.a:>10.2f}"
        )

    print()
    print("OSQP status:", qp_solution.status)
    print("QP objective:", f"{qp_solution.objective_value:.3f}")

    print()
    print("=" * 60)
    print("W07 QP SAFETY ACCEPTANCE")
    print("=" * 60)

    print(f"Max |jerk|       : {max_abs_jerk:.5f} m/s^3")
    print(f"Allowed max jerk : {max_jerk:.5f} m/s^3")
    print(f"Max acceleration : {max_acc:.5f} m/s^2")
    print(f"Min acceleration : {min_acc:.5f} m/s^2")
    print(f"Sampled ST safe  : {st_safe}")

    # Hard assertions
    assert max_abs_jerk <= max_jerk + 1e-4

    assert max_acc <= max_accel + 1e-4
    assert min_acc >= -max_decel - 1e-4

    assert st_safe, (
        "QP path crossed the ST obstacle "
        "between trajectory samples"
    )

    for qp_point, corridor_point in zip(
        qp_profile.points,
        corridor.points,
    ):
        assert (
            corridor_point.lower_s - 1e-4
            <= qp_point.s
            <= corridor_point.upper_s + 1e-4
        )

    print("QP ACCEPTANCE: PASS")

    # ====================================================
    # 9. ST graph
    # ====================================================

    plt.figure(figsize=(10, 6))

    plt.fill_between(
        boundary.times,
        boundary.lower_s,
        boundary.upper_s,
        alpha=0.30,
        label="Dynamic obstacle",
    )

    plt.plot(
        dp_profile.times,
        dp_profile.distances,
        marker="o",
        linestyle="--",
        label="W06 DP",
    )

    plt.plot(
        qp_profile.times,
        qp_profile.distances,
        marker="o",
        linewidth=2.5,
        label="W07 QP",
    )

    plt.plot(
        corridor.times,
        corridor.lower_bounds,
        linestyle=":",
        label="Corridor lower",
    )

    plt.plot(
        corridor.times,
        corridor.upper_bounds,
        linestyle=":",
        label="Corridor upper",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Path progress s [m]")
    plt.title("FalconPlan W07 - DP-QP ST Optimization")
    plt.grid()
    plt.legend()

    # ====================================================
    # 10. Speed comparison
    # ====================================================

    plt.figure(figsize=(10, 5))

    plt.plot(
        dp_profile.times,
        dp_profile.speeds,
        marker="o",
        linestyle="--",
        label="W06 DP speed",
    )

    plt.plot(
        qp_profile.times,
        qp_profile.speeds,
        marker="o",
        linewidth=2.5,
        label="W07 QP speed",
    )

    plt.axhline(
        desired_speed,
        linestyle=":",
        label="Desired speed",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("FalconPlan W07 - DP vs QP Speed")
    plt.grid()
    plt.legend()

    # ====================================================
    # 11. Acceleration comparison
    # ====================================================

    plt.figure(figsize=(10, 5))

    plt.plot(
        dp_profile.times,
        dp_profile.accelerations,
        marker="o",
        linestyle="--",
        label="W06 DP acceleration",
    )

    plt.plot(
        qp_profile.times,
        qp_profile.accelerations,
        marker="o",
        linewidth=2.5,
        label="W07 QP acceleration",
    )

    plt.axhline(
        max_accel,
        linestyle=":",
        label="Max acceleration",
    )

    plt.axhline(
        -max_decel,
        linestyle=":",
        label="Max deceleration",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s^2]")
    plt.title("FalconPlan W07 - DP vs QP Acceleration")
    plt.grid()
    plt.legend()

    from pathlib import Path
    output=Path('artifacts/w07');output.mkdir(parents=True,exist_ok=True)
    for number in plt.get_fignums():
        plt.figure(number).savefig(output/f'dp_qp_{number}.png',dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
