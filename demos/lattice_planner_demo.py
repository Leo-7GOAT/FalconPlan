import matplotlib.pyplot as plt

from coordinate_system.reference_line import ReferenceLine

from planning.collision import (
    CircularObstacle,
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


# ============================================================
# 1. Build reference road
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
# 2. Build W04 Frenet Lattice Planner
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


# ============================================================
# 3. Helper: build planning input
# ============================================================

def make_planning_input(
    obstacles=None,
):

    if obstacles is None:
        obstacles = []

    return FrenetPlanningInput(
        # ----------------------------------------------------
        # Current longitudinal state
        # ----------------------------------------------------

        s0=0.0,
        s_d0=20.0,
        s_dd0=0.0,

        # ----------------------------------------------------
        # Current lateral state
        # ----------------------------------------------------

        d0=0.0,
        d_d0=0.0,
        d_dd0=0.0,

        # ----------------------------------------------------
        # Lane topology
        #
        # Current lane center = 0.0
        # Left lane center    = 3.5
        #
        # Sampling offsets = [-0.5, 0.0, +0.5]
        #
        # Therefore:
        #
        # current lane:
        #   [-0.5, 0.0, 0.5]
        #
        # left lane:
        #   [3.0, 3.5, 4.0]
        # ----------------------------------------------------

        lane_centers=[
            0.0,
            3.5,
        ],

        # ----------------------------------------------------
        # Longitudinal terminal-state sampling
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # Behavior-layer preference
        #
        # W03 wants to change to left lane.
        # ----------------------------------------------------

        desired_d=3.5,
        desired_speed=25.0,

        obstacles=obstacles,
    )


# ============================================================
# 4. Helper: print planner result
# ============================================================

def print_result(
    title,
    result,
):

    print()
    print("=" * 70)
    print(title)
    print("=" * 70)

    print(
        f"candidate count      : "
        f"{result.candidate_count}"
    )

    print(
        f"Frenet feasible      : "
        f"{result.frenet_feasible_count}"
    )

    print(
        f"World feasible       : "
        f"{result.world_feasible_count}"
    )

    print(
        f"collision free       : "
        f"{result.collision_free_count}"
    )

    print()

    print("BEST TRAJECTORY")

    print(
        f"target d             : "
        f"{result.best.target_d:.3f} m"
    )

    print(
        f"target speed         : "
        f"{result.best.target_speed:.3f} m/s"
    )

    print(
        f"duration             : "
        f"{result.best.duration:.3f} s"
    )

    print(
        f"cost                 : "
        f"{result.best_cost.total:.6f}"
    )

    print()

    print(
        f"World start          : "
        f"({result.best.x[0]:.3f}, "
        f"{result.best.y[0]:.3f})"
    )

    print(
        f"World end            : "
        f"({result.best.x[-1]:.3f}, "
        f"{result.best.y[-1]:.3f})"
    )

    max_curvature = max(
        abs(kappa)
        for kappa in result.best.curvature
    )

    print(
        f"max |curvature|      : "
        f"{max_curvature:.6f} 1/m"
    )


# ============================================================
# 5. Scenario 1
#
# No obstacle.
#
# Expected:
# Planner follows W03 behavior preference and selects
# a left-lane trajectory around d = 3.5.
# ============================================================

normal_input = (
    make_planning_input()
)

normal_result = planner.plan(
    normal_input
)

print_result(
    "SCENARIO 1 - NORMAL LEFT LANE CHANGE",
    normal_result,
)


# ============================================================
# 6. Build obstacle on original best trajectory
# ============================================================

original_best = normal_result.best

mid_index = (
    len(original_best.x)
    // 2
)

obstacle = CircularObstacle(
    x=original_best.x[mid_index],
    y=original_best.y[mid_index],
    radius=0.5,
)


# ============================================================
# 7. Scenario 2
#
# Obstacle blocks original best.
#
# Expected:
#
# Original left-lane trajectory is rejected.
# Planner selects a collision-free safe fallback.
# ============================================================

obstacle_input = (
    make_planning_input(
        obstacles=[
            obstacle,
        ]
    )
)

fallback_result = planner.plan(
    obstacle_input
)

print_result(
    "SCENARIO 2 - COLLISION-AWARE SAFE FALLBACK",
    fallback_result,
)


# ============================================================
# 8. Print comparison
# ============================================================

print()
print("=" * 70)
print("PLANNING COMPARISON")
print("=" * 70)

print(
    "Original best:"
)

print(
    f"  dT={normal_result.best.target_d:.1f}, "
    f"vT={normal_result.best.target_speed:.1f}, "
    f"T={normal_result.best.duration:.1f}, "
    f"cost={normal_result.best_cost.total:.6f}"
)

print()

print(
    "Best after obstacle:"
)

print(
    f"  dT={fallback_result.best.target_d:.1f}, "
    f"vT={fallback_result.best.target_speed:.1f}, "
    f"T={fallback_result.best.duration:.1f}, "
    f"cost={fallback_result.best_cost.total:.6f}"
)

print()

print(
    "Obstacle:"
)

print(
    f"  x={obstacle.x:.3f}, "
    f"y={obstacle.y:.3f}, "
    f"radius={obstacle.radius:.3f}"
)


# ============================================================
# 9. Reference line samples
# ============================================================

_, reference_x, reference_y = (
    reference_line.sample(
        num=500
    )
)


# ============================================================
# 10. Plot Scenario 1
# ============================================================

plt.figure(
    figsize=(11, 7)
)

plt.plot(
    reference_x,
    reference_y,
    linestyle="--",
    label="Reference line",
)

plt.plot(
    normal_result.best.x,
    normal_result.best.y,
    linewidth=3.0,
    label="Best trajectory",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W04 - Normal Frenet Lattice Planning"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 11. Plot Scenario 2
#
# Original best + obstacle + safe fallback
# ============================================================

plt.figure(
    figsize=(11, 7)
)

plt.plot(
    reference_x,
    reference_y,
    linestyle="--",
    label="Reference line",
)

# Original trajectory selected before obstacle existed.
plt.plot(
    normal_result.best.x,
    normal_result.best.y,
    linestyle="--",
    linewidth=2.0,
    label="Blocked original best",
)

# Safe fallback selected after collision filtering.
plt.plot(
    fallback_result.best.x,
    fallback_result.best.y,
    linewidth=3.0,
    label="Safe fallback",
)

plt.scatter(
    [obstacle.x],
    [obstacle.y],
    marker="x",
    s=180,
    linewidths=3.0,
    label="Obstacle",
)

plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W04 - Collision-Aware Safe Fallback"
)

plt.axis(
    "equal"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 12. Compare curvature
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    normal_result.best.s,
    normal_result.best.curvature,
    label="Original best",
)

plt.plot(
    fallback_result.best.s,
    fallback_result.best.curvature,
    label="Safe fallback",
)

plt.xlabel(
    "Frenet s [m]"
)

plt.ylabel(
    "Curvature [1/m]"
)

plt.title(
    "FalconPlan W04 - Best Trajectory Curvature"
)

plt.grid()
plt.legend()

plt.show()


# ============================================================
# 13. Final W04 summary
# ============================================================

print()
print("=" * 70)
print("FALCONPLAN W04 DEMO COMPLETE")
print("=" * 70)

print(
    "Pipeline:"
)

print(
    "lane-aware sampling"
    " -> lattice generation"
    " -> Frenet constraints"
    " -> World projection"
    " -> curvature constraints"
    " -> collision filtering"
    " -> cost ranking"
    " -> best trajectory"
)

print()

print(
    "Safety result:"
)

print(
    "Blocked behavior-preferred trajectory "
    "was rejected and a safe fallback was selected."
)