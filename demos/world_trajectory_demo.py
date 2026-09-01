import math

import matplotlib.pyplot as plt

from coordinate_system.reference_line import (
    ReferenceLine,
)

from planning.constraints import (
    TrajectoryConstraints,
    filter_feasible_trajectories,
)

from planning.cost import (
    TrajectoryCostWeights,
    select_best_trajectory,
)

from planning.trajectory import (
    generate_candidate_trajectories,
)

from planning.world_projection import (
    project_trajectory_to_world,
)


# ============================================================
# 1. Curved road reference line
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


print(
    f"reference line length: "
    f"{reference_line.length:.3f} m"
)


# ============================================================
# 2. Generate lattice candidates
# ============================================================

trajectories = generate_candidate_trajectories(
    s0=0.0,
    s_d0=20.0,
    s_dd0=0.0,

    d0=0.0,
    d_d0=0.0,
    d_dd0=0.0,

    target_d_values=[
        3.2,
        3.5,
        3.8,
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

    dt=0.1,
)


# ============================================================
# 3. Hard constraints
# ============================================================

constraints = TrajectoryConstraints(
    max_speed=30.0,

    max_longitudinal_accel=3.0,
    max_longitudinal_jerk=5.0,

    max_lateral_accel=2.5,
    max_lateral_jerk=8.0,
)


feasible = filter_feasible_trajectories(
    trajectories=trajectories,
    constraints=constraints,
)


# ============================================================
# 4. Cost ranking
# ============================================================

best, best_cost = (
    select_best_trajectory(
        trajectories=feasible,

        desired_d=3.5,
        desired_speed=25.0,

        weights=TrajectoryCostWeights(),
    )
)


# ============================================================
# 5. Frenet -> World
# ============================================================

project_trajectory_to_world(
    trajectory=best,
    reference_line=reference_line,
)


# ============================================================
# 6. Print planner result
# ============================================================

print()
print("PLANNER RESULT")
print("=" * 60)

print(
    f"all candidates : "
    f"{len(trajectories)}"
)

print(
    f"feasible       : "
    f"{len(feasible)}"
)

print(
    f"rejected       : "
    f"{len(trajectories) - len(feasible)}"
)

print()

print(
    f"best target d  : "
    f"{best.target_d:.3f} m"
)

print(
    f"best target v  : "
    f"{best.target_speed:.3f} m/s"
)

print(
    f"best duration  : "
    f"{best.duration:.3f} s"
)

print(
    f"best cost      : "
    f"{best_cost.total:.6f}"
)

print()

print(
    "World start:"
)

print(
    f"x={best.x[0]:.3f}, "
    f"y={best.y[0]:.3f}, "
    f"yaw={math.degrees(best.yaw[0]):.3f} deg"
)

print()

print(
    "World end:"
)

print(
    f"x={best.x[-1]:.3f}, "
    f"y={best.y[-1]:.3f}, "
    f"yaw={math.degrees(best.yaw[-1]):.3f} deg"
)

print()

print(
    "max |curvature|:",
    max(
        abs(kappa)
        for kappa in best.curvature
    ),
)


# ============================================================
# 7. Reference line samples
# ============================================================

reference_s, reference_x, reference_y = (
    reference_line.sample(
        num=500
    )
)


# ============================================================
# 8. Plot
# ============================================================

plt.figure(
    figsize=(10, 6)
)


plt.plot(
    reference_x,
    reference_y,
    linestyle="--",
    label="Reference line",
)


plt.plot(
    best.x,
    best.y,
    linewidth=3.0,
    label="Best trajectory",
)


waypoint_x = [
    point[0]
    for point in reference_path
]

waypoint_y = [
    point[1]
    for point in reference_path
]


plt.scatter(
    waypoint_x,
    waypoint_y,
    label="Reference waypoints",
)


plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "FalconPlan W04 - Best World Trajectory"
)

plt.axis(
    "equal"
)

plt.grid()

plt.legend()

plt.show()


# ============================================================
# 9. Curvature plot
# ============================================================

plt.figure(
    figsize=(10, 5)
)

plt.plot(
    best.s,
    best.curvature,
)

plt.xlabel(
    "Frenet s [m]"
)

plt.ylabel(
    "Curvature [1/m]"
)

plt.title(
    "Best Trajectory Curvature"
)

plt.grid()

plt.show()