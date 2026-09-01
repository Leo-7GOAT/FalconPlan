import matplotlib.pyplot as plt

from coordinate_system.reference_line import ReferenceLine

from planning.trajectory import (
    generate_candidate_trajectories,
)

from planning.constraints import (
    TrajectoryConstraints,
    filter_feasible_trajectories,
)

from planning.world_projection import (
    project_trajectory_to_world,
)

from planning.cost import (
    TrajectoryCostWeights,
    select_best_trajectory,
)

from planning.collision import (
    CircularObstacle,
    CollisionParams,
    filter_collision_free_trajectories,
)
from planning.collision import (
    trajectory_min_distance_to_obstacle,
)

# ============================================================
# 1. Reference road
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
# 2. Generate 36 candidates
# ============================================================

all_candidates = generate_candidate_trajectories(
    s0=0.0,
    s_d0=20.0,
    s_dd0=0.0,

    d0=0.0,
    d_d0=0.0,
    d_dd0=0.0,

    target_d_values=[
        -0.5,
        0.0,
        0.5,

        3.0,
        3.5,
        4.0,
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
# 3. Frenet hard constraints
# ============================================================

constraints = TrajectoryConstraints(
    max_speed=30.0,

    max_longitudinal_accel=3.0,
    max_longitudinal_jerk=5.0,

    max_lateral_accel=2.5,
    max_lateral_jerk=8.0,
)

feasible = filter_feasible_trajectories(
    trajectories=all_candidates,
    constraints=constraints,
)


# ============================================================
# 4. Project ALL 22 feasible trajectories to World
# ============================================================

for trajectory in feasible:

    project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )


# ============================================================
# 5. Find original best
# ============================================================

weights = TrajectoryCostWeights()

original_best, original_cost = (
    select_best_trajectory(
        trajectories=feasible,

        desired_d=3.5,
        desired_speed=25.0,

        weights=weights,
    )
)


# ============================================================
# 6. Put obstacle on original best trajectory
# ============================================================

mid_index = len(
    original_best.x
) // 2

obstacle = CircularObstacle(
    x=original_best.x[mid_index],
    y=original_best.y[mid_index],
    radius=0.5,
)

collision_params = CollisionParams(
    vehicle_radius=1.0,
    safety_margin=0.3,
)


# ============================================================
# 7. Remove colliding candidates
# ============================================================

collision_free = (
    filter_collision_free_trajectories(
        trajectories=feasible,

        obstacles=[
            obstacle,
        ],

        params=collision_params,
    )
)


print(
    "all:",
    len(all_candidates),
)

print(
    "Frenet feasible:",
    len(feasible),
)

print(
    "collision free:",
    len(collision_free),
)

collision_distance = (
    collision_params.vehicle_radius
    + obstacle.radius
    + collision_params.safety_margin
)

print()
print(
    "collision threshold:",
    collision_distance,
)

print()
print("CANDIDATE CLEARANCES")
print("=" * 60)

for i, trajectory in enumerate(
    feasible
):

    distance = (
        trajectory_min_distance_to_obstacle(
            trajectory,
            obstacle,
        )
    )

    print(
        f"{i:02d}",
        f"dT={trajectory.target_d:.1f}",
        f"vT={trajectory.target_speed:.1f}",
        f"T={trajectory.duration:.1f}",
        f"clearance={distance:.3f}",
    )

obstacle_s = original_best.s[mid_index]
obstacle_d = original_best.d[mid_index]

print()
print("OBSTACLE FRENET POSITION")
print(
    f"s={obstacle_s:.3f}, "
    f"d={obstacle_d:.3f}"
)

print(
    "approx forbidden d band:",
    f"[{obstacle_d - collision_distance:.3f}, "
    f"{obstacle_d + collision_distance:.3f}]"
)

max_clearance = max(
    trajectory_min_distance_to_obstacle(
        trajectory,
        obstacle,
    )
    for trajectory in feasible
)

print(
    "max candidate clearance:",
    f"{max_clearance:.3f}"
)

if len(collision_free) == 0:
    raise RuntimeError(
        "all trajectories collide"
    )


# ============================================================
# 8. Re-select best
# ============================================================

new_best, new_cost = (
    select_best_trajectory(
        trajectories=collision_free,

        desired_d=3.5,
        desired_speed=25.0,

        weights=weights,
    )
)


print()
print("ORIGINAL BEST")

print(
    f"dT={original_best.target_d:.1f}, "
    f"vT={original_best.target_speed:.1f}, "
    f"T={original_best.duration:.1f}, "
    f"cost={original_cost.total:.6f}"
)


print()
print("NEW BEST")

print(
    f"dT={new_best.target_d:.1f}, "
    f"vT={new_best.target_speed:.1f}, "
    f"T={new_best.duration:.1f}, "
    f"cost={new_cost.total:.6f}"
)


# ============================================================
# 9. Plot
# ============================================================

_, ref_x, ref_y = reference_line.sample(
    num=500
)


plt.figure(
    figsize=(11, 7)
)


plt.plot(
    ref_x,
    ref_y,
    linestyle="--",
    label="Reference line",
)


for trajectory in collision_free:

    plt.plot(
        trajectory.x,
        trajectory.y,
        alpha=0.15,
    )


plt.plot(
    original_best.x,
    original_best.y,
    linestyle="--",
    linewidth=2.0,
    label="Blocked original best",
)


plt.plot(
    new_best.x,
    new_best.y,
    linewidth=3.0,
    label="New best",
)


plt.scatter(
    [obstacle.x],
    [obstacle.y],
    marker="x",
    s=150,
    label="Obstacle",
)


plt.xlabel("World X [m]")
plt.ylabel("World Y [m]")

plt.title(
    "FalconPlan - Static Obstacle Avoidance"
)

plt.axis("equal")
plt.grid()
plt.legend()

plt.show()
