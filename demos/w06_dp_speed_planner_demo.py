import matplotlib.pyplot as plt

from planning.speed_profile import speed_profile_from_dp_plan
from planning.st_dp import DPCostWeights, DPSpeedPlanner
from planning.st_graph import (
    DynamicObstacle,
    build_constant_velocity_st_boundary,
)
from planning.st_grid import build_st_grid
from planning.st_transition import STKinematicLimits


initial_speed = 15.0
desired_speed = 15.0
horizon = 6.0
grid_dt = 1.0
grid_ds = 5.0
max_s = 90.0

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

grid = build_st_grid(
    horizon=horizon,
    dt=grid_dt,
    max_s=max_s,
    ds=grid_ds,
)

limits = STKinematicLimits(
    min_speed=0.0,
    max_speed=25.0,
    max_accel=3.0,
    max_decel=6.0,
)

planner = DPSpeedPlanner(
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

plan = planner.plan()

profile = speed_profile_from_dp_plan(
    plan=plan,
    initial_speed=initial_speed,
    initial_acceleration=0.0,
)

print()
print("=" * 82)
print("FALCONPLAN W06 - DP SPEED PLANNER")
print("=" * 82)
print(
    f"{'t [s]':>8}"
    f"{'s [m]':>10}"
    f"{'v [m/s]':>12}"
    f"{'a [m/s^2]':>14}"
)
print("-" * 82)

for point in profile.points:
    print(
        f"{point.t:>8.2f}"
        f"{point.s:>10.2f}"
        f"{point.v:>12.2f}"
        f"{point.a:>14.2f}"
    )

print()
print(
    f"DP total cost: "
    f"{plan.total_cost:.3f}"
)

baseline_t = list(boundary.times)
baseline_s = [
    initial_speed * t
    for t in baseline_t
]

plt.figure(figsize=(10, 6))
plt.fill_between(
    boundary.times,
    boundary.lower_s,
    boundary.upper_s,
    alpha=0.35,
    label="Dynamic obstacle ST boundary",
)
plt.plot(
    baseline_t,
    baseline_s,
    linestyle="--",
    linewidth=2.0,
    label="Baseline: 15 m/s constant speed",
)
plt.plot(
    profile.times,
    profile.distances,
    marker="o",
    linewidth=2.5,
    label="DP optimal s(t)",
)
plt.xlabel("Time t [s]")
plt.ylabel("Path progress s [m]")
plt.title(
    "FalconPlan W06 - ST Dynamic Programming"
)
plt.grid()
plt.legend()
plt.show()

plt.figure(figsize=(10, 5))
plt.plot(
    profile.times,
    profile.speeds,
    marker="o",
    linewidth=2.0,
)
plt.axhline(
    desired_speed,
    linestyle="--",
    label="Desired speed",
)
plt.xlabel("Time t [s]")
plt.ylabel("Speed v [m/s]")
plt.title(
    "FalconPlan W06 - DP Speed Profile"
)
plt.grid()
plt.legend()
plt.show()

plt.figure(figsize=(10, 5))
plt.plot(
    profile.times,
    profile.accelerations,
    marker="o",
    linewidth=2.0,
)
plt.axhline(
    limits.max_accel,
    linestyle="--",
    label="Max acceleration",
)
plt.axhline(
    -limits.max_decel,
    linestyle="--",
    label="Max deceleration",
)
plt.xlabel("Time t [s]")
plt.ylabel("Acceleration a [m/s²]")
plt.title(
    "FalconPlan W06 - Acceleration Profile"
)
plt.grid()
plt.legend()
plt.show()
