import matplotlib.pyplot as plt

from planning.st_graph import (
    DynamicObstacle,
    build_constant_velocity_st_boundary,
)


obstacle = DynamicObstacle(
    obstacle_id="suizeier",
    s0=30.0,
    speed=5.0,
    length=4.0,
    safety_margin=2.0,
)

boundary = build_constant_velocity_st_boundary(
    obstacle=obstacle,
    horizon=5.0,
    dt=0.1,
)

ego_times = boundary.times
ego_s = [
    15.0 * t
    for t in ego_times
]

plt.figure(figsize=(9, 6))
plt.fill_between(
    boundary.times,
    boundary.lower_s,
    boundary.upper_s,
    alpha=0.35,
    label="Dynamic obstacle ST boundary",
)
plt.plot(
    ego_times,
    ego_s,
    linewidth=2.0,
    label="Ego: v = 15 m/s",
)
plt.xlabel("Time t [s]")
plt.ylabel("Path distance s [m]")
plt.title("FalconPlan W06 - ST Graph")
plt.grid()
plt.legend()
plt.show()
