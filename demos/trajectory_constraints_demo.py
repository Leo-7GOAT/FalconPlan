import matplotlib.pyplot as plt

from planning.trajectory import (
    generate_candidate_trajectories,
)

from planning.constraints import (
    TrajectoryConstraints,
    filter_feasible_trajectories,
)


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


constraints = TrajectoryConstraints(
    max_speed=30.0,

    max_longitudinal_accel=3.0,
    max_longitudinal_jerk=5.0,

    max_lateral_accel=2.5,
    max_lateral_jerk=8.0,
)


feasible = filter_feasible_trajectories(
    trajectories,
    constraints,
)


print(
    "all:",
    len(trajectories),
)

print(
    "feasible:",
    len(feasible),
)

print(
    "rejected:",
    len(trajectories) - len(feasible),
)


plt.figure()

for trajectory in feasible:

    plt.plot(
        trajectory.s,
        trajectory.d,
        alpha=0.6,
    )

plt.xlabel(
    "Longitudinal s [m]"
)

plt.ylabel(
    "Lateral d [m]"
)

plt.title(
    "Feasible Frenet Trajectories"
)

plt.grid()



from planning.cost import (
    TrajectoryCostWeights,
    select_best_trajectory,
)


weights = TrajectoryCostWeights()

best, best_cost = (
    select_best_trajectory(
        trajectories=feasible,

        desired_d=3.5,
        desired_speed=25.0,

        weights=weights,
    )
)


print()
print("BEST TRAJECTORY")
print(
    f"dT={best.target_d:.1f}"
)
print(
    f"vT={best.target_speed:.1f}"
)
print(
    f"T={best.duration:.1f}"
)
print(
    f"cost={best_cost.total:.6f}"
)

print()
print("COST BREAKDOWN")
print(best_cost)


plt.show()
for trajectory in feasible:

    plt.plot(
        trajectory.s,
        trajectory.d,
        alpha=0.25,
    )

plt.plot(
    best.s,
    best.d,
    linewidth=30.0,
    label="Best trajectory",
)

plt.legend()