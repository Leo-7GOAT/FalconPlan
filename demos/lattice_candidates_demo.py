import matplotlib.pyplot as plt

from planning.trajectory import (
    generate_candidate_trajectories,
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


print(
    f"Generated {len(trajectories)} trajectories"
)


plt.figure()

for trajectory in trajectories:

    plt.plot(
        trajectory.s,
        trajectory.d,
        alpha=0.5,
    )


plt.xlabel(
    "Longitudinal s [m]"
)

plt.ylabel(
    "Lateral d [m]"
)

plt.title(
    "Frenet Lattice Candidate Trajectories"
)

plt.grid()

plt.show()