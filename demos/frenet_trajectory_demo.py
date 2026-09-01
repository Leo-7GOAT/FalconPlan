import numpy as np
import matplotlib.pyplot as plt

from planning.polynomial import (
    QuinticPolynomial,
    QuarticPolynomial,
)

from planning.trajectory import (
    generate_frenet_trajectory,
)

from planning.trajectory import (
    generate_candidate_trajectories,
)

T = 3.0

lateral = QuinticPolynomial(
    x0=0.0,
    v0=0.0,
    a0=0.0,
    x1=3.5,
    v1=0.0,
    a1=0.0,
    T=T,
)

longitudinal = QuarticPolynomial(
    x0=0.0,
    v0=20.0,
    a0=0.0,
    v1=25.0,
    a1=0.0,
    T=T,
)


times = np.linspace(
    0.0,
    T,
    61,
)

s_values = [
    longitudinal.position(t)
    for t in times
]

d_values = [
    lateral.position(t)
    for t in times
]


plt.figure()

plt.plot(
    s_values,
    d_values,
)

plt.xlabel("Longitudinal s [m]")
plt.ylabel("Lateral d [m]")
plt.title("Frenet Lane Change Trajectory")

plt.grid()

plt.show()

trajectory = generate_frenet_trajectory(
    s0=0.0,
    s_d0=20.0,
    s_dd0=0.0,

    d0=0.0,
    d_d0=0.0,
    d_dd0=0.0,

    target_d=3.5,
    target_speed=25.0,

    T=3.0,
    dt=0.5,
)

for i in range(len(trajectory.t)):
    print(
        f"t={trajectory.t[i]:.1f}",
        f"s={trajectory.s[i]:.3f}",
        f"v={trajectory.s_d[i]:.3f}",
        f"d={trajectory.d[i]:.3f}",
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
    "number of trajectories:",
    len(trajectories),
)


for i, traj in enumerate(
    trajectories
):
    print(
        f"{i:02d}",
        f"dT={traj.target_d:.1f}",
        f"vT={traj.target_speed:.1f}",
        f"T={traj.duration:.1f}",
        f"s_end={traj.s[-1]:.2f}",
    )