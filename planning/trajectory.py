from dataclasses import dataclass, field

import numpy as np

from .polynomial import (
    QuinticPolynomial,
    QuarticPolynomial,
)

@dataclass
class FrenetTrajectory:
    # ========================================================
    # Time
    # ========================================================

    t: list[float] = field(default_factory=list)

    # ========================================================
    # Frenet longitudinal
    # ========================================================

    s: list[float] = field(default_factory=list)
    s_d: list[float] = field(default_factory=list)
    s_dd: list[float] = field(default_factory=list)
    s_ddd: list[float] = field(default_factory=list)

    # ========================================================
    # Frenet lateral
    # ========================================================

    d: list[float] = field(default_factory=list)
    d_d: list[float] = field(default_factory=list)
    d_dd: list[float] = field(default_factory=list)
    d_ddd: list[float] = field(default_factory=list)

    # ========================================================
    # World trajectory
    # ========================================================

    x: list[float] = field(default_factory=list)
    y: list[float] = field(default_factory=list)

    yaw: list[float] = field(default_factory=list)

    curvature: list[float] = field(
        default_factory=list
    )

    # ========================================================
    # Candidate metadata
    # ========================================================

    target_d: float = 0.0
    target_speed: float = 0.0
    duration: float = 0.0

    cost: float = 0.0

def generate_frenet_trajectory(
    s0: float,
    s_d0: float,
    s_dd0: float,

    d0: float,
    d_d0: float,
    d_dd0: float,

    target_d: float,
    target_speed: float,

    T: float,
    dt: float,
) -> FrenetTrajectory:
    lateral_poly = QuinticPolynomial(
        x0=d0,
        v0=d_d0,
        a0=d_dd0,

        x1=target_d,
        v1=0.0,
        a1=0.0,

        T=T,
    )
    longitudinal_poly = QuarticPolynomial(
        x0=s0,
        v0=s_d0,
        a0=s_dd0,

        v1=target_speed,
        a1=0.0,

        T=T,
    )

    times = np.arange(
        0.0,
        T + 0.5 * dt,
        dt,
    )

    trajectory = FrenetTrajectory()
    for t in times:
        trajectory.t.append(float(t))

        trajectory.s.append(
            longitudinal_poly.position(t)
        )

        trajectory.s_d.append(
            longitudinal_poly.velocity(t)
        )

        trajectory.s_dd.append(
            longitudinal_poly.acceleration(t)
        )

        trajectory.s_ddd.append(
            longitudinal_poly.jerk(t)
        )

        trajectory.d.append(
            lateral_poly.position(t)
        )

        trajectory.d_d.append(
            lateral_poly.velocity(t)
        )

        trajectory.d_dd.append(
            lateral_poly.acceleration(t)
        )

        trajectory.d_ddd.append(
            lateral_poly.jerk(t)
        )

        trajectory.target_d = target_d
        trajectory.target_speed = target_speed
        trajectory.duration = T

    return trajectory

def generate_candidate_trajectories(
    s0: float,
    s_d0: float,
    s_dd0: float,

    d0: float,
    d_d0: float,
    d_dd0: float,

    target_d_values: list[float],
    target_speed_values: list[float],
    duration_values: list[float],

    dt: float,
) -> list[FrenetTrajectory]:

    trajectories = []

    for target_d in target_d_values:

        for T in duration_values:

            for target_speed in target_speed_values:

                trajectory = generate_frenet_trajectory(
                    s0=s0,
                    s_d0=s_d0,
                    s_dd0=s_dd0,

                    d0=d0,
                    d_d0=d_d0,
                    d_dd0=d_dd0,

                    target_d=target_d,
                    target_speed=target_speed,

                    T=T,
                    dt=dt,
                )

                trajectories.append(
                    trajectory
                )

    return trajectories

