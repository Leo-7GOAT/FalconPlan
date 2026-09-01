from dataclasses import dataclass

import numpy as np

from .trajectory import FrenetTrajectory


@dataclass(frozen=True)
class TrajectoryCostWeights:
    lateral_offset: float = 20.0
    speed_error: float = 1.0

    lateral_jerk: float = 0.05
    longitudinal_jerk: float = 0.05

    duration: float = 0.5


@dataclass(frozen=True)
class TrajectoryCostBreakdown:
    lateral_offset: float
    speed_error: float

    lateral_jerk: float
    longitudinal_jerk: float

    duration: float

    total: float


def compute_trajectory_cost(
    trajectory: FrenetTrajectory,

    desired_d: float,
    desired_speed: float,

    weights: TrajectoryCostWeights,
) -> TrajectoryCostBreakdown:

    if len(trajectory.t) == 0:
        raise ValueError(
            "trajectory must contain samples"
        )

    lateral_offset_cost = (
        trajectory.target_d
        - desired_d
    ) ** 2

    speed_error_cost = (
        trajectory.target_speed
        - desired_speed
    ) ** 2

    lateral_jerk_cost = float(
        np.mean(
            np.square(
                trajectory.d_ddd
            )
        )
    )

    longitudinal_jerk_cost = float(
        np.mean(
            np.square(
                trajectory.s_ddd
            )
        )
    )

    duration_cost = (
        trajectory.duration
    )

    total = (
        weights.lateral_offset
        * lateral_offset_cost

        + weights.speed_error
        * speed_error_cost

        + weights.lateral_jerk
        * lateral_jerk_cost

        + weights.longitudinal_jerk
        * longitudinal_jerk_cost

        + weights.duration
        * duration_cost
    )

    return TrajectoryCostBreakdown(
        lateral_offset=lateral_offset_cost,
        speed_error=speed_error_cost,

        lateral_jerk=lateral_jerk_cost,
        longitudinal_jerk=longitudinal_jerk_cost,

        duration=duration_cost,

        total=total,
    )

def select_best_trajectory(
    trajectories: list[FrenetTrajectory],

    desired_d: float,
    desired_speed: float,

    weights: TrajectoryCostWeights,
) -> tuple[
    FrenetTrajectory,
    TrajectoryCostBreakdown,
]:

    if len(trajectories) == 0:
        raise ValueError(
            "no trajectories available"
        )

    best_trajectory = None
    best_cost = None

    for trajectory in trajectories:

        cost = compute_trajectory_cost(
            trajectory=trajectory,

            desired_d=desired_d,
            desired_speed=desired_speed,

            weights=weights,
        )

        trajectory.cost = cost.total

        if (
            best_cost is None
            or cost.total < best_cost.total
        ):

            best_trajectory = trajectory
            best_cost = cost

    return (
        best_trajectory,
        best_cost,
    )