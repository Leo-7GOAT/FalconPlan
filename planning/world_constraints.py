from dataclasses import dataclass

from .trajectory import FrenetTrajectory


@dataclass(frozen=True)
class WorldTrajectoryConstraints:
    max_curvature: float


def is_world_trajectory_feasible(
    trajectory: FrenetTrajectory,
    constraints: WorldTrajectoryConstraints,
) -> bool:

    if constraints.max_curvature <= 0.0:
        raise ValueError(
            "max_curvature must be positive"
        )

    if len(trajectory.curvature) == 0:
        raise ValueError(
            "trajectory must be projected "
            "to World coordinates first"
        )

    return not any(
        abs(kappa)
        > constraints.max_curvature
        for kappa in trajectory.curvature
    )


def filter_world_feasible_trajectories(
    trajectories: list[FrenetTrajectory],
    constraints: WorldTrajectoryConstraints,
) -> list[FrenetTrajectory]:

    return [
        trajectory
        for trajectory in trajectories
        if is_world_trajectory_feasible(
            trajectory,
            constraints,
        )
    ]
