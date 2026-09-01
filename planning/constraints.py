from dataclasses import dataclass

from .trajectory import FrenetTrajectory


@dataclass(frozen=True)
class TrajectoryConstraints:
    max_speed: float

    max_longitudinal_accel: float
    max_longitudinal_jerk: float

    max_lateral_accel: float
    max_lateral_jerk: float

@dataclass(frozen=True)
class ConstraintCheckResult:
    feasible: bool
    reason: str | None = None

def is_trajectory_feasible(
    trajectory: FrenetTrajectory,
    constraints: TrajectoryConstraints,
) -> bool:

    # --------------------------------------------------------
    # Speed
    # --------------------------------------------------------

    if any(
        speed < 0.0
        or speed > constraints.max_speed
        for speed in trajectory.s_d
    ):
        return False

    # --------------------------------------------------------
    # Longitudinal acceleration
    # --------------------------------------------------------

    if any(
        abs(accel)
        > constraints.max_longitudinal_accel
        for accel in trajectory.s_dd
    ):
        return False

    # --------------------------------------------------------
    # Longitudinal jerk
    # --------------------------------------------------------

    if any(
        abs(jerk)
        > constraints.max_longitudinal_jerk
        for jerk in trajectory.s_ddd
    ):
        return False

    # --------------------------------------------------------
    # Lateral acceleration
    # --------------------------------------------------------

    if any(
        abs(accel)
        > constraints.max_lateral_accel
        for accel in trajectory.d_dd
    ):
        return False

    # --------------------------------------------------------
    # Lateral jerk
    # --------------------------------------------------------

    if any(
        abs(jerk)
        > constraints.max_lateral_jerk
        for jerk in trajectory.d_ddd
    ):
        return False

    return True

def filter_feasible_trajectories(
    trajectories: list[FrenetTrajectory],
    constraints: TrajectoryConstraints,
) -> list[FrenetTrajectory]:

    return [
        trajectory
        for trajectory in trajectories
        if is_trajectory_feasible(
            trajectory,
            constraints,
        )
    ]
