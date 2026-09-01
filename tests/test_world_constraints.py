import pytest

from planning.trajectory import (
    FrenetTrajectory,
)

from planning.world_constraints import (
    WorldTrajectoryConstraints,
    is_world_trajectory_feasible,
)


def test_valid_world_trajectory():

    trajectory = FrenetTrajectory(
        curvature=[
            0.005,
            0.010,
            -0.020,
        ]
    )

    constraints = WorldTrajectoryConstraints(
        max_curvature=0.03,
    )

    assert is_world_trajectory_feasible(
        trajectory,
        constraints,
    )


def test_excessive_curvature_rejected():

    trajectory = FrenetTrajectory(
        curvature=[
            0.01,
            0.04,
        ]
    )

    constraints = WorldTrajectoryConstraints(
        max_curvature=0.03,
    )

    assert not is_world_trajectory_feasible(
        trajectory,
        constraints,
    )


def test_world_projection_required():

    trajectory = FrenetTrajectory()

    constraints = WorldTrajectoryConstraints(
        max_curvature=0.03,
    )

    with pytest.raises(ValueError):

        is_world_trajectory_feasible(
            trajectory,
            constraints,
        )


def test_invalid_curvature_limit_rejected():

    trajectory = FrenetTrajectory(
        curvature=[0.01]
    )

    constraints = WorldTrajectoryConstraints(
        max_curvature=0.0,
    )

    with pytest.raises(ValueError):

        is_world_trajectory_feasible(
            trajectory,
            constraints,
        )