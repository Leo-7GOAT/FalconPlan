import pytest

from control.models import ControlTrajectory
from planning.time_parameterization import (
    TimeParameterizedTrajectory,
    TimeParameterizedTrajectoryPoint,
)


def make_time_trajectory():
    return TimeParameterizedTrajectory(
        points=(
            TimeParameterizedTrajectoryPoint(
                t=0.0,
                progress_s=0.0,
                path_s=100.0,
                x=0.0,
                y=0.0,
                yaw=0.0,
                curvature=0.01,
                speed=10.0,
                acceleration=0.0,
            ),
            TimeParameterizedTrajectoryPoint(
                t=1.0,
                progress_s=10.0,
                path_s=110.0,
                x=10.0,
                y=1.0,
                yaw=0.1,
                curvature=0.02,
                speed=11.0,
                acceleration=1.0,
            ),
            TimeParameterizedTrajectoryPoint(
                t=2.0,
                progress_s=22.0,
                path_s=122.0,
                x=22.0,
                y=2.0,
                yaw=0.2,
                curvature=0.03,
                speed=12.0,
                acceleration=1.0,
            ),
        )
    )


def test_w06_trajectory_converts_to_control_trajectory():
    control_trajectory = (
        ControlTrajectory
        .from_time_parameterized_trajectory(
            make_time_trajectory()
        )
    )
    assert control_trajectory.x == pytest.approx(
        (0.0, 10.0, 22.0)
    )
    assert control_trajectory.y == pytest.approx(
        (0.0, 1.0, 2.0)
    )
    assert control_trajectory.yaw == pytest.approx(
        (0.0, 0.1, 0.2)
    )
    assert control_trajectory.curvature == pytest.approx(
        (0.01, 0.02, 0.03)
    )
    assert control_trajectory.speed == pytest.approx(
        (10.0, 11.0, 12.0)
    )
    assert control_trajectory.acceleration == pytest.approx(
        (0.0, 1.0, 1.0)
    )


def test_w06_control_trajectory_lengths_match():
    control_trajectory = (
        ControlTrajectory
        .from_time_parameterized_trajectory(
            make_time_trajectory()
        )
    )
    assert len(control_trajectory) == 3
    assert len(control_trajectory.x) == len(
        control_trajectory.speed
    )
    assert len(control_trajectory.x) == len(
        control_trajectory.acceleration
    )
