import math

import pytest

from planning.speed_profile import (
    SpeedProfile,
    SpeedProfilePoint,
)
from planning.time_parameterization import (
    parameterize_geometry_with_speed,
)
from planning.trajectory import FrenetTrajectory


def make_geometry():
    trajectory = FrenetTrajectory()
    trajectory.s = [
        100.0, 110.0, 120.0, 130.0, 140.0
    ]
    trajectory.x = [
        0.0, 10.0, 20.0, 30.0, 40.0
    ]
    trajectory.y = [
        0.0, 1.0, 2.0, 3.0, 4.0
    ]
    trajectory.yaw = [
        0.0, 0.1, 0.2, 0.3, 0.4
    ]
    trajectory.curvature = [
        0.00, 0.01, 0.02, 0.03, 0.04
    ]
    return trajectory


def test_parameterization_preserves_speed_profile():
    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 0.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 10.0, 11.0, 1.0),
            SpeedProfilePoint(2.0, 20.0, 12.0, 1.0),
        )
    )
    result = parameterize_geometry_with_speed(
        geometry=make_geometry(),
        speed_profile=profile,
    )
    assert result.times == pytest.approx(
        (0.0, 1.0, 2.0)
    )
    assert result.speed == pytest.approx(
        (10.0, 11.0, 12.0)
    )
    assert result.acceleration == pytest.approx(
        (0.0, 1.0, 1.0)
    )


def test_parameterization_converts_relative_progress_to_absolute_s():
    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 0.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 20.0, 10.0, 0.0),
        )
    )
    result = parameterize_geometry_with_speed(
        geometry=make_geometry(),
        speed_profile=profile,
    )
    assert result.points[0].path_s == pytest.approx(
        100.0
    )
    assert result.points[1].path_s == pytest.approx(
        120.0
    )


def test_parameterization_interpolates_geometry():
    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 5.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 15.0, 10.0, 0.0),
        )
    )
    result = parameterize_geometry_with_speed(
        geometry=make_geometry(),
        speed_profile=profile,
    )
    first = result.points[0]
    assert first.path_s == pytest.approx(105.0)
    assert first.x == pytest.approx(5.0)
    assert first.y == pytest.approx(0.5)
    assert first.yaw == pytest.approx(0.05)
    assert first.curvature == pytest.approx(0.005)


def test_parameterization_interpolates_wrapped_yaw():
    geometry = FrenetTrajectory()
    geometry.s = [0.0, 10.0]
    geometry.x = [0.0, 10.0]
    geometry.y = [0.0, 0.0]
    geometry.yaw = [
        math.radians(179.0),
        math.radians(-179.0),
    ]
    geometry.curvature = [0.0, 0.0]

    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 5.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 10.0, 10.0, 0.0),
        )
    )
    result = parameterize_geometry_with_speed(
        geometry=geometry,
        speed_profile=profile,
    )
    midpoint_yaw = result.points[0].yaw
    assert abs(
        abs(math.degrees(midpoint_yaw))
        - 180.0
    ) < 1e-6


def test_parameterization_rejects_profile_beyond_geometry():
    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 0.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 50.0, 10.0, 0.0),
        )
    )
    with pytest.raises(ValueError):
        parameterize_geometry_with_speed(
            geometry=make_geometry(),
            speed_profile=profile,
        )


def test_parameterization_rejects_mismatched_geometry_lengths():
    geometry = make_geometry()
    geometry.y.pop()

    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(0.0, 0.0, 10.0, 0.0),
            SpeedProfilePoint(1.0, 10.0, 10.0, 0.0),
        )
    )
    with pytest.raises(ValueError):
        parameterize_geometry_with_speed(
            geometry=geometry,
            speed_profile=profile,
        )
