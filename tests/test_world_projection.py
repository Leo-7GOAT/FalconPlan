import math

import pytest

from coordinate_system.reference_line import (
    ReferenceLine,
)

from planning.trajectory import (
    FrenetTrajectory,
    generate_frenet_trajectory,
)

from planning.world_projection import (
    project_trajectory_to_world,
)


def make_straight_reference_line():

    return ReferenceLine(
        reference_path=[
            (0.0, 0.0),
            (40.0, 0.0),
            (80.0, 0.0),
            (120.0, 0.0),
        ]
    )


def make_lane_change_trajectory():

    return generate_frenet_trajectory(
        s0=0.0,
        s_d0=20.0,
        s_dd0=0.0,

        d0=0.0,
        d_d0=0.0,
        d_dd0=0.0,

        target_d=3.5,
        target_speed=25.0,

        T=3.0,
        dt=0.1,
    )


def test_straight_reference_world_position_matches_frenet():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = (
        make_lane_change_trajectory()
    )

    result = project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    assert len(result.x) == len(
        result.t
    )

    assert len(result.y) == len(
        result.t
    )

    assert len(result.yaw) == len(
        result.t
    )

    assert len(result.curvature) == len(
        result.t
    )

    # Straight road:
    #
    # theta_r = 0
    #
    # therefore
    #
    # x = s
    # y = d

    for x, s in zip(
        result.x,
        result.s,
    ):
        assert x == pytest.approx(
            s,
            abs=1e-5,
        )

    for y, d in zip(
        result.y,
        result.d,
    ):
        assert y == pytest.approx(
            d,
            abs=1e-5,
        )


def test_world_yaw_uses_d_prime():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = (
        make_lane_change_trajectory()
    )

    result = project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    # Pick a point during the lane change.
    i = len(result.t) // 2

    d_prime = (
        result.d_d[i]
        / result.s_d[i]
    )

    # On a straight reference line:
    #
    # kappa_r = 0
    # theta_r = 0
    #
    # therefore:
    #
    # yaw = atan2(d_prime, 1)

    expected_yaw = math.atan2(
        d_prime,
        1.0,
    )

    assert result.yaw[i] == pytest.approx(
        expected_yaw,
        abs=1e-6,
    )

    # Start/end lateral velocity are zero,
    # so yaw aligns with the straight reference line.

    assert result.yaw[0] == pytest.approx(
        0.0,
        abs=1e-8,
    )

    assert result.yaw[-1] == pytest.approx(
        0.0,
        abs=1e-8,
    )


def test_lane_change_has_nonzero_curvature():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = (
        make_lane_change_trajectory()
    )

    result = project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    max_curvature = max(
        abs(kappa)
        for kappa in result.curvature
    )

    assert max_curvature > 0.0


def test_projection_does_not_duplicate_world_samples():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = (
        make_lane_change_trajectory()
    )

    project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    first_length = len(
        trajectory.x
    )

    project_trajectory_to_world(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    second_length = len(
        trajectory.x
    )

    assert first_length == len(
        trajectory.t
    )

    assert second_length == len(
        trajectory.t
    )


def test_zero_longitudinal_speed_rejected():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = FrenetTrajectory(
        t=[0.0],

        s=[0.0],
        s_d=[0.0],
        s_dd=[0.0],
        s_ddd=[0.0],

        d=[0.0],
        d_d=[0.0],
        d_dd=[0.0],
        d_ddd=[0.0],
    )

    with pytest.raises(
        ValueError,
        match="longitudinal speed",
    ):

        project_trajectory_to_world(
            trajectory=trajectory,
            reference_line=reference_line,
        )


def test_reference_line_range_is_enforced():

    reference_line = (
        make_straight_reference_line()
    )

    trajectory = FrenetTrajectory(
        t=[0.0],

        # Outside reference line length.
        s=[130.0],

        s_d=[20.0],
        s_dd=[0.0],
        s_ddd=[0.0],

        d=[0.0],
        d_d=[0.0],
        d_dd=[0.0],
        d_ddd=[0.0],
    )

    with pytest.raises(ValueError):

        project_trajectory_to_world(
            trajectory=trajectory,
            reference_line=reference_line,
        )