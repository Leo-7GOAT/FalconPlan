import pytest

from planning.st_graph import (
    DynamicObstacle,
    STBoundary,
    STPoint,
    build_constant_velocity_st_boundary,
)


def test_st_point_stores_time_and_distance():
    point = STPoint(t=2.0, s=30.0)
    assert point.t == 2.0
    assert point.s == 30.0


def test_st_boundary_can_be_created():
    boundary = STBoundary(
        obstacle_id="suizeier",
        times=(0.0, 1.0, 2.0, 3.0, 4.0),
        lower_s=(26.0, 31.0, 36.0, 41.0, 46.0),
        upper_s=(34.0, 39.0, 44.0, 49.0, 54.0),
    )
    assert boundary.obstacle_id == "suizeier"
    assert len(boundary.times) == 5
    assert boundary.lower_s[2] == 36.0
    assert boundary.upper_s[2] == 44.0


def test_st_boundary_rejects_mismatched_lengths():
    with pytest.raises(ValueError):
        STBoundary(
            obstacle_id="bad_obstacle",
            times=(0.0, 1.0, 2.0),
            lower_s=(10.0, 15.0),
            upper_s=(20.0, 25.0, 30.0),
        )


def test_st_boundary_rejects_invalid_lower_upper():
    with pytest.raises(ValueError):
        STBoundary(
            obstacle_id="bad_obstacle",
            times=(0.0, 1.0, 2.0),
            lower_s=(10.0, 30.0, 20.0),
            upper_s=(15.0, 25.0, 30.0),
        )


def test_dynamic_obstacle_builds_expected_st_boundary():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=4.0,
        dt=1.0,
    )
    assert boundary.times == (
        0.0, 1.0, 2.0, 3.0, 4.0
    )
    assert boundary.lower_s == (
        26.0, 31.0, 36.0, 41.0, 46.0
    )
    assert boundary.upper_s == (
        34.0, 39.0, 44.0, 49.0, 54.0
    )


def test_dynamic_obstacle_boundary_contains_horizon():
    obstacle = DynamicObstacle(
        obstacle_id="car",
        s0=10.0,
        speed=2.0,
        length=4.0,
        safety_margin=1.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=2.0,
        dt=0.5,
    )
    assert boundary.times[-1] == pytest.approx(2.0)
    assert len(boundary.times) == 5


def test_dynamic_obstacle_rejects_invalid_length():
    with pytest.raises(ValueError):
        DynamicObstacle(
            obstacle_id="bad_car",
            s0=10.0,
            speed=5.0,
            length=0.0,
            safety_margin=1.0,
        )


def test_dynamic_obstacle_rejects_negative_margin():
    with pytest.raises(ValueError):
        DynamicObstacle(
            obstacle_id="bad_car",
            s0=10.0,
            speed=5.0,
            length=4.0,
            safety_margin=-1.0,
        )


def test_st_boundary_builder_rejects_invalid_dt():
    obstacle = DynamicObstacle(
        obstacle_id="car",
        s0=10.0,
        speed=5.0,
        length=4.0,
        safety_margin=1.0,
    )
    with pytest.raises(ValueError):
        build_constant_velocity_st_boundary(
            obstacle=obstacle,
            horizon=4.0,
            dt=0.0,
        )


def test_boundary_at_exact_sample():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=4.0,
        dt=1.0,
    )
    lower, upper = boundary.boundary_at(2.0)
    assert lower == pytest.approx(36.0)
    assert upper == pytest.approx(44.0)


def test_boundary_at_interpolates_between_samples():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=4.0,
        dt=1.0,
    )
    lower, upper = boundary.boundary_at(2.35)
    assert lower == pytest.approx(37.75)
    assert upper == pytest.approx(45.75)


def test_boundary_at_last_sample():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=4.0,
        dt=1.0,
    )
    lower, upper = boundary.boundary_at(4.0)
    assert lower == pytest.approx(46.0)
    assert upper == pytest.approx(54.0)


def test_boundary_at_outside_horizon_returns_none():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    boundary = build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=4.0,
        dt=1.0,
    )
    assert boundary.boundary_at(-0.1) is None
    assert boundary.boundary_at(4.1) is None
