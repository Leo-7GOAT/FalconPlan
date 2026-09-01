import pytest

from planning.collision import (
    point_to_segment_distance,
)
from planning.collision import (
    point_to_segment_distance,
    CircularObstacle,
    CollisionParams,
    trajectory_collides,
)

from planning.trajectory import (
    FrenetTrajectory,
)

def test_point_projects_inside_segment():

    distance = point_to_segment_distance(
        px=5.0,
        py=2.0,

        x1=0.0,
        y1=0.0,

        x2=10.0,
        y2=0.0,
    )

    assert distance == pytest.approx(
        2.0
    )


def test_point_projects_before_segment():

    distance = point_to_segment_distance(
        px=-3.0,
        py=4.0,

        x1=0.0,
        y1=0.0,

        x2=10.0,
        y2=0.0,
    )

    # 最近点应该是 A=(0,0)
    # 距离 = sqrt(3^2 + 4^2) = 5

    assert distance == pytest.approx(
        5.0
    )


def test_point_projects_after_segment():

    distance = point_to_segment_distance(
        px=13.0,
        py=4.0,

        x1=0.0,
        y1=0.0,

        x2=10.0,
        y2=0.0,
    )

    # 最近点应该是 B=(10,0)

    assert distance == pytest.approx(
        5.0
    )


def test_point_on_segment_has_zero_distance():

    distance = point_to_segment_distance(
        px=5.0,
        py=0.0,

        x1=0.0,
        y1=0.0,

        x2=10.0,
        y2=0.0,
    )

    assert distance == pytest.approx(
        0.0
    )


def test_degenerate_segment():

    distance = point_to_segment_distance(
        px=3.0,
        py=4.0,

        x1=0.0,
        y1=0.0,

        x2=0.0,
        y2=0.0,
    )

    assert distance == pytest.approx(
        5.0
    )

def test_obstacle_on_trajectory_collides():

    trajectory = FrenetTrajectory(
        x=[0.0, 5.0, 10.0],
        y=[0.0, 0.0, 0.0],
    )

    obstacle = CircularObstacle(
        x=5.0,
        y=0.0,
        radius=0.5,
    )

    params = CollisionParams(
        vehicle_radius=1.0,
        safety_margin=0.3,
    )

    assert trajectory_collides(
        trajectory,
        [obstacle],
        params,
    )


def test_obstacle_outside_safety_radius():

    trajectory = FrenetTrajectory(
        x=[0.0, 10.0],
        y=[0.0, 0.0],
    )

    obstacle = CircularObstacle(
        x=5.0,
        y=2.0,
        radius=0.5,
    )

    params = CollisionParams(
        vehicle_radius=1.0,
        safety_margin=0.3,
    )

    # collision radius = 1.8 m
    # actual minimum distance = 2.0 m
    assert not trajectory_collides(
        trajectory,
        [obstacle],
        params,
    )


def test_obstacle_inside_safety_radius():

    trajectory = FrenetTrajectory(
        x=[0.0, 10.0],
        y=[0.0, 0.0],
    )

    obstacle = CircularObstacle(
        x=5.0,
        y=1.5,
        radius=0.5,
    )

    params = CollisionParams(
        vehicle_radius=1.0,
        safety_margin=0.3,
    )

    # collision radius = 1.8 m
    # actual minimum distance = 1.5 m
    assert trajectory_collides(
        trajectory,
        [obstacle],
        params,
    )


def test_empty_obstacle_list_is_safe():

    trajectory = FrenetTrajectory(
        x=[0.0, 10.0],
        y=[0.0, 0.0],
    )

    params = CollisionParams(
        vehicle_radius=1.0,
        safety_margin=0.3,
    )

    assert not trajectory_collides(
        trajectory,
        [],
        params,
    )