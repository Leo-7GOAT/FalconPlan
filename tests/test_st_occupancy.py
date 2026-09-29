import pytest

from planning.st_graph import (
    DynamicObstacle,
    build_constant_velocity_st_boundary,
)
from planning.st_grid import build_st_grid
from planning.st_occupancy import (
    build_blocked_mask,
    interpolate_edge_s,
    is_edge_blocked,
    is_node_blocked,
)


def make_boundary():
    obstacle = DynamicObstacle(
        obstacle_id="suizeier",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    return build_constant_velocity_st_boundary(
        obstacle=obstacle,
        horizon=5.0,
        dt=1.0,
    )


def make_grid():
    return build_st_grid(
        horizon=5.0,
        dt=1.0,
        max_s=80.0,
        ds=10.0,
    )


def test_node_inside_boundary_is_blocked():
    node = make_grid().node(
        time_index=2,
        distance_index=4,
    )
    assert is_node_blocked(
        node=node,
        boundaries=[make_boundary()],
    )


def test_node_below_boundary_is_free():
    node = make_grid().node(
        time_index=2,
        distance_index=3,
    )
    assert not is_node_blocked(
        node=node,
        boundaries=[make_boundary()],
    )


def test_node_above_boundary_is_free():
    node = make_grid().node(
        time_index=2,
        distance_index=5,
    )
    assert not is_node_blocked(
        node=node,
        boundaries=[make_boundary()],
    )


def test_no_boundaries_means_free_node():
    node = make_grid().node(
        time_index=2,
        distance_index=4,
    )
    assert not is_node_blocked(
        node=node,
        boundaries=[],
    )


def test_multiple_boundaries_are_checked():
    obstacle_1 = DynamicObstacle(
        obstacle_id="car_1",
        s0=30.0,
        speed=5.0,
        length=4.0,
        safety_margin=2.0,
    )
    obstacle_2 = DynamicObstacle(
        obstacle_id="car_2",
        s0=60.0,
        speed=0.0,
        length=4.0,
        safety_margin=3.0,
    )
    boundary_1 = build_constant_velocity_st_boundary(
        obstacle=obstacle_1,
        horizon=5.0,
        dt=1.0,
    )
    boundary_2 = build_constant_velocity_st_boundary(
        obstacle=obstacle_2,
        horizon=5.0,
        dt=1.0,
    )
    node = make_grid().node(
        time_index=2,
        distance_index=6,
    )
    assert is_node_blocked(
        node=node,
        boundaries=[boundary_1, boundary_2],
    )


def test_blocked_mask_has_grid_shape():
    grid = make_grid()
    mask = build_blocked_mask(
        grid=grid,
        boundaries=[make_boundary()],
    )
    assert len(mask) == grid.num_time_steps
    assert len(mask[0]) == grid.num_distance_steps
    assert mask[2][4] is True
    assert mask[2][3] is False


def test_interpolate_edge_s_returns_midpoint():
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=2,
    )
    end = grid.node(
        time_index=1,
        distance_index=4,
    )
    assert interpolate_edge_s(
        start=start,
        end=end,
        t=0.5,
    ) == pytest.approx(30.0)


def test_interpolate_edge_s_rejects_time_outside_edge():
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=2,
    )
    end = grid.node(
        time_index=1,
        distance_index=4,
    )
    with pytest.raises(ValueError):
        interpolate_edge_s(
            start=start,
            end=end,
            t=1.5,
        )


def test_edge_crossing_boundary_is_blocked():
    boundary = make_boundary()
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=2,
    )
    end = grid.node(
        time_index=1,
        distance_index=4,
    )
    assert not is_node_blocked(
        node=start,
        boundaries=[boundary],
    )
    assert not is_node_blocked(
        node=end,
        boundaries=[boundary],
    )
    assert is_edge_blocked(
        start=start,
        end=end,
        boundaries=[boundary],
        sample_dt=0.02,
    )


def test_edge_below_boundary_is_free():
    boundary = make_boundary()
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=0,
    )
    end = grid.node(
        time_index=1,
        distance_index=2,
    )
    assert not is_edge_blocked(
        start=start,
        end=end,
        boundaries=[boundary],
        sample_dt=0.02,
    )


def test_edge_with_no_boundaries_is_free():
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=2,
    )
    end = grid.node(
        time_index=1,
        distance_index=4,
    )
    assert not is_edge_blocked(
        start=start,
        end=end,
        boundaries=[],
    )


def test_edge_collision_rejects_invalid_sample_dt():
    grid = make_grid()
    start = grid.node(
        time_index=0,
        distance_index=2,
    )
    end = grid.node(
        time_index=1,
        distance_index=4,
    )
    with pytest.raises(ValueError):
        is_edge_blocked(
            start=start,
            end=end,
            boundaries=[make_boundary()],
            sample_dt=0.0,
        )
