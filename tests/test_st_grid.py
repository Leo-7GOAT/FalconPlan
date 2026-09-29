import pytest

from planning.st_grid import STGridNode, build_st_grid


def test_build_st_grid_creates_expected_samples():
    grid = build_st_grid(
        horizon=5.0,
        dt=1.0,
        max_s=80.0,
        ds=10.0,
    )
    assert grid.times == (
        0.0, 1.0, 2.0, 3.0, 4.0, 5.0
    )
    assert grid.distances == (
        0.0, 10.0, 20.0, 30.0, 40.0,
        50.0, 60.0, 70.0, 80.0
    )


def test_st_grid_shape_is_correct():
    grid = build_st_grid(
        horizon=5.0,
        dt=1.0,
        max_s=80.0,
        ds=10.0,
    )
    assert grid.num_time_steps == 6
    assert grid.num_distance_steps == 9
    assert grid.shape == (6, 9)


def test_st_grid_returns_correct_node():
    grid = build_st_grid(
        horizon=5.0,
        dt=1.0,
        max_s=80.0,
        ds=10.0,
    )
    node = grid.node(
        time_index=2,
        distance_index=3,
    )
    assert isinstance(node, STGridNode)
    assert node.time_index == 2
    assert node.distance_index == 3
    assert node.t == pytest.approx(2.0)
    assert node.s == pytest.approx(30.0)


def test_st_grid_rejects_invalid_node_index():
    grid = build_st_grid(
        horizon=5.0,
        dt=1.0,
        max_s=80.0,
        ds=10.0,
    )
    with pytest.raises(IndexError):
        grid.node(
            time_index=6,
            distance_index=0,
        )
    with pytest.raises(IndexError):
        grid.node(
            time_index=0,
            distance_index=9,
        )


def test_st_grid_rejects_invalid_dt():
    with pytest.raises(ValueError):
        build_st_grid(
            horizon=5.0,
            dt=0.0,
            max_s=80.0,
            ds=10.0,
        )


def test_st_grid_rejects_invalid_ds():
    with pytest.raises(ValueError):
        build_st_grid(
            horizon=5.0,
            dt=1.0,
            max_s=80.0,
            ds=0.0,
        )
