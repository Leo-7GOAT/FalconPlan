import pytest

from planning.st_dp import (
    DPCostWeights,
    DPSpeedPlanner,
    DPStateKey,
)
from planning.st_graph import STBoundary
from planning.st_grid import build_st_grid
from planning.st_occupancy import (
    is_edge_blocked,
    is_node_blocked,
)
from planning.st_transition import STKinematicLimits


def make_grid():
    return build_st_grid(
        horizon=3.0,
        dt=1.0,
        max_s=40.0,
        ds=10.0,
    )


def make_limits():
    return STKinematicLimits(
        min_speed=0.0,
        max_speed=20.0,
        max_accel=20.0,
        max_decel=20.0,
    )


def test_dp_state_distinguishes_incoming_history():
    state_a = DPStateKey(
        time_index=2,
        previous_distance_index=0,
        distance_index=2,
    )
    state_b = DPStateKey(
        time_index=2,
        previous_distance_index=1,
        distance_index=2,
    )
    assert state_a != state_b


def test_dp_without_obstacle_tracks_desired_speed():
    planner = DPSpeedPlanner(
        grid=make_grid(),
        boundaries=[],
        limits=make_limits(),
        initial_speed=10.0,
        desired_speed=10.0,
        weights=DPCostWeights(
            speed_error=1.0,
            acceleration=0.2,
        ),
    )
    plan = planner.plan()

    assert tuple(
        node.s for node in plan.nodes
    ) == (
        0.0, 10.0, 20.0, 30.0
    )
    assert plan.speeds == pytest.approx(
        (10.0, 10.0, 10.0)
    )
    assert plan.accelerations == pytest.approx(
        (0.0, 0.0)
    )
    assert plan.total_cost == pytest.approx(0.0)


def test_dp_respects_speed_limit():
    limits = STKinematicLimits(
        min_speed=0.0,
        max_speed=10.0,
        max_accel=20.0,
        max_decel=20.0,
    )
    planner = DPSpeedPlanner(
        grid=make_grid(),
        boundaries=[],
        limits=limits,
        initial_speed=10.0,
        desired_speed=20.0,
    )
    plan = planner.plan()
    assert all(
        speed <= 10.0 + 1e-12
        for speed in plan.speeds
    )


def test_dp_avoids_static_st_obstacle():
    grid = make_grid()
    obstacle = STBoundary(
        obstacle_id="wall",
        times=(0.0, 1.0, 2.0, 3.0),
        lower_s=(12.0, 12.0, 12.0, 12.0),
        upper_s=(18.0, 18.0, 18.0, 18.0),
    )

    planner = DPSpeedPlanner(
        grid=grid,
        boundaries=[obstacle],
        limits=make_limits(),
        initial_speed=10.0,
        desired_speed=10.0,
        edge_sample_dt=0.02,
    )
    plan = planner.plan()

    for node in plan.nodes:
        assert not is_node_blocked(
            node=node,
            boundaries=[obstacle],
        )

    for i in range(len(plan.nodes) - 1):
        assert not is_edge_blocked(
            start=plan.nodes[i],
            end=plan.nodes[i + 1],
            boundaries=[obstacle],
            sample_dt=0.02,
        )


def test_dp_raises_when_first_column_is_infeasible():
    obstacle = STBoundary(
        obstacle_id="full_block",
        times=(1.0, 2.0, 3.0),
        lower_s=(0.0, 0.0, 0.0),
        upper_s=(40.0, 40.0, 40.0),
    )

    planner = DPSpeedPlanner(
        grid=make_grid(),
        boundaries=[obstacle],
        limits=make_limits(),
        initial_speed=10.0,
        desired_speed=10.0,
    )

    with pytest.raises(RuntimeError):
        planner.plan()


def test_dp_rejects_negative_desired_speed():
    with pytest.raises(ValueError):
        DPSpeedPlanner(
            grid=make_grid(),
            boundaries=[],
            limits=make_limits(),
            initial_speed=10.0,
            desired_speed=-1.0,
        )


def test_dp_respects_initial_speed_acceleration():
    grid = build_st_grid(
        horizon=2.0,
        dt=1.0,
        max_s=40.0,
        ds=10.0,
    )
    limits = STKinematicLimits(
        min_speed=0.0,
        max_speed=20.0,
        max_accel=2.0,
        max_decel=2.0,
    )
    planner = DPSpeedPlanner(
        grid=grid,
        boundaries=[],
        limits=limits,
        initial_speed=20.0,
        desired_speed=0.0,
    )
    plan = planner.plan()
    assert plan.speeds[0] == pytest.approx(20.0)
