import pytest

from planning.st_grid import STGridNode
from planning.st_transition import (
    STKinematicLimits,
    edge_speed,
    initial_edge_acceleration,
    is_edge_speed_feasible,
    is_initial_edge_acceleration_feasible,
    is_transition_acceleration_feasible,
    is_transition_kinematically_feasible,
    transition_acceleration,
)


def node(t: float, s: float) -> STGridNode:
    return STGridNode(
        time_index=0,
        distance_index=0,
        t=t,
        s=s,
    )


def make_limits():
    return STKinematicLimits(
        min_speed=0.0,
        max_speed=25.0,
        max_accel=3.0,
        max_decel=6.0,
    )


def test_edge_speed_is_st_slope():
    assert edge_speed(
        start=node(1.0, 20.0),
        end=node(2.0, 40.0),
    ) == pytest.approx(20.0)


def test_edge_with_speed_below_limit_is_feasible():
    assert is_edge_speed_feasible(
        start=node(1.0, 20.0),
        end=node(2.0, 40.0),
        limits=make_limits(),
    )


def test_edge_above_speed_limit_is_rejected():
    assert not is_edge_speed_feasible(
        start=node(1.0, 20.0),
        end=node(2.0, 50.0),
        limits=make_limits(),
    )


def test_backward_motion_is_rejected():
    assert not is_edge_speed_feasible(
        start=node(1.0, 20.0),
        end=node(2.0, 10.0),
        limits=make_limits(),
    )


def test_transition_acceleration_is_computed():
    acceleration = transition_acceleration(
        previous=node(0.0, 0.0),
        current=node(1.0, 10.0),
        next_node=node(2.0, 23.0),
    )
    assert acceleration == pytest.approx(3.0)


def test_acceleration_above_limit_is_rejected():
    assert not is_transition_acceleration_feasible(
        previous=node(0.0, 0.0),
        current=node(1.0, 10.0),
        next_node=node(2.0, 24.0),
        limits=make_limits(),
    )


def test_deceleration_inside_limit_is_feasible():
    assert is_transition_acceleration_feasible(
        previous=node(0.0, 0.0),
        current=node(1.0, 15.0),
        next_node=node(2.0, 25.0),
        limits=make_limits(),
    )


def test_deceleration_above_limit_is_rejected():
    assert not is_transition_acceleration_feasible(
        previous=node(0.0, 0.0),
        current=node(1.0, 20.0),
        next_node=node(2.0, 30.0),
        limits=make_limits(),
    )


def test_first_transition_only_checks_speed():
    assert is_transition_kinematically_feasible(
        current=node(0.0, 0.0),
        next_node=node(1.0, 20.0),
        limits=make_limits(),
        previous=None,
    )


def test_full_transition_checks_speed_and_acceleration():
    assert not is_transition_kinematically_feasible(
        previous=node(0.0, 0.0),
        current=node(1.0, 10.0),
        next_node=node(2.0, 24.0),
        limits=make_limits(),
    )


def test_initial_edge_acceleration_is_computed():
    limits = make_limits()
    start = node(0.0, 0.0)
    end = node(1.0, 11.0)

    acceleration = initial_edge_acceleration(
        initial_speed=10.0,
        start=start,
        end=end,
    )
    assert acceleration == pytest.approx(2.0)
    assert is_initial_edge_acceleration_feasible(
        initial_speed=10.0,
        start=start,
        end=end,
        limits=limits,
    )


def test_initial_edge_excessive_braking_is_rejected():
    assert not is_initial_edge_acceleration_feasible(
        initial_speed=10.0,
        start=node(0.0, 0.0),
        end=node(1.0, 0.0),
        limits=make_limits(),
    )
