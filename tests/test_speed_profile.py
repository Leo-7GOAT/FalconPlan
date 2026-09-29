import pytest

from planning.speed_profile import (
    SpeedProfile,
    SpeedProfilePoint,
    speed_profile_from_dp_plan,
)
from planning.st_dp import DPSpeedPlan
from planning.st_grid import STGridNode


def node(
    time_index: int,
    t: float,
    s: float,
) -> STGridNode:
    return STGridNode(
        time_index=time_index,
        distance_index=time_index,
        t=t,
        s=s,
    )


def make_constant_speed_plan():
    return DPSpeedPlan(
        nodes=(
            node(0, 0.0, 0.0),
            node(1, 1.0, 10.0),
            node(2, 2.0, 20.0),
            node(3, 3.0, 30.0),
        ),
        total_cost=0.0,
    )


def test_speed_profile_from_constant_speed_plan():
    profile = speed_profile_from_dp_plan(
        plan=make_constant_speed_plan(),
        initial_speed=10.0,
        initial_acceleration=0.0,
    )
    assert profile.times == pytest.approx(
        (0.0, 1.0, 2.0, 3.0)
    )
    assert profile.distances == pytest.approx(
        (0.0, 10.0, 20.0, 30.0)
    )
    assert profile.speeds == pytest.approx(
        (10.0, 10.0, 10.0, 10.0)
    )
    assert profile.accelerations == pytest.approx(
        (0.0, 0.0, 0.0, 0.0)
    )


def test_speed_profile_preserves_initial_state():
    profile = speed_profile_from_dp_plan(
        plan=make_constant_speed_plan(),
        initial_speed=10.0,
        initial_acceleration=1.5,
    )
    first = profile.points[0]
    assert first.t == pytest.approx(0.0)
    assert first.s == pytest.approx(0.0)
    assert first.v == pytest.approx(10.0)
    assert first.a == pytest.approx(1.5)


def test_speed_profile_computes_dp_acceleration():
    plan = DPSpeedPlan(
        nodes=(
            node(0, 0.0, 0.0),
            node(1, 1.0, 10.0),
            node(2, 2.0, 23.0),
        ),
        total_cost=1.0,
    )
    profile = speed_profile_from_dp_plan(
        plan=plan,
        initial_speed=10.0,
    )
    assert profile.speeds == pytest.approx(
        (10.0, 10.0, 13.0)
    )
    assert profile.accelerations == pytest.approx(
        (0.0, 0.0, 3.0)
    )


def test_speed_profile_properties_are_aligned():
    profile = SpeedProfile(
        points=(
            SpeedProfilePoint(
                t=0.0,
                s=0.0,
                v=5.0,
                a=1.0,
            ),
            SpeedProfilePoint(
                t=1.0,
                s=6.0,
                v=6.0,
                a=1.0,
            ),
        )
    )
    assert len(profile.times) == 2
    assert len(profile.distances) == 2
    assert len(profile.speeds) == 2
    assert len(profile.accelerations) == 2
    assert profile.duration == pytest.approx(1.0)


def test_speed_profile_rejects_negative_initial_speed():
    with pytest.raises(ValueError):
        speed_profile_from_dp_plan(
            plan=make_constant_speed_plan(),
            initial_speed=-1.0,
        )


def test_speed_profile_rejects_nonincreasing_time():
    with pytest.raises(ValueError):
        SpeedProfile(
            points=(
                SpeedProfilePoint(
                    t=0.0,
                    s=0.0,
                    v=10.0,
                    a=0.0,
                ),
                SpeedProfilePoint(
                    t=0.0,
                    s=10.0,
                    v=10.0,
                    a=0.0,
                ),
            )
        )


def test_speed_profile_rejects_decreasing_distance():
    with pytest.raises(ValueError):
        SpeedProfile(
            points=(
                SpeedProfilePoint(
                    t=0.0,
                    s=10.0,
                    v=10.0,
                    a=0.0,
                ),
                SpeedProfilePoint(
                    t=1.0,
                    s=5.0,
                    v=5.0,
                    a=-5.0,
                ),
            )
        )
