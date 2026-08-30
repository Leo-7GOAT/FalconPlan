import pytest

from behavior.idm import (
    compute_desired_gap,
    compute_idm_acceleration,
)

from behavior.models import IDMParams


@pytest.fixture
def params():
    return IDMParams(
        desired_speed=30.0,
        max_accel=2.0,
        comfortable_decel=2.0,
        min_gap=2.0,
        time_headway=1.5,
        delta=4.0,
    )


def test_desired_gap_approaching_vehicle(params):

    result = compute_desired_gap(
        ego_speed=20.0,
        front_speed=15.0,
        params=params,
    )

    assert result == pytest.approx(57.0)


def test_desired_gap_same_speed(params):

    result = compute_desired_gap(
        ego_speed=20.0,
        front_speed=20.0,
        params=params,
    )

    assert result == pytest.approx(32.0)


def test_free_road_acceleration(params):

    result = compute_idm_acceleration(
        ego_speed=20.0,
        gap=None,
        front_speed=None,
        params=params,
    )

    assert result == pytest.approx(
        1.6049382716,
        abs=1e-9,
    )


def test_acceleration_at_desired_speed_is_zero(params):

    result = compute_idm_acceleration(
        ego_speed=30.0,
        gap=None,
        front_speed=None,
        params=params,
    )

    assert result == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_speed_above_desired_causes_deceleration(params):

    result = compute_idm_acceleration(
        ego_speed=35.0,
        gap=None,
        front_speed=None,
        params=params,
    )

    assert result < 0.0


def test_equilibrium_following(params):

    result = compute_idm_acceleration(
        ego_speed=20.0,
        gap=35.7220035617,
        front_speed=20.0,
        params=params,
    )

    assert result == pytest.approx(
        0.0,
        abs=1e-8,
    )


def test_fast_approach_causes_strong_deceleration(params):

    result = compute_idm_acceleration(
        ego_speed=20.0,
        gap=20.0,
        front_speed=15.0,
        params=params,
    )

    assert result == pytest.approx(
        -14.6400617284,
        abs=1e-8,
    )


def test_smaller_gap_causes_more_braking(params):

    far_result = compute_idm_acceleration(
        ego_speed=20.0,
        gap=50.0,
        front_speed=15.0,
        params=params,
    )

    near_result = compute_idm_acceleration(
        ego_speed=20.0,
        gap=20.0,
        front_speed=15.0,
        params=params,
    )

    assert near_result < far_result


def test_faster_front_vehicle_reduces_braking(params):

    slow_front = compute_idm_acceleration(
        ego_speed=20.0,
        gap=30.0,
        front_speed=10.0,
        params=params,
    )

    fast_front = compute_idm_acceleration(
        ego_speed=20.0,
        gap=30.0,
        front_speed=20.0,
        params=params,
    )

    assert fast_front > slow_front


def test_non_positive_gap_is_invalid(params):

    with pytest.raises(ValueError):

        compute_idm_acceleration(
            ego_speed=20.0,
            gap=0.0,
            front_speed=15.0,
            params=params,
        )


def test_partial_front_vehicle_input_is_invalid(params):

    with pytest.raises(ValueError):

        compute_idm_acceleration(
            ego_speed=20.0,
            gap=None,
            front_speed=15.0,
            params=params,
        )

    with pytest.raises(ValueError):

        compute_idm_acceleration(
            ego_speed=20.0,
            gap=20.0,
            front_speed=None,
            params=params,
        )