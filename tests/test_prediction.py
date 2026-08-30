import pytest

from behavior.prediction import (
    predict_cv,
    predict_ca,
)


# ============================================================
# CV: Constant Velocity
# ============================================================

def test_cv_basic():

    result = predict_cv(
        s0=20.0,
        v0=10.0,
        t=3.0
    )

    assert result.s == pytest.approx(50.0)
    assert result.v == pytest.approx(10.0)


def test_cv_zero_time():

    result = predict_cv(
        s0=20.0,
        v0=10.0,
        t=0.0
    )

    assert result.s == pytest.approx(20.0)
    assert result.v == pytest.approx(10.0)


def test_cv_stopped_vehicle():

    result = predict_cv(
        s0=30.0,
        v0=0.0,
        t=5.0
    )

    assert result.s == pytest.approx(30.0)
    assert result.v == pytest.approx(0.0)


def test_cv_negative_initial_speed_is_treated_as_stopped():

    result = predict_cv(
        s0=30.0,
        v0=-10.0,
        t=5.0
    )

    assert result.s == pytest.approx(30.0)
    assert result.v == pytest.approx(0.0)


def test_cv_negative_time():

    with pytest.raises(ValueError):

        predict_cv(
            s0=0.0,
            v0=10.0,
            t=-1.0
        )


# ============================================================
# CA: Constant Acceleration
# ============================================================

def test_ca_acceleration():

    result = predict_ca(
        s0=10.0,
        v0=8.0,
        a=2.0,
        t=3.0
    )

    assert result.s == pytest.approx(43.0)
    assert result.v == pytest.approx(14.0)


def test_ca_deceleration_before_stop():

    result = predict_ca(
        s0=50.0,
        v0=20.0,
        a=-4.0,
        t=2.0
    )

    assert result.s == pytest.approx(82.0)
    assert result.v == pytest.approx(12.0)


def test_ca_exact_stop_time():

    result = predict_ca(
        s0=0.0,
        v0=10.0,
        a=-5.0,
        t=2.0
    )

    assert result.s == pytest.approx(10.0)
    assert result.v == pytest.approx(0.0)


def test_ca_after_stop():

    result = predict_ca(
        s0=0.0,
        v0=10.0,
        a=-5.0,
        t=5.0
    )

    assert result.s == pytest.approx(10.0)
    assert result.v == pytest.approx(0.0)


def test_ca_stopped_vehicle():

    result = predict_ca(
        s0=30.0,
        v0=0.0,
        a=-5.0,
        t=3.0
    )

    assert result.s == pytest.approx(30.0)
    assert result.v == pytest.approx(0.0)


def test_ca_negative_initial_speed_is_treated_as_stopped():

    result = predict_ca(
        s0=30.0,
        v0=-10.0,
        a=-5.0,
        t=3.0
    )

    assert result.s == pytest.approx(30.0)
    assert result.v == pytest.approx(0.0)


def test_ca_zero_acceleration():

    result = predict_ca(
        s0=20.0,
        v0=10.0,
        a=0.0,
        t=3.0
    )

    assert result.s == pytest.approx(50.0)
    assert result.v == pytest.approx(10.0)


def test_ca_zero_time():

    result = predict_ca(
        s0=20.0,
        v0=10.0,
        a=2.0,
        t=0.0
    )

    assert result.s == pytest.approx(20.0)
    assert result.v == pytest.approx(10.0)


def test_ca_negative_time():

    with pytest.raises(ValueError):

        predict_ca(
            s0=0.0,
            v0=10.0,
            a=2.0,
            t=-1.0
        )


# ============================================================
# CV / CA consistency
# ============================================================

@pytest.mark.parametrize(
    "s0, v0, t",
    [
        (0.0, 10.0, 1.0),
        (20.0, 5.0, 2.0),
        (-10.0, 15.0, 0.5),
        (100.0, 30.0, 3.0),
    ]
)
def test_ca_with_zero_acceleration_matches_cv(
        s0,
        v0,
        t
):

    cv_result = predict_cv(
        s0=s0,
        v0=v0,
        t=t
    )

    ca_result = predict_ca(
        s0=s0,
        v0=v0,
        a=0.0,
        t=t
    )

    assert ca_result.s == pytest.approx(
        cv_result.s
    )

    assert ca_result.v == pytest.approx(
        cv_result.v
    )