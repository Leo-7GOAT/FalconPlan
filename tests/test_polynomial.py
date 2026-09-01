import pytest

from planning.polynomial import (
    QuinticPolynomial,
)


def test_quintic_start_boundary():

    poly = QuinticPolynomial(
        x0=1.0,
        v0=2.0,
        a0=0.5,

        x1=10.0,
        v1=0.0,
        a1=0.0,

        T=3.0,
    )

    assert poly.position(0.0) == pytest.approx(
        1.0
    )

    assert poly.velocity(0.0) == pytest.approx(
        2.0
    )

    assert poly.acceleration(0.0) == pytest.approx(
        0.5
    )


def test_quintic_end_boundary():

    poly = QuinticPolynomial(
        x0=1.0,
        v0=2.0,
        a0=0.5,

        x1=10.0,
        v1=3.0,
        a1=-0.2,

        T=4.0,
    )

    assert poly.position(4.0) == pytest.approx(
        10.0
    )

    assert poly.velocity(4.0) == pytest.approx(
        3.0
    )

    assert poly.acceleration(4.0) == pytest.approx(
        -0.2
    )


def test_lane_change_midpoint():

    poly = QuinticPolynomial(
        x0=0.0,
        v0=0.0,
        a0=0.0,

        x1=3.5,
        v1=0.0,
        a1=0.0,

        T=3.0,
    )

    assert poly.position(1.5) == pytest.approx(
        1.75
    )

    assert poly.acceleration(1.5) == pytest.approx(
        0.0,
        abs=1e-10,
    )


def test_lane_change_coefficients():

    poly = QuinticPolynomial(
        x0=0.0,
        v0=0.0,
        a0=0.0,

        x1=3.5,
        v1=0.0,
        a1=0.0,

        T=3.0,
    )

    assert poly.c0 == pytest.approx(0.0)
    assert poly.c1 == pytest.approx(0.0)
    assert poly.c2 == pytest.approx(0.0)

    assert poly.c3 == pytest.approx(
        1.2962962963
    )

    assert poly.c4 == pytest.approx(
        -0.6481481481
    )

    assert poly.c5 == pytest.approx(
        0.0864197531
    )


def test_non_positive_duration_rejected():

    with pytest.raises(ValueError):

        QuinticPolynomial(
            x0=0.0,
            v0=0.0,
            a0=0.0,

            x1=3.5,
            v1=0.0,
            a1=0.0,

            T=0.0,
        )


def test_negative_duration_rejected():

    with pytest.raises(ValueError):

        QuinticPolynomial(
            x0=0.0,
            v0=0.0,
            a0=0.0,

            x1=3.5,
            v1=0.0,
            a1=0.0,

            T=-2.0,
        )


from planning.polynomial import (
    QuinticPolynomial,
    QuarticPolynomial,
)


def test_quartic_start_boundary():

    poly = QuarticPolynomial(
        x0=10.0,
        v0=20.0,
        a0=1.0,
        v1=25.0,
        a1=0.0,
        T=3.0,
    )

    assert poly.position(0.0) == pytest.approx(10.0)
    assert poly.velocity(0.0) == pytest.approx(20.0)
    assert poly.acceleration(0.0) == pytest.approx(1.0)


def test_quartic_end_velocity_acceleration():

    poly = QuarticPolynomial(
        x0=0.0,
        v0=20.0,
        a0=0.0,
        v1=25.0,
        a1=0.0,
        T=3.0,
    )

    assert poly.velocity(3.0) == pytest.approx(25.0)
    assert poly.acceleration(3.0) == pytest.approx(0.0)


def test_quartic_reference_case():

    poly = QuarticPolynomial(
        x0=0.0,
        v0=20.0,
        a0=0.0,
        v1=25.0,
        a1=0.0,
        T=3.0,
    )

    assert poly.c0 == pytest.approx(0.0)
    assert poly.c1 == pytest.approx(20.0)
    assert poly.c2 == pytest.approx(0.0)

    assert poly.c3 == pytest.approx(
        0.5555555556
    )

    assert poly.c4 == pytest.approx(
        -0.0925925926
    )

    assert poly.position(3.0) == pytest.approx(
        67.5
    )


def test_quartic_non_positive_duration_rejected():

    with pytest.raises(ValueError):

        QuarticPolynomial(
            x0=0.0,
            v0=20.0,
            a0=0.0,
            v1=25.0,
            a1=0.0,
            T=0.0,
        )