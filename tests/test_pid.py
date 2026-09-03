import pytest

from control.pid import PIDController


def test_p_only_control():

    pid = PIDController(
        Kp=2.0,
        Ki=0.0,
        Kd=0.0,
    )

    u = pid.update(
        target=10.0,
        measurement=7.0,
        dt=0.1,
    )

    # error = 3
    # u = 2 * 3 = 6
    assert u == pytest.approx(6.0)


def test_integral_accumulates():

    pid = PIDController(
        Kp=0.0,
        Ki=1.0,
        Kd=0.0,
    )

    u1 = pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.5,
    )

    # I1 = 0 + 2 * 0.5 = 1
    assert u1 == pytest.approx(1.0)

    u2 = pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.5,
    )

    # I2 = 1 + 2 * 0.5 = 2
    assert u2 == pytest.approx(2.0)


def test_first_derivative_is_zero():

    pid = PIDController(
        Kp=0.0,
        Ki=0.0,
        Kd=1.0,
    )

    u = pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.1,
    )

    # 第一帧没有 previous_error
    assert u == pytest.approx(0.0)


def test_derivative_uses_previous_error():

    pid = PIDController(
        Kp=0.0,
        Ki=0.0,
        Kd=1.0,
    )

    pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.1,
    )

    # first error = 2

    u = pid.update(
        target=10.0,
        measurement=8.5,
        dt=0.1,
    )

    # new error = 1.5
    #
    # D = (1.5 - 2.0) / 0.1
    #   = -5
    assert u == pytest.approx(-5.0)


def test_reset_clears_history():

    pid = PIDController(
        Kp=0.0,
        Ki=1.0,
        Kd=1.0,
    )

    pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.1,
    )

    assert pid.integral != 0.0
    assert pid.previous_error is not None

    pid.reset()

    assert pid.integral == pytest.approx(0.0)
    assert pid.previous_error is None


def test_non_positive_dt_rejected():

    pid = PIDController(
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
    )

    with pytest.raises(ValueError):
        pid.update(
            target=10.0,
            measurement=8.0,
            dt=0.0,
        )

def test_integral_does_not_wind_up_at_upper_limit():

    pid = PIDController(
        Kp=1.0,
        Ki=1.0,
        Kd=0.0,

        output_min=-3.0,
        output_max=3.0,
    )

    for _ in range(100):
        pid.update(
            target=20.0,
            measurement=0.0,
            dt=0.1,
        )

    assert pid.integral == pytest.approx(
        0.0
    )


def test_integral_does_not_wind_up_at_lower_limit():

    pid = PIDController(
        Kp=1.0,
        Ki=1.0,
        Kd=0.0,

        output_min=-3.0,
        output_max=3.0,
    )

    for _ in range(100):
        pid.update(
            target=0.0,
            measurement=20.0,
            dt=0.1,
        )

    assert pid.integral == pytest.approx(
        0.0
    )


def test_integral_accumulates_when_not_saturated():

    pid = PIDController(
        Kp=0.1,
        Ki=0.1,
        Kd=0.0,

        output_min=-10.0,
        output_max=10.0,
    )

    pid.update(
        target=10.0,
        measurement=9.0,
        dt=0.1,
    )

    assert pid.integral == pytest.approx(
        0.1
    )

def test_feedforward_is_added_to_pid_output():

    pid = PIDController(
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        output_min=-10.0,
        output_max=10.0,
    )

    u = pid.update(
        target=10.0,
        measurement=9.0,
        dt=0.1,
        feedforward=2.0,
    )

    # feedback:
    # Kp * error = 1.0
    #
    # feedforward = 2.0
    #
    # total = 3.0

    assert u == pytest.approx(
        3.0
    )

def test_feedforward_respects_output_saturation():

    pid = PIDController(
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        output_min=-6.0,
        output_max=3.0,
    )

    u = pid.update(
        target=10.0,
        measurement=8.0,
        dt=0.1,
        feedforward=2.0,
    )

    # raw = 2 + 2 = 4
    # saturated = 3

    assert u == pytest.approx(
        3.0
    )