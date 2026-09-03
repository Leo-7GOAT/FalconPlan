import math

import pytest

from control.actuator import (
    SteeringActuator,
)


def test_small_command_passes_directly():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=math.radians(90.0),
    )

    steer = actuator.update(
        target_steer=math.radians(2.0),
        dt=0.05,
    )

    assert math.degrees(
        steer
    ) == pytest.approx(
        2.0
    )


def test_large_command_is_rate_limited():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=math.radians(90.0),
    )

    steer = actuator.update(
        target_steer=math.radians(20.0),
        dt=0.05,
    )

    # 90 deg/s * 0.05 s = 4.5 deg
    assert math.degrees(
        steer
    ) == pytest.approx(
        4.5
    )


def test_rate_limit_accumulates_over_frames():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=math.radians(90.0),
    )

    steer = 0.0

    for _ in range(3):
        steer = actuator.update(
            target_steer=math.radians(20.0),
            dt=0.05,
        )

    assert math.degrees(
        steer
    ) == pytest.approx(
        13.5
    )


def test_negative_direction_rate_limit():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=math.radians(90.0),
    )

    steer = actuator.update(
        target_steer=math.radians(-20.0),
        dt=0.05,
    )

    assert math.degrees(
        steer
    ) == pytest.approx(
        -4.5
    )


def test_absolute_steering_limit():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=100.0,
    )

    steer = actuator.update(
        target_steer=10.0,
        dt=1.0,
    )

    assert steer == pytest.approx(
        0.5
    )


def test_reset():

    actuator = SteeringActuator(
        max_steer=0.5,
        max_steer_rate=1.0,
    )

    actuator.update(
        target_steer=0.3,
        dt=0.1,
    )

    actuator.reset()

    assert actuator.current_steer == pytest.approx(
        0.0
    )