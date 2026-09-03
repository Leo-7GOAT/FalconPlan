import math

import pytest

from control.delay import (
    SteeringCommandDelay,
)


def test_zero_delay_passes_command_immediately():

    delay = SteeringCommandDelay(
        delay_seconds=0.0,
        dt=0.05,
    )

    output = delay.update(
        math.radians(10.0)
    )

    assert math.degrees(
        output
    ) == pytest.approx(
        10.0
    )


def test_three_frame_delay():

    delay = SteeringCommandDelay(
        delay_seconds=0.15,
        dt=0.05,
    )

    outputs = []

    for angle_deg in [
        10.0,
        15.0,
        20.0,
        25.0,
    ]:
        outputs.append(
            math.degrees(
                delay.update(
                    math.radians(
                        angle_deg
                    )
                )
            )
        )

    assert outputs == pytest.approx([
        0.0,
        0.0,
        0.0,
        10.0,
    ])


def test_delayed_commands_preserve_order():

    delay = SteeringCommandDelay(
        delay_seconds=0.10,
        dt=0.05,
    )

    outputs = []

    for command in [
        1.0,
        2.0,
        3.0,
        4.0,
    ]:
        outputs.append(
            delay.update(
                command
            )
        )

    assert outputs == pytest.approx([
        0.0,
        0.0,
        1.0,
        2.0,
    ])


def test_custom_initial_steer():

    delay = SteeringCommandDelay(
        delay_seconds=0.10,
        dt=0.05,
        initial_steer=0.2,
    )

    assert delay.update(
        1.0
    ) == pytest.approx(
        0.2
    )

    assert delay.update(
        2.0
    ) == pytest.approx(
        0.2
    )


def test_reset_clears_pending_commands():

    delay = SteeringCommandDelay(
        delay_seconds=0.10,
        dt=0.05,
    )

    delay.update(
        1.0
    )

    delay.update(
        2.0
    )

    delay.reset()

    assert delay.update(
        3.0
    ) == pytest.approx(
        0.0
    )


def test_negative_delay_rejected():

    with pytest.raises(
        ValueError
    ):
        SteeringCommandDelay(
            delay_seconds=-0.1,
            dt=0.05,
        )


def test_non_integer_delay_steps_rejected():

    with pytest.raises(
        ValueError
    ):
        SteeringCommandDelay(
            delay_seconds=0.12,
            dt=0.05,
        )