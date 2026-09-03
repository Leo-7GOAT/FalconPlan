import math

import numpy as np
import pytest

from control.lqr import LQRController


def make_controller():

    return LQRController(
        q_cross_track=1.0,
        q_heading=1.0,
        r_steer=10.0,
        max_steer=0.5,
    )


def test_discrete_model():

    A, B = (
        LQRController
        .build_discrete_model(
            speed=20.0,
            wheel_base=2.8,
            dt=0.05,
        )
    )

    assert np.allclose(
        A,
        np.array([
            [1.0, 1.0],
            [0.0, 1.0],
        ]),
    )

    assert np.allclose(
        B,
        np.array([
            [0.0],
            [-1.0 / 2.8],
        ]),
    )


def test_positive_cross_track_error_steers_left():

    controller = make_controller()

    steer = controller.update(
        cross_track_error=1.0,
        heading_error=0.0,

        speed=20.0,
        wheel_base=2.8,
        dt=0.05,
    )

    assert steer > 0.0


def test_negative_cross_track_error_steers_right():

    controller = make_controller()

    steer = controller.update(
        cross_track_error=-1.0,
        heading_error=0.0,

        speed=20.0,
        wheel_base=2.8,
        dt=0.05,
    )

    assert steer < 0.0


def test_zero_error_zero_curvature_zero_steer():

    controller = make_controller()

    steer = controller.update(
        cross_track_error=0.0,
        heading_error=0.0,

        speed=20.0,
        wheel_base=2.8,
        dt=0.05,

        reference_curvature=0.0,
    )

    assert steer == pytest.approx(
        0.0
    )


def test_curvature_feedforward():

    controller = make_controller()

    curvature = 0.01

    steer = controller.update(
        cross_track_error=0.0,
        heading_error=0.0,

        speed=20.0,
        wheel_base=2.8,
        dt=0.05,

        reference_curvature=curvature,
    )

    expected = math.atan(
        2.8 * curvature
    )

    assert steer == pytest.approx(
        expected
    )


def test_steering_saturation():

    controller = make_controller()

    steer = controller.update(
        cross_track_error=100.0,
        heading_error=0.0,

        speed=20.0,
        wheel_base=2.8,
        dt=0.05,
    )

    assert steer == pytest.approx(
        0.5
    )