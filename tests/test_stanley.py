import math

import pytest

from control.stanley import StanleyController

from vehicle_model.models import (
    VehicleState,
)


def make_controller():

    return StanleyController(
        k=2.0,
        softening=1.0,
        max_steer=0.5,
    )


def test_vehicle_right_of_path_has_positive_cross_track_error():

    controller = make_controller()

    state = VehicleState(
        x=0.0,
        y=-2.0,
        psi=0.0,
        v=10.0,
    )

    index, heading_error, cross_track_error = (
        controller.compute_errors(
            state=state,

            trajectory_x=[
                0.0,
                5.0,
                10.0,
            ],

            trajectory_y=[
                0.0,
                0.0,
                0.0,
            ],

            trajectory_yaw=[
                0.0,
                0.0,
                0.0,
            ],

            wheel_base=2.8,
        )
    )

    assert heading_error == pytest.approx(
        0.0
    )

    # Path is on vehicle's left.
    assert cross_track_error == pytest.approx(
        2.0
    )


def test_vehicle_left_of_path_has_negative_cross_track_error():

    controller = make_controller()

    state = VehicleState(
        x=0.0,
        y=2.0,
        psi=0.0,
        v=10.0,
    )

    _, _, cross_track_error = (
        controller.compute_errors(
            state=state,

            trajectory_x=[
                0.0,
                5.0,
                10.0,
            ],

            trajectory_y=[
                0.0,
                0.0,
                0.0,
            ],

            trajectory_yaw=[
                0.0,
                0.0,
                0.0,
            ],

            wheel_base=2.8,
        )
    )

    assert cross_track_error == pytest.approx(
        -2.0
    )


def test_reference_stanley_case():

    controller = make_controller()

    steer = controller.update(
        heading_error=math.radians(3.0),
        cross_track_error=-1.5,
        speed=15.0,
    )

    assert math.degrees(
        steer
    ) == pytest.approx(
        -7.619655276,
        abs=1e-6,
    )


def test_steering_is_saturated():

    controller = StanleyController(
        k=20.0,
        softening=1.0,
        max_steer=0.5,
    )

    steer = controller.update(
        heading_error=1.0,
        cross_track_error=20.0,
        speed=1.0,
    )

    assert steer == pytest.approx(
        0.5
    )


def test_negative_speed_rejected():

    controller = make_controller()

    with pytest.raises(ValueError):

        controller.update(
            heading_error=0.0,
            cross_track_error=1.0,
            speed=-1.0,
        )


def test_empty_trajectory_rejected():

    controller = make_controller()

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=10.0,
    )

    with pytest.raises(ValueError):

        controller.compute_errors(
            state=state,
            trajectory_x=[],
            trajectory_y=[],
            trajectory_yaw=[],
            wheel_base=2.8,
        )


def test_mismatched_trajectory_lengths_rejected():

    controller = make_controller()

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=10.0,
    )

    with pytest.raises(ValueError):

        controller.compute_errors(
            state=state,

            trajectory_x=[
                0.0,
                1.0,
            ],

            trajectory_y=[
                0.0,
            ],

            trajectory_yaw=[
                0.0,
                0.0,
            ],

            wheel_base=2.8,
        )