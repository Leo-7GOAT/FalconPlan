import math

from control.lateral_controller import (
    LQRLateralController,
    StanleyLateralController,
)

from control.models import (
    LateralControlInput,
)


def make_input(
    cross_track_error=1.0,
    heading_error=0.0,
    curvature=0.0,
):

    return LateralControlInput(
        cross_track_error=(
            cross_track_error
        ),

        heading_error=(
            heading_error
        ),

        speed=20.0,

        wheel_base=2.8,

        dt=0.05,

        reference_curvature=(
            curvature
        ),
    )


def test_stanley_strategy_steers_left():

    controller = (
        StanleyLateralController(
            k=2.0,
            softening=1.0,
            max_steer=0.5,
        )
    )

    steer = controller.update(
        make_input()
    )

    assert steer > 0.0


def test_lqr_strategy_steers_left():

    controller = (
        LQRLateralController(
            q_cross_track=1.0,
            q_heading=1.0,

            r_steer=10.0,

            max_steer=0.5,
        )
    )

    steer = controller.update(
        make_input()
    )

    assert steer > 0.0


def test_lqr_strategy_uses_curvature_feedforward():

    controller = (
        LQRLateralController(
            q_cross_track=1.0,
            q_heading=1.0,

            r_steer=10.0,

            max_steer=0.5,
        )
    )

    steer = controller.update(
        make_input(
            cross_track_error=0.0,
            heading_error=0.0,

            curvature=0.01,
        )
    )

    expected = math.atan(
        2.8 * 0.01
    )

    assert abs(
        steer - expected
    ) < 1e-9