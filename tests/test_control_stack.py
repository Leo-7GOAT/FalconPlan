import math

import pytest

from control.control_stack import (
    VehicleControlStack,
)

from control.lateral_controller import (
    StanleyLateralController,
)

from control.models import (
    ControlTrajectory,
)

from control.pid import PIDController

from control.watchdog import (
    WatchdogConfig,
)

from vehicle_model.models import (
    VehicleParams,
    VehicleState,
)


def make_trajectory():

    return ControlTrajectory(
        x=[
            0.0,
            10.0,
            20.0,
            30.0,
            40.0,
        ],

        y=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        yaw=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        speed=[
            10.0,
            10.0,
            10.0,
            10.0,
            10.0,
        ],

        acceleration=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],

        curvature=[
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    )


def make_params():

    return VehicleParams(
        wheel_base=2.8,

        max_steer=0.5,

        max_accel=3.0,
        max_decel=6.0,

        max_speed=40.0,
        min_speed=0.0,
    )


def make_stack(
    delay_seconds=0.0,
    watchdog_config=None,
):

    params = make_params()

    lateral = (
        StanleyLateralController(
            k=2.0,
            softening=1.0,

            max_steer=(
                params.max_steer
            ),
        )
    )

    pid = PIDController(
        Kp=0.8,
        Ki=0.1,
        Kd=0.05,

        output_min=-6.0,
        output_max=3.0,
    )

    return VehicleControlStack(
        trajectory=make_trajectory(),

        vehicle_params=params,

        lateral_controller=lateral,

        longitudinal_controller=pid,

        dt=0.05,

        steering_delay_seconds=(
            delay_seconds
        ),

        # Effectively unlimited for most unit tests.
        max_steer_rate=100.0,

        watchdog_config=(
            watchdog_config
        ),
    )


def test_stack_outputs_vehicle_command():

    stack = make_stack()

    state = VehicleState(
        x=0.0,
        y=-1.0,

        psi=0.0,

        v=10.0,
    )

    result = stack.step(
        state
    )

    assert (
        result.controller_name
        == "Stanley"
    )

    assert (
        result.cross_track_error
        > 0.0
    )

    assert (
        result.requested_steering
        > 0.0
    )

    assert (
        result.actual_steering
        > 0.0
    )

    assert result.safe_stop is False


def test_zero_delay_passes_steering_immediately():

    stack = make_stack(
        delay_seconds=0.0
    )

    state = VehicleState(
        x=0.0,
        y=-1.0,
        psi=0.0,
        v=10.0,
    )

    result = stack.step(
        state
    )

    assert (
        result.delayed_steering
        == pytest.approx(
            result.requested_steering
        )
    )


def test_delay_holds_initial_command():

    stack = make_stack(
        delay_seconds=0.10
    )

    state = VehicleState(
        x=0.0,
        y=-1.0,
        psi=0.0,
        v=10.0,
    )

    result_1 = stack.step(
        state
    )

    result_2 = stack.step(
        state
    )

    result_3 = stack.step(
        state
    )

    assert (
        result_1.delayed_steering
        == pytest.approx(0.0)
    )

    assert (
        result_2.delayed_steering
        == pytest.approx(0.0)
    )

    assert (
        result_3.delayed_steering
        > 0.0
    )


def test_speed_target_is_used():

    stack = make_stack()

    state = VehicleState(
        x=0.0,
        y=0.0,

        psi=0.0,

        v=8.0,
    )

    result = stack.step(
        state
    )

    assert (
        result.target_speed
        == pytest.approx(
            10.0
        )
    )

    assert (
        result.command.a
        > 0.0
    )


def test_watchdog_triggers_safe_stop():

    watchdog_config = WatchdogConfig(
        max_cross_track_error=2.5,

        max_heading_error_deg=20.0,

        violation_frames=2,
    )

    stack = make_stack(
        watchdog_config=(
            watchdog_config
        )
    )

    state = VehicleState(
        x=0.0,
        y=0.0,

        # Path yaw = 0.
        # Vehicle yaw = -30 deg.
        #
        # heading error = +30 deg.
        psi=math.radians(
            -30.0
        ),

        v=10.0,
    )

    result_1 = stack.step(
        state
    )

    assert result_1.safe_stop is False

    result_2 = stack.step(
        state
    )

    assert result_2.safe_stop is True

    assert (
        result_2.watchdog_status.tripped
        is True
    )

    assert (
        result_2.command.a
        == pytest.approx(
            -6.0
        )
    )


def test_stack_detects_trajectory_completion():

    stack = make_stack()

    # Rear axle position such that front axle is near x=40.
    state = VehicleState(
        x=40.0 - 2.8,

        y=0.0,
        psi=0.0,

        v=10.0,
    )

    result = stack.step(
        state
    )

    assert (
        result.trajectory_completed
        is True
    )

    assert (
        result.command.a
        == pytest.approx(
            0.0
        )
    )


def test_reset_clears_stack_state():

    stack = make_stack(
        delay_seconds=0.10
    )

    state = VehicleState(
        x=20.0 - 2.8,

        y=-1.0,
        psi=0.0,

        v=10.0,
    )

    stack.step(
        state
    )

    assert (
        stack.tracker.previous_index
        > 0
    )

    stack.reset()

    assert (
        stack.tracker.previous_index
        == 0
    )

    assert (
        stack.watchdog.tripped
        is False
    )

    assert (
        stack.actuator.current_steer
        == pytest.approx(
            0.0
        )
    )