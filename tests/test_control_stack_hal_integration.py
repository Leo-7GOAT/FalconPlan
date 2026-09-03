import math

import pytest

from control.control_stack import VehicleControlStack
from control.lateral_controller import (
    StanleyLateralController,
)
from control.models import ControlTrajectory
from control.pid import PIDController
from control.watchdog import WatchdogConfig

from hal import (
    MockHardwareAdapter,
    SimulatorAdapter,
)

from vehicle_model import (
    KinematicBicycleModel,
    VehicleParams,
    VehicleState,
)


DT = 0.05


def make_params():

    return VehicleParams(
        wheel_base=2.8,

        max_steer=0.5,

        max_accel=3.0,
        max_decel=6.0,

        max_speed=40.0,
        min_speed=0.0,
    )


def make_trajectory():

    x = [
        float(i * 5)
        for i in range(30)
    ]

    n = len(x)

    return ControlTrajectory(
        x=x,

        y=[
            0.0
            for _ in range(n)
        ],

        yaw=[
            0.0
            for _ in range(n)
        ],

        speed=[
            10.0
            for _ in range(n)
        ],

        acceleration=[
            0.0
            for _ in range(n)
        ],

        curvature=[
            0.0
            for _ in range(n)
        ],
    )


def make_stack(
    params,
):

    return VehicleControlStack(
        trajectory=make_trajectory(),

        vehicle_params=params,

        lateral_controller=(
            StanleyLateralController(
                k=2.0,
                softening=1.0,

                max_steer=(
                    params.max_steer
                ),
            )
        ),

        longitudinal_controller=(
            PIDController(
                Kp=0.8,
                Ki=0.1,
                Kd=0.05,

                output_min=(
                    -params.max_decel
                ),

                output_max=(
                    params.max_accel
                ),
            )
        ),

        dt=DT,

        steering_delay_seconds=0.0,

        max_steer_rate=math.radians(
            90.0
        ),

        watchdog_config=(
            WatchdogConfig(
                max_cross_track_error=2.5,

                max_heading_error_deg=20.0,

                violation_frames=3,
            )
        ),
    )


def make_initial_state():

    return VehicleState(
        x=0.0,
        y=-1.0,

        psi=0.0,

        v=10.0,
    )


def run_closed_loop(
    hal,
    stack,
    steps=40,
):

    results = []

    for _ in range(steps):

        state = hal.get_state()

        result = stack.step(
            state
        )

        results.append(
            result
        )

        if result.trajectory_completed:
            break

        hal.apply_command(
            result.command
        )

        hal.step(
            DT
        )

        if result.safe_stop:
            break

    return results


def test_control_stack_drives_simulator_hal():

    params = make_params()

    model = KinematicBicycleModel(
        params=params
    )

    hal = SimulatorAdapter(
        model=model,

        initial_state=(
            make_initial_state()
        ),

        integration_method="rk4",
    )

    stack = make_stack(
        params
    )

    results = run_closed_loop(
        hal=hal,
        stack=stack,
    )

    final_state = (
        hal.get_state()
    )

    assert len(results) > 0

    assert final_state.x > 0.0

    assert abs(
        final_state.y
    ) < 1.0

    assert not any(
        result.safe_stop
        for result in results
    )


def test_control_stack_drives_mock_hardware_hal():

    params = make_params()

    model = KinematicBicycleModel(
        params=params
    )

    hal = MockHardwareAdapter(
        model=model,

        initial_state=(
            make_initial_state()
        ),
    )

    stack = make_stack(
        params
    )

    results = run_closed_loop(
        hal=hal,
        stack=stack,
    )

    final_state = (
        hal.get_state()
    )

    assert len(results) > 0

    assert final_state.x > 0.0

    assert len(
        hal.can_tx_log
    ) > 0

    assert not any(
        result.safe_stop
        for result in results
    )


def test_simulator_and_mock_hardware_match():

    params_1 = make_params()

    simulator_hal = (
        SimulatorAdapter(
            model=(
                KinematicBicycleModel(
                    params=params_1
                )
            ),

            initial_state=(
                make_initial_state()
            ),

            integration_method="rk4",
        )
    )

    simulator_stack = make_stack(
        params_1
    )


    params_2 = make_params()

    hardware_hal = (
        MockHardwareAdapter(
            model=(
                KinematicBicycleModel(
                    params=params_2
                )
            ),

            initial_state=(
                make_initial_state()
            ),
        )
    )

    hardware_stack = make_stack(
        params_2
    )


    run_closed_loop(
        hal=simulator_hal,

        stack=simulator_stack,
    )

    run_closed_loop(
        hal=hardware_hal,

        stack=hardware_stack,
    )


    simulator_state = (
        simulator_hal.get_state()
    )

    hardware_state = (
        hardware_hal.get_state()
    )


    assert hardware_state.x == pytest.approx(
        simulator_state.x,
        abs=1e-6,
    )

    assert hardware_state.y == pytest.approx(
        simulator_state.y,
        abs=1e-6,
    )

    assert hardware_state.psi == pytest.approx(
        simulator_state.psi,
        abs=1e-6,
    )

    assert hardware_state.v == pytest.approx(
        simulator_state.v,
        abs=1e-6,
    )