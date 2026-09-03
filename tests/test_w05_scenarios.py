import math

import pytest

from control.actuator import SteeringActuator
from control.control_stack import VehicleControlStack
from control.lateral_controller import (
    LQRLateralController,
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


def make_straight_trajectory(
    speed=15.0,
):

    count = 40

    return ControlTrajectory(
        x=[
            float(i * 5)
            for i in range(count)
        ],

        y=[
            0.0
            for _ in range(count)
        ],

        yaw=[
            0.0
            for _ in range(count)
        ],

        speed=[
            speed
            for _ in range(count)
        ],

        acceleration=[
            0.0
            for _ in range(count)
        ],

        curvature=[
            0.0
            for _ in range(count)
        ],
    )


def make_stack(
    params,
    *,
    controller="Stanley",
    delay=0.0,
    max_steer_rate_deg=90.0,
    watchdog=None,
):

    if controller == "Stanley":

        lateral = StanleyLateralController(
            k=2.0,
            softening=1.0,

            max_steer=params.max_steer,
        )

    elif controller == "LQR":

        lateral = LQRLateralController(
            q_cross_track=1.0,
            q_heading=1.0,

            r_steer=10.0,

            max_steer=params.max_steer,
        )

    else:

        raise ValueError(
            controller
        )


    pid = PIDController(
        Kp=0.8,
        Ki=0.1,
        Kd=0.05,

        output_min=-params.max_decel,
        output_max=params.max_accel,
    )


    if watchdog is None:

        watchdog = WatchdogConfig(
            max_cross_track_error=2.5,

            max_heading_error_deg=20.0,

            violation_frames=3,
        )


    return VehicleControlStack(
        trajectory=(
            make_straight_trajectory()
        ),

        vehicle_params=params,

        lateral_controller=lateral,

        longitudinal_controller=pid,

        dt=DT,

        steering_delay_seconds=delay,

        max_steer_rate=math.radians(
            max_steer_rate_deg
        ),

        watchdog_config=watchdog,
    )


def run_closed_loop(
    hal,
    stack,
    max_steps=320,
):

    results = []

    for _ in range(max_steps):

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


# ============================================================
# SCENARIO 1
#
# Nominal:
# centered vehicle should complete trajectory.
# ============================================================

def test_w05_nominal_tracking_completes():

    params = make_params()

    hal = SimulatorAdapter(
        model=KinematicBicycleModel(
            params=params
        ),

        initial_state=VehicleState(
            x=0.0,
            y=0.0,
            psi=0.0,
            v=10.0,
        ),

        integration_method="rk4",
    )

    stack = make_stack(
        params
    )

    results = run_closed_loop(
        hal,
        stack,
    )

    assert (
        results[-1]
        .trajectory_completed
        is True
    )

    assert not any(
        result.safe_stop
        for result in results
    )


# ============================================================
# SCENARIO 2
#
# Initial 1 m lateral disturbance should converge.
# ============================================================

def test_w05_lateral_disturbance_converges():

    params = make_params()

    hal = SimulatorAdapter(
        model=KinematicBicycleModel(
            params=params
        ),

        initial_state=VehicleState(
            x=0.0,
            y=-1.0,
            psi=0.0,
            v=10.0,
        ),

        integration_method="rk4",
    )

    stack = make_stack(
        params
    )

    results = run_closed_loop(
        hal,
        stack,
    )

    assert (
        results[-1]
        .trajectory_completed
        is True
    )

    assert abs(
        results[-1]
        .cross_track_error
    ) < 0.05

    assert not any(
        result.safe_stop
        for result in results
    )


# ============================================================
# SCENARIO 3
#
# Steering actuator must obey rate limit.
# ============================================================

def test_w05_steering_rate_limit_is_enforced():

    actuator = SteeringActuator(
        max_steer=0.5,

        max_steer_rate=math.radians(
            90.0
        ),
    )

    outputs = []

    for _ in range(10):

        outputs.append(
            actuator.update(
                target_steer=math.radians(
                    28.0
                ),

                dt=DT,
            )
        )

    rates = [
        abs(
            outputs[i]
            - outputs[i - 1]
        )
        / DT

        for i in range(
            1,
            len(outputs)
        )
    ]

    assert max(
        rates
    ) <= (
        math.radians(90.0)
        + 1e-12
    )


# ============================================================
# SCENARIO 4
#
# Unsafe heading should trigger watchdog safe stop.
# ============================================================

def test_w05_watchdog_triggers_safe_stop():

    params = make_params()

    hal = SimulatorAdapter(
        model=KinematicBicycleModel(
            params=params
        ),

        initial_state=VehicleState(
            x=0.0,
            y=0.0,

            psi=math.radians(
                -35.0
            ),

            v=10.0,
        ),

        integration_method="rk4",
    )

    stack = make_stack(
        params,

        watchdog=WatchdogConfig(
            max_cross_track_error=2.5,

            max_heading_error_deg=20.0,

            violation_frames=3,
        ),
    )

    results = run_closed_loop(
        hal,
        stack,
        max_steps=20,
    )

    assert any(
        result.safe_stop
        for result in results
    )

    safe_stop_result = next(
        result
        for result in results
        if result.safe_stop
    )

    assert (
        safe_stop_result
        .watchdog_status
        .tripped
        is True
    )

    assert (
        safe_stop_result
        .command
        .a
        == pytest.approx(
            -params.max_decel
        )
    )


# ============================================================
# SCENARIO 5
#
# Same ControlStack contract must produce the same result
# through SimulatorAdapter and MockHardwareAdapter.
# ============================================================

def test_w05_hal_backends_are_equivalent():

    initial_state = VehicleState(
        x=0.0,
        y=-1.0,
        psi=0.0,
        v=10.0,
    )


    sim_params = make_params()

    simulator = SimulatorAdapter(
        model=KinematicBicycleModel(
            params=sim_params
        ),

        initial_state=initial_state,

        integration_method="rk4",
    )

    sim_stack = make_stack(
        sim_params
    )


    hw_params = make_params()

    hardware = MockHardwareAdapter(
        model=KinematicBicycleModel(
            params=hw_params
        ),

        initial_state=initial_state,
    )

    hw_stack = make_stack(
        hw_params
    )


    sim_results = run_closed_loop(
        simulator,
        sim_stack,
    )

    hw_results = run_closed_loop(
        hardware,
        hw_stack,
    )


    assert (
        sim_results[-1]
        .trajectory_completed
        is True
    )

    assert (
        hw_results[-1]
        .trajectory_completed
        is True
    )


    sim_state = simulator.get_state()
    hw_state = hardware.get_state()


    assert hw_state.x == pytest.approx(
        sim_state.x,
        abs=1e-9,
    )

    assert hw_state.y == pytest.approx(
        sim_state.y,
        abs=1e-9,
    )

    assert hw_state.psi == pytest.approx(
        sim_state.psi,
        abs=1e-9,
    )

    assert hw_state.v == pytest.approx(
        sim_state.v,
        abs=1e-9,
    )

    assert len(
        hardware.can_tx_log
    ) > 0