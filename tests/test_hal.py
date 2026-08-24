import math

import pytest

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)

from hal import (
    VehicleHAL,
    KinematicVehicleHAL,
)


@pytest.fixture
def model():

    params = VehicleParams(
        wheel_base=2.8,
        max_steer=math.radians(35),
        max_accel=3.0,
        max_decel=6.0,
        max_speed=40.0,
        min_speed=0.0
    )

    return KinematicBicycleModel(
        params
    )


@pytest.fixture
def initial_state():

    return VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=10.0
    )


@pytest.fixture
def vehicle(
    model,
    initial_state
):

    return KinematicVehicleHAL(
        model=model,
        initial_state=initial_state
    )

def test_vehicle_hal_is_abstract():

    with pytest.raises(TypeError):
        VehicleHAL()

def test_get_initial_state(
        vehicle,
        initial_state
):

    state = vehicle.get_state()

    assert state == initial_state

def test_default_command_straight(
        vehicle
):

    state = vehicle.step(
        dt=0.1
    )

    assert state.x == pytest.approx(
        1.0
    )

    assert state.y == pytest.approx(
        0.0
    )

    assert state.psi == pytest.approx(
        0.0
    )

    assert state.v == pytest.approx(
        10.0
    )

def test_apply_steering_command(
        vehicle
):

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    vehicle.apply_command(
        command
    )

    state = vehicle.step(
        dt=0.1
    )

    assert state.psi > 0.0
    assert state.y > 0.0

def test_hal_matches_direct_model(
        model,
        initial_state
):

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    direct_result = model.step(
        initial_state,
        command,
        dt=0.5,
        method="rk4"
    )

    vehicle = KinematicVehicleHAL(
        model=model,
        initial_state=initial_state,
        integration_method="rk4"
    )

    vehicle.apply_command(
        command
    )

    hal_result = vehicle.step(
        dt=0.5
    )

    assert hal_result.x == pytest.approx(
        direct_result.x
    )

    assert hal_result.y == pytest.approx(
        direct_result.y
    )

    assert hal_result.psi == pytest.approx(
        direct_result.psi
    )

    assert hal_result.v == pytest.approx(
        direct_result.v
    )

def test_state_persists_between_steps(
        vehicle
):

    first = vehicle.step(
        dt=0.1
    )

    second = vehicle.step(
        dt=0.1
    )

    assert first.x == pytest.approx(
        1.0
    )

    assert second.x == pytest.approx(
        2.0
    )

def test_command_persists_between_steps(
        vehicle
):

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    vehicle.apply_command(
        command
    )

    first = vehicle.step(
        dt=0.1
    )

    second = vehicle.step(
        dt=0.1
    )

    assert second.psi > first.psi
    assert second.y > first.y

def test_hal_supports_euler(
        model,
        initial_state
):

    vehicle = KinematicVehicleHAL(
        model=model,
        initial_state=initial_state,
        integration_method="euler"
    )

    command = VehicleCommand(
        delta=math.radians(10),
        a=0.0
    )

    vehicle.apply_command(
        command
    )

    state = vehicle.step(
        dt=0.5
    )

    assert state.x == pytest.approx(
        5.0
    )

    assert state.y == pytest.approx(
        0.0
    )

    assert state.psi == pytest.approx(
        0.3148696084,
        abs=1e-9
    )