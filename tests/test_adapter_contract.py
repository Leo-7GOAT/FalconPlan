import math

import pytest

from hal import MockHardwareAdapter, SimulatorAdapter, VehicleHAL
from vehicle_model import (
    KinematicBicycleModel,
    VehicleCommand,
    VehicleParams,
    VehicleState,
)


@pytest.fixture
def model() -> KinematicBicycleModel:
    return KinematicBicycleModel(
        VehicleParams(
            wheel_base=2.8,
            max_steer=math.radians(35.0),
            max_accel=3.0,
            max_decel=6.0,
            max_speed=40.0,
        )
    )


@pytest.fixture
def initial_state() -> VehicleState:
    return VehicleState(x=0.0, y=0.0, psi=0.0, v=5.0)


def test_backends_implement_vehicle_hal(model, initial_state):
    assert isinstance(SimulatorAdapter(model, initial_state), VehicleHAL)
    assert isinstance(MockHardwareAdapter(model, initial_state), VehicleHAL)


def test_same_upper_level_program_switches_backends(model, initial_state):
    command = VehicleCommand(delta=0.1, a=1.5)
    backends = [
        SimulatorAdapter(model, initial_state),
        MockHardwareAdapter(model, initial_state),
    ]

    results = []
    for backend in backends:
        backend.apply_command(command)
        results.append(backend.step(0.2))

    assert results[0] == results[1]


@pytest.mark.parametrize(
    ("acceleration", "expected_throttle", "expected_brake"),
    [(1.5, 0.5, 0.0), (-3.0, 0.0, 0.5), (100.0, 1.0, 0.0)],
)
def test_mock_longitudinal_channel_semantics(
    model,
    initial_state,
    acceleration,
    expected_throttle,
    expected_brake,
):
    adapter = MockHardwareAdapter(model, initial_state)
    adapter.apply_command(VehicleCommand(delta=0.0, a=acceleration))

    assert adapter.throttle == pytest.approx(expected_throttle)
    assert adapter.brake == pytest.approx(expected_brake)
    assert adapter.can_tx_log[-1]["throttle"] == pytest.approx(expected_throttle)
    assert adapter.can_tx_log[-1]["brake"] == pytest.approx(expected_brake)


def test_mock_can_uses_saturated_road_wheel_steering(model, initial_state):
    adapter = MockHardwareAdapter(model, initial_state)
    adapter.apply_command(VehicleCommand(delta=math.pi / 2, a=0.0))

    assert adapter.steering == pytest.approx(model.params.max_steer)
    assert adapter.can_tx_log[-1]["steering"] == pytest.approx(
        model.params.max_steer
    )


@pytest.mark.parametrize(
    "overrides",
    [
        {"wheel_base": 0.0},
        {"max_steer": math.pi / 2},
        {"max_accel": 0.0},
        {"max_decel": -1.0},
        {"min_speed": 2.0, "max_speed": 1.0},
    ],
)
def test_vehicle_params_reject_invalid_limits(overrides):
    values = {
        "wheel_base": 2.8,
        "max_steer": 0.5,
        "max_accel": 3.0,
        "max_decel": 6.0,
        "max_speed": 40.0,
        "min_speed": 0.0,
    }
    values.update(overrides)

    with pytest.raises(ValueError):
        VehicleParams(**values)


def test_state_and_command_reject_non_finite_values():
    with pytest.raises(ValueError):
        VehicleState(x=math.nan, y=0.0, psi=0.0, v=0.0)
    with pytest.raises(ValueError):
        VehicleCommand(delta=0.0, a=math.inf)
