import math

import pytest

from coordinate_system import (
    ReferenceLine,
    FrenetTransformer,
    FrenetState,
    WorldState,
)

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)


def world_to_vehicle_state(
        world_state: WorldState,
        v: float
) -> VehicleState:

    return VehicleState(
        x=world_state.x,
        y=world_state.y,
        psi=world_state.theta,
        v=v
    )


def vehicle_to_world_state(
        vehicle_state: VehicleState
) -> WorldState:

    return WorldState(
        x=vehicle_state.x,
        y=vehicle_state.y,
        theta=vehicle_state.psi
    )


@pytest.fixture
def transformer():

    reference_points = [
        (0.0, 0.0),
        (5.0, 1.0),
        (10.0, 4.0),
        (15.0, 8.0),
        (20.0, 10.0),
    ]

    reference_line = ReferenceLine(
        reference_points
    )

    return FrenetTransformer(
        reference_line
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

def test_frenet_vehicle_single_step(
        transformer,
        model
):

    # 1. 初始 Frenet 状态
    frenet_state = FrenetState(
        s=10.0,
        d=0.0,
        d_prime=0.0
    )

    # 2. Frenet -> World
    world_state = transformer.frenet_to_world(
        frenet_state
    )

    # 3. World -> Vehicle
    vehicle_state = world_to_vehicle_state(
        world_state,
        v=5.0
    )

    command = VehicleCommand(
        delta=0.0,
        a=0.0
    )

    # 4. Vehicle Model 前进一步
    next_vehicle_state = model.step(
        vehicle_state,
        command,
        dt=0.1
    )

    # 5. Vehicle -> World
    next_world_state = vehicle_to_world_state(
        next_vehicle_state
    )

    # 6. World -> Frenet
    next_frenet_state = transformer.world_to_frenet(
        next_world_state
    )

    # -------------------------
    # 验证
    # -------------------------

    assert next_frenet_state.s == pytest.approx(
        10.5,
        abs=0.01
    )

    assert abs(next_frenet_state.d) < 0.02

    assert math.isfinite(
        next_frenet_state.s
    )

    assert math.isfinite(
        next_frenet_state.d
    )

    assert math.isfinite(
        next_frenet_state.d_prime
    )


def test_world_vehicle_adapter_round_trip():

    original = WorldState(
        x=12.3,
        y=-4.5,
        theta=0.8
    )

    vehicle = world_to_vehicle_state(
        original,
        v=7.0
    )

    recovered = vehicle_to_world_state(
        vehicle
    )

    assert recovered.x == pytest.approx(
        original.x
    )

    assert recovered.y == pytest.approx(
        original.y
    )

    assert recovered.theta == pytest.approx(
        original.theta
    )

    assert vehicle.v == pytest.approx(
        7.0
    )

def test_open_loop_one_second(
        transformer,
        model
):

    frenet_state = FrenetState(
        s=10.0,
        d=0.0,
        d_prime=0.0
    )

    world_state = transformer.frenet_to_world(
        frenet_state
    )

    state = world_to_vehicle_state(
        world_state,
        v=5.0
    )

    command = VehicleCommand(
        delta=0.0,
        a=0.0
    )

    dt = 0.1

    for _ in range(10):

        state = model.step(
            state,
            command,
            dt
        )

    world = vehicle_to_world_state(
        state
    )

    frenet = transformer.world_to_frenet(
        world
    )

    assert frenet.s > 14.0
    assert frenet.s < 16.0

    assert abs(frenet.d) < 1.0