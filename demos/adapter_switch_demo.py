import math

from hal import MockHardwareAdapter, SimulatorAdapter, VehicleHAL
from vehicle_model import (
    KinematicBicycleModel,
    VehicleCommand,
    VehicleParams,
    VehicleState,
)


def run_vehicle(
    vehicle: VehicleHAL,
    command: VehicleCommand,
    dt: float = 0.1,
    steps: int = 10,
) -> VehicleState:
    vehicle.apply_command(command)
    state = vehicle.get_state()
    for _ in range(steps):
        state = vehicle.step(dt)
    return state


def main() -> None:
    params = VehicleParams(
        wheel_base=2.8,
        max_steer=math.radians(35),
        max_accel=3.0,
        max_decel=6.0,
        max_speed=40.0,
    )
    model = KinematicBicycleModel(params)
    initial_state = VehicleState(x=0.0, y=0.0, psi=0.0, v=10.0)
    command = VehicleCommand(delta=math.radians(10), a=2.0)

    backends: list[tuple[str, VehicleHAL]] = [
        ("SimulatorAdapter", SimulatorAdapter(model, initial_state)),
        ("MockHardwareAdapter", MockHardwareAdapter(model, initial_state)),
    ]

    results = []
    for name, backend in backends:
        state = run_vehicle(backend, command)
        results.append(state)
        print(f"PASS: {name} -> {state}")

    assert results[0] == results[1]
    print("ADAPTER SWITCH CONTRACT PASS")


if __name__ == "__main__":
    main()
