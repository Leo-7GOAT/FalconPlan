import math

from hal import (
    SimulatorAdapter,
    MockHardwareAdapter,
)

from vehicle_model import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
    KinematicBicycleModel,
)


def run_vehicle(
        vehicle,
        command,
        dt=0.1,
        steps=10
):

    vehicle.apply_command(command)

    for _ in range(steps):
        state = vehicle.step(dt)

    return state


# =========================
# Vehicle Model
# =========================

params = VehicleParams(
    wheel_base=2.8,
    max_steer=math.radians(35),
    max_accel=3.0,
    max_decel=6.0,
    max_speed=40.0,
    min_speed=0.0
)

model = KinematicBicycleModel(
    params
)


# =========================
# Initial State
# =========================

initial_state = VehicleState(
    x=0.0,
    y=0.0,
    psi=0.0,
    v=10.0
)


# =========================
# Command
# =========================

command = VehicleCommand(
    delta=math.radians(10),
    a=2.0
)


# =========================
# Two Backends
# =========================

sim_vehicle = SimulatorAdapter(
    model=model,
    initial_state=initial_state
)

hw_vehicle = MockHardwareAdapter(
    model=model,
    initial_state=initial_state
)


# =========================
# Same Upper-Level Code
# =========================

sim_result = run_vehicle(
    sim_vehicle,
    command
)

hw_result = run_vehicle(
    hw_vehicle,
    command
)


print("Simulator:")
print(sim_result)

print("\nMock Hardware:")
print(hw_result)

print("\nMock CAN:")
print(hw_vehicle.can_tx_log)