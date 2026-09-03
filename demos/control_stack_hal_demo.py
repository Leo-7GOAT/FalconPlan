import math

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


# ============================================================
# 1. Same trajectory for both HAL implementations
# ============================================================

trajectory = ControlTrajectory(
    x=[
        float(i * 5)
        for i in range(40)
    ],

    y=[
        0.0
        for _ in range(40)
    ],

    yaw=[
        0.0
        for _ in range(40)
    ],

    speed=[
        15.0
        for _ in range(40)
    ],

    acceleration=[
        0.0
        for _ in range(40)
    ],

    curvature=[
        0.0
        for _ in range(40)
    ],
)


# ============================================================
# 2. Shared factory
# ============================================================

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
    params,
):

    return VehicleControlStack(
        trajectory=trajectory,

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

        # Deliberately start 1 m
        # to the right of target path.
        y=-1.0,

        psi=0.0,

        v=10.0,
    )


# ============================================================
# 3. This function knows ONLY VehicleHAL contract
# ============================================================

def run_vehicle(
    name,
    hal,
    stack,
):

    print()
    print("=" * 70)
    print(name)
    print("=" * 70)

    for step_index in range(
        320
    ):

        state = hal.get_state()

        result = stack.step(
            state
        )

        if (
            step_index % 10 == 0
            or result.safe_stop
            or result.trajectory_completed
        ):

            print(
                f"step={step_index:03d} "
                f"x={state.x:8.3f} "
                f"y={state.y:8.3f} "
                f"v={state.v:6.3f} "
                f"cte={result.cross_track_error:7.3f} "
                f"delta={math.degrees(result.actual_steering):7.3f} deg "
                f"a={result.command.a:7.3f}"
            )

        if result.trajectory_completed:

            print(
                "trajectory completed"
            )

            break

        # ====================================================
        # ControlStack outputs VehicleCommand.
        # HAL consumes VehicleCommand.
        # ====================================================

        hal.apply_command(
            result.command
        )

        hal.step(
            DT
        )

        if result.safe_stop:

            print(
                "WATCHDOG SAFE STOP:",
                result.watchdog_status.reason,
            )

            break


    final_state = (
        hal.get_state()
    )

    print()

    print(
        "final state:"
    )

    print(
        final_state
    )

    return final_state


# ============================================================
# 4. Simulator backend
# ============================================================

sim_params = make_params()

sim_hal = SimulatorAdapter(
    model=(
        KinematicBicycleModel(
            params=sim_params
        )
    ),

    initial_state=(
        make_initial_state()
    ),

    integration_method="rk4",
)


sim_stack = make_stack(
    sim_params
)


sim_final = run_vehicle(
    name="SIMULATOR ADAPTER",

    hal=sim_hal,

    stack=sim_stack,
)


# ============================================================
# 5. Mock hardware backend
# ============================================================

hardware_params = make_params()

hardware_hal = MockHardwareAdapter(
    model=(
        KinematicBicycleModel(
            params=hardware_params
        )
    ),

    initial_state=(
        make_initial_state()
    ),
)


hardware_stack = make_stack(
    hardware_params
)


hardware_final = run_vehicle(
    name="MOCK HARDWARE ADAPTER",

    hal=hardware_hal,

    stack=hardware_stack,
)


# ============================================================
# 6. Contract comparison
# ============================================================

print()
print("=" * 70)
print("HAL BACKEND COMPARISON")
print("=" * 70)

print(
    "Simulator final:"
)

print(
    sim_final
)

print()

print(
    "Mock hardware final:"
)

print(
    hardware_final
)

print()

print(
    "Mock CAN frame count:",
    len(
        hardware_hal.can_tx_log
    )
)

if hardware_hal.can_tx_log:

    print(
        "Last mock CAN frame:"
    )

    print(
        hardware_hal.can_tx_log[-1]
    )


print()
print(
    "state difference:"
)

print(
    "dx =",
    hardware_final.x
    - sim_final.x,
)

print(
    "dy =",
    hardware_final.y
    - sim_final.y,
)

print(
    "dpsi =",
    hardware_final.psi
    - sim_final.psi,
)

print(
    "dv =",
    hardware_final.v
    - sim_final.v,
)