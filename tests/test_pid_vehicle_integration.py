from control.pid import PIDController

from vehicle_model.kinematic_bicycle import (
    KinematicBicycleModel,
)

from vehicle_model.models import (
    VehicleState,
    VehicleCommand,
    VehicleParams,
)


def test_pid_tracks_target_speed_with_vehicle_model():

    params = VehicleParams(
        wheel_base=2.8,

        max_steer=0.5,

        max_accel=3.0,
        max_decel=6.0,

        max_speed=40.0,
        min_speed=0.0,
    )

    model = KinematicBicycleModel(
        params=params,
    )

    pid = PIDController(
        Kp=0.8,
        Ki=0.1,
        Kd=0.05,

        output_min=-params.max_decel,
        output_max=params.max_accel,
    )

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=20.0,
    )

    target_speed = 25.0

    dt = 0.05

    commands = []

    for _ in range(
        int(10.0 / dt)
    ):

        acceleration = pid.update(
            target=target_speed,
            measurement=state.v,
            dt=dt,
        )

        commands.append(
            acceleration
        )

        state = model.step(
            state=state,

            command=VehicleCommand(
                delta=0.0,
                a=acceleration,
            ),

            dt=dt,
            method="rk4",
        )

    assert abs(
        state.v
        - target_speed
    ) < 0.5

    assert max(commands) <= (
        params.max_accel
    )

    assert min(commands) >= (
        -params.max_decel
    )


def test_zero_steering_keeps_vehicle_on_straight_line():

    params = VehicleParams(
        wheel_base=2.8,

        max_steer=0.5,

        max_accel=3.0,
        max_decel=6.0,

        max_speed=40.0,
    )

    model = KinematicBicycleModel(
        params=params,
    )

    state = VehicleState(
        x=0.0,
        y=0.0,
        psi=0.0,
        v=20.0,
    )

    for _ in range(100):

        state = model.step(
            state=state,

            command=VehicleCommand(
                delta=0.0,
                a=0.0,
            ),

            dt=0.05,
            method="rk4",
        )

    assert abs(state.y) < 1e-10
    assert abs(state.psi) < 1e-10