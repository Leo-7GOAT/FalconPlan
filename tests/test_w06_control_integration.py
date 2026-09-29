import math

from control.control_stack import VehicleControlStack
from control.lateral_controller import StanleyLateralController
from control.models import ControlTrajectory
from control.pid import PIDController
from control.watchdog import WatchdogConfig
from hal import SimulatorAdapter
from planning.speed_profile import (
    SpeedProfile,
    SpeedProfilePoint,
)
from planning.time_parameterization import (
    parameterize_geometry_with_speed,
)
from planning.trajectory import FrenetTrajectory
from vehicle_model import (
    KinematicBicycleModel,
    VehicleParams,
    VehicleState,
)


DT = 0.05


def make_geometry():
    trajectory = FrenetTrajectory()
    trajectory.s = [
        0.0, 10.0, 20.0, 30.0, 40.0, 50.0
    ]
    trajectory.x = [
        0.0, 10.0, 20.0, 30.0, 40.0, 50.0
    ]
    trajectory.y = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
    trajectory.yaw = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
    trajectory.curvature = [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
    return trajectory


def make_speed_profile():
    return SpeedProfile(
        points=tuple(
            SpeedProfilePoint(
                t=float(i),
                s=float(i * 10),
                v=10.0,
                a=0.0,
            )
            for i in range(6)
        )
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


def test_w06_trajectory_runs_through_w05_control_stack():
    time_trajectory = parameterize_geometry_with_speed(
        geometry=make_geometry(),
        speed_profile=make_speed_profile(),
    )
    control_trajectory = (
        ControlTrajectory
        .from_time_parameterized_trajectory(
            time_trajectory
        )
    )

    params = make_params()
    hal = SimulatorAdapter(
        model=KinematicBicycleModel(
            params=params
        ),
        initial_state=VehicleState(
            x=0.0,
            y=-0.5,
            psi=0.0,
            v=10.0,
        ),
        integration_method="rk4",
    )

    stack = VehicleControlStack(
        trajectory=control_trajectory,
        vehicle_params=params,
        lateral_controller=StanleyLateralController(
            k=2.0,
            softening=1.0,
            max_steer=params.max_steer,
        ),
        longitudinal_controller=PIDController(
            Kp=0.8,
            Ki=0.1,
            Kd=0.05,
            output_min=-params.max_decel,
            output_max=params.max_accel,
        ),
        dt=DT,
        steering_delay_seconds=0.0,
        max_steer_rate=math.radians(90.0),
        watchdog_config=WatchdogConfig(
            max_cross_track_error=2.5,
            max_heading_error_deg=20.0,
            violation_frames=3,
        ),
    )

    results = []

    for _ in range(200):
        state = hal.get_state()
        result = stack.step(state)
        results.append(result)

        if result.trajectory_completed:
            break

        assert result.safe_stop is False

        hal.apply_command(
            result.command
        )
        hal.step(DT)

    assert len(results) > 0
    assert results[-1].trajectory_completed is True

    final_state = hal.get_state()
    final_front_axle_x = (
        final_state.x
        + params.wheel_base
        * math.cos(final_state.psi)
    )

    assert final_front_axle_x > 35.0
    assert final_state.x > 30.0
    assert abs(final_state.y) < 0.1
