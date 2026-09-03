from vehicle_model.models import (
    VehicleCommand,
    VehicleParams,
    VehicleState,
)

from .actuator import SteeringActuator
from .delay import SteeringCommandDelay

from .lateral_controller import (
    LateralController,
)

from .models import (
    ControlStepResult,
    ControlTrajectory,
    LateralControlInput,
)

from .pid import PIDController

from .trajectory_tracker import (
    TrajectoryTracker,
)

from .watchdog import (
    TrackingWatchdog,
    WatchdogConfig,
)


class VehicleControlStack:

    def __init__(
        self,
        trajectory: ControlTrajectory,
        vehicle_params: VehicleParams,

        lateral_controller: LateralController,
        longitudinal_controller: PIDController,

        dt: float,

        steering_delay_seconds: float,
        max_steer_rate: float,

        watchdog_config: WatchdogConfig | None = None,

        search_window: int = 20,
    ):

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        if max_steer_rate <= 0.0:
            raise ValueError(
                "max_steer_rate must be positive"
            )

        self.trajectory = trajectory

        self.vehicle_params = (
            vehicle_params
        )

        self.lateral_controller = (
            lateral_controller
        )

        self.longitudinal_controller = (
            longitudinal_controller
        )

        self.dt = dt

        # ----------------------------------------------------
        # Trajectory tracking
        # ----------------------------------------------------

        self.tracker = TrajectoryTracker(
            trajectory_x=trajectory.x,
            trajectory_y=trajectory.y,
            trajectory_yaw=trajectory.yaw,

            wheel_base=(
                vehicle_params.wheel_base
            ),

            search_window=search_window,
        )

        # ----------------------------------------------------
        # Command delay
        # ----------------------------------------------------

        self.delay = SteeringCommandDelay(
            delay_seconds=(
                steering_delay_seconds
            ),

            dt=dt,

            initial_steer=0.0,
        )

        # ----------------------------------------------------
        # Physical steering actuator
        # ----------------------------------------------------

        self.actuator = SteeringActuator(
            max_steer=(
                vehicle_params.max_steer
            ),

            max_steer_rate=(
                max_steer_rate
            ),
        )

        # ----------------------------------------------------
        # Safety watchdog
        # ----------------------------------------------------

        if watchdog_config is None:

            watchdog_config = (
                WatchdogConfig()
            )

        self.watchdog = TrackingWatchdog(
            config=watchdog_config
        )

    # ========================================================
    # Reset complete stateful control stack
    # ========================================================

    def reset(self) -> None:

        self.longitudinal_controller.reset()

        self.tracker.reset()

        self.delay.reset()

        self.actuator.reset()

        self.watchdog.reset()

    # ========================================================
    # One control cycle
    # ========================================================

    def step(
        self,
        state: VehicleState,
    ) -> ControlStepResult:

        # ----------------------------------------------------
        # 1. Tracking geometry
        # ----------------------------------------------------

        (
            nearest_index,
            heading_error,
            cross_track_error,
        ) = self.tracker.compute_errors(
            state
        )

        target_speed = (
            self.trajectory.speed[
                nearest_index
            ]
        )

        target_acceleration = (
            self.trajectory.acceleration[
                nearest_index
            ]
        )

        reference_curvature = (
            self.trajectory.curvature[
                nearest_index
            ]
        )

        trajectory_completed = (
            nearest_index
            >= len(self.trajectory) - 2
        )

        # ----------------------------------------------------
        # 2. Runtime safety watchdog
        # ----------------------------------------------------

        watchdog_status = (
            self.watchdog.update(
                cross_track_error=(
                    cross_track_error
                ),

                heading_error=(
                    heading_error
                ),
            )
        )

        # ====================================================
        # 3. FAIL-SAFE
        #
        # Watchdog commands:
        #
        # steering -> gradually return toward zero
        # acceleration -> maximum braking
        #
        # Safety command deliberately bypasses delayed
        # controller command queue.
        # ====================================================

        if watchdog_status.tripped:

            actual_steering = (
                self.actuator.update(
                    target_steer=0.0,
                    dt=self.dt,
                )
            )

            command = VehicleCommand(
                delta=actual_steering,

                a=(
                    -self.vehicle_params
                    .max_decel
                ),
            )

            return ControlStepResult(
                command=command,

                controller_name=(
                    self.lateral_controller
                    .name
                ),

                nearest_index=(
                    nearest_index
                ),

                trajectory_completed=(
                    trajectory_completed
                ),

                cross_track_error=(
                    cross_track_error
                ),

                heading_error=(
                    heading_error
                ),

                target_speed=(
                    target_speed
                ),

                target_acceleration=(
                    target_acceleration
                ),

                reference_curvature=(
                    reference_curvature
                ),

                requested_steering=0.0,
                delayed_steering=0.0,

                actual_steering=(
                    actual_steering
                ),

                watchdog_status=(
                    watchdog_status
                ),

                safe_stop=True,
            )

        # ====================================================
        # 4. End of planned trajectory
        # ====================================================

        if trajectory_completed:

            actual_steering = (
                self.actuator.update(
                    target_steer=0.0,
                    dt=self.dt,
                )
            )

            command = VehicleCommand(
                delta=actual_steering,
                a=0.0,
            )

            return ControlStepResult(
                command=command,

                controller_name=(
                    self.lateral_controller
                    .name
                ),

                nearest_index=(
                    nearest_index
                ),

                trajectory_completed=True,

                cross_track_error=(
                    cross_track_error
                ),

                heading_error=(
                    heading_error
                ),

                target_speed=(
                    target_speed
                ),

                target_acceleration=(
                    target_acceleration
                ),

                reference_curvature=(
                    reference_curvature
                ),

                requested_steering=0.0,
                delayed_steering=0.0,

                actual_steering=(
                    actual_steering
                ),

                watchdog_status=(
                    watchdog_status
                ),

                safe_stop=False,
            )

        # ----------------------------------------------------
        # 5. Longitudinal:
        #
        # planned acceleration FF
        # +
        # PID speed feedback
        # ----------------------------------------------------

        acceleration = (
            self.longitudinal_controller
            .update(
                target=target_speed,

                measurement=state.v,

                dt=self.dt,

                feedforward=(
                    target_acceleration
                ),
            )
        )

        # ----------------------------------------------------
        # 6. Lateral controller
        # ----------------------------------------------------

        lateral_input = (
            LateralControlInput(
                cross_track_error=(
                    cross_track_error
                ),

                heading_error=(
                    heading_error
                ),

                speed=state.v,

                wheel_base=(
                    self.vehicle_params
                    .wheel_base
                ),

                dt=self.dt,

                reference_curvature=(
                    reference_curvature
                ),
            )
        )

        requested_steering = (
            self.lateral_controller
            .update(
                lateral_input
            )
        )

        # ----------------------------------------------------
        # 7. Steering command delay
        # ----------------------------------------------------

        delayed_steering = (
            self.delay.update(
                requested_steer=(
                    requested_steering
                )
            )
        )

        # ----------------------------------------------------
        # 8. Physical rate limit + saturation
        # ----------------------------------------------------

        actual_steering = (
            self.actuator.update(
                target_steer=(
                    delayed_steering
                ),

                dt=self.dt,
            )
        )

        # ----------------------------------------------------
        # 9. Unified vehicle command
        # ----------------------------------------------------

        command = VehicleCommand(
            delta=actual_steering,
            a=acceleration,
        )

        # ----------------------------------------------------
        # 10. Diagnostics
        # ----------------------------------------------------

        return ControlStepResult(
            command=command,

            controller_name=(
                self.lateral_controller
                .name
            ),

            nearest_index=(
                nearest_index
            ),

            trajectory_completed=False,

            cross_track_error=(
                cross_track_error
            ),

            heading_error=(
                heading_error
            ),

            target_speed=(
                target_speed
            ),

            target_acceleration=(
                target_acceleration
            ),

            reference_curvature=(
                reference_curvature
            ),

            requested_steering=(
                requested_steering
            ),

            delayed_steering=(
                delayed_steering
            ),

            actual_steering=(
                actual_steering
            ),

            watchdog_status=(
                watchdog_status
            ),

            safe_stop=False,
        )