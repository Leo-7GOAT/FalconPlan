from vehicle_model import (
    VehicleState,
    VehicleCommand,
    KinematicBicycleModel,
)

from .base import VehicleHAL


class MockHardwareAdapter(VehicleHAL):

    def __init__(
        self,
        model: KinematicBicycleModel,
        initial_state: VehicleState
    ):
        self.model = model
        self.state = initial_state

        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0

        self.can_tx_log = []

    def get_state(self) -> VehicleState:
        return self.state

    def apply_command(
        self,
        command: VehicleCommand
    ) -> None:

        # steering actuator
        self.steering = command.delta

        # longitudinal actuator
        if command.a >= 0.0:

            self.throttle = min(
                command.a / self.model.params.max_accel,
                1.0
            )

            self.brake = 0.0

        else:

            self.throttle = 0.0

            self.brake = min(
                -command.a / self.model.params.max_decel,
                1.0
            )

        # mock CAN frame
        self.can_tx_log.append(
            {
                "steering": self.steering,
                "throttle": self.throttle,
                "brake": self.brake,
            }
        )

    def step(
        self,
        dt: float
    ) -> VehicleState:

        # mock actuator -> equivalent acceleration
        acceleration = (
            self.throttle * self.model.params.max_accel
            -
            self.brake * self.model.params.max_decel
        )

        command = VehicleCommand(
            delta=self.steering,
            a=acceleration
        )

        self.state = self.model.step(
            self.state,
            command,
            dt
        )

        return self.state