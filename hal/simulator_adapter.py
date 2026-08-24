from vehicle_model import (
    VehicleState,
    VehicleCommand,
    KinematicBicycleModel,
)

from .base import VehicleHAL


class SimulatorAdapter(VehicleHAL):

    def __init__(
        self,
        model: KinematicBicycleModel,
        initial_state: VehicleState,
        integration_method: str = "rk4"
    ):
        self.model = model
        self.state = initial_state

        self.command = VehicleCommand(
            delta=0.0,
            a=0.0
        )

        self.integration_method = integration_method

    def get_state(self) -> VehicleState:
        return self.state

    def apply_command(
        self,
        command: VehicleCommand
    ) -> None:
        self.command = command

    def step(
        self,
        dt: float
    ) -> VehicleState:

        self.state = self.model.step(
            self.state,
            self.command,
            dt,
            method=self.integration_method
        )

        return self.state