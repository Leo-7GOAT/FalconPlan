from abc import ABC, abstractmethod

from vehicle_model import (
    VehicleState,
    VehicleCommand,
)


class VehicleHAL(ABC):

    @abstractmethod
    def get_state(self) -> VehicleState:
        pass

    @abstractmethod
    def apply_command(
        self,
        command: VehicleCommand
    ) -> None:
        pass

    @abstractmethod
    def step(
        self,
        dt: float
    ) -> VehicleState:
        pass