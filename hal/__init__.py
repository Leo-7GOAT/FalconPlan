from .base import VehicleHAL

from .simulator_adapter import (
    SimulatorAdapter,
)
from .mock_hardware_adapter import (
    MockHardwareAdapter,
)

# 暂时保留旧名字，避免以前的 demo / test 全炸
KinematicVehicleHAL = SimulatorAdapter


__all__ = [
    "VehicleHAL",
    "SimulatorAdapter",
    "MockHardwareAdapter",
]