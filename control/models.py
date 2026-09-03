from dataclasses import dataclass
from typing import Sequence

from vehicle_model.models import VehicleCommand

from .watchdog import WatchdogStatus


@dataclass(frozen=True)
class ControlTrajectory:
    """
    Controller-side trajectory representation.

    This intentionally hides Frenet/planner-specific names such as
    s_d and s_dd from the control layer.
    """

    x: Sequence[float]
    y: Sequence[float]
    yaw: Sequence[float]

    speed: Sequence[float]
    acceleration: Sequence[float]

    curvature: Sequence[float]

    def __post_init__(self):

        fields = (
            "x",
            "y",
            "yaw",
            "speed",
            "acceleration",
            "curvature",
        )

        for name in fields:

            values = tuple(
                float(value)
                for value in getattr(
                    self,
                    name,
                )
            )

            object.__setattr__(
                self,
                name,
                values,
            )

        lengths = {
            len(getattr(self, name))
            for name in fields
        }

        if lengths == {0}:
            raise ValueError(
                "trajectory must not be empty"
            )

        if len(lengths) != 1:
            raise ValueError(
                "trajectory lengths must match"
            )

    @classmethod
    def from_planner_trajectory(
        cls,
        trajectory,
    ):

        return cls(
            x=trajectory.x,
            y=trajectory.y,
            yaw=trajectory.yaw,

            speed=trajectory.s_d,
            acceleration=trajectory.s_dd,

            curvature=trajectory.curvature,
        )

    def __len__(self) -> int:

        return len(
            self.x
        )


@dataclass(frozen=True)
class LateralControlInput:

    cross_track_error: float
    heading_error: float

    speed: float

    wheel_base: float
    dt: float

    reference_curvature: float


@dataclass(frozen=True)
class ControlStepResult:

    command: VehicleCommand

    controller_name: str

    nearest_index: int
    trajectory_completed: bool

    cross_track_error: float
    heading_error: float

    target_speed: float
    target_acceleration: float

    reference_curvature: float

    requested_steering: float
    delayed_steering: float
    actual_steering: float

    watchdog_status: WatchdogStatus

    safe_stop: bool