from abc import ABC, abstractmethod

from .lqr import LQRController
from .models import LateralControlInput
from .stanley import StanleyController


class LateralController(ABC):

    @property
    @abstractmethod
    def name(self) -> str:
        ...

    @abstractmethod
    def update(
        self,
        control_input: LateralControlInput,
    ) -> float:
        ...


class StanleyLateralController(
    LateralController
):

    def __init__(
        self,
        k: float,
        softening: float,
        max_steer: float,
    ):

        self.controller = StanleyController(
            k=k,
            softening=softening,
            max_steer=max_steer,
        )

    @property
    def name(self) -> str:

        return "Stanley"

    def update(
        self,
        control_input: LateralControlInput,
    ) -> float:

        return self.controller.update(
            heading_error=(
                control_input.heading_error
            ),

            cross_track_error=(
                control_input.cross_track_error
            ),

            speed=(
                control_input.speed
            ),
        )


class LQRLateralController(
    LateralController
):

    def __init__(
        self,
        q_cross_track: float,
        q_heading: float,
        r_steer: float,
        max_steer: float,
    ):

        self.controller = LQRController(
            q_cross_track=q_cross_track,
            q_heading=q_heading,

            r_steer=r_steer,

            max_steer=max_steer,
        )

    @property
    def name(self) -> str:

        return "LQR"

    def update(
        self,
        control_input: LateralControlInput,
    ) -> float:

        return self.controller.update(
            cross_track_error=(
                control_input.cross_track_error
            ),

            heading_error=(
                control_input.heading_error
            ),

            speed=(
                control_input.speed
            ),

            wheel_base=(
                control_input.wheel_base
            ),

            dt=(
                control_input.dt
            ),

            reference_curvature=(
                control_input.reference_curvature
            ),
        )