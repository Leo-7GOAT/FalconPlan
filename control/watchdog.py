import math
from dataclasses import dataclass


@dataclass(frozen=True)
class WatchdogConfig:
    max_cross_track_error: float = 2.5

    max_heading_error_deg: float = 20.0

    violation_frames: int = 3


@dataclass(frozen=True)
class WatchdogStatus:
    tripped: bool

    violation_count: int

    reason: str


class TrackingWatchdog:

    def __init__(
        self,
        config: WatchdogConfig,
    ):
        if config.max_cross_track_error <= 0.0:
            raise ValueError(
                "max_cross_track_error must be positive"
            )

        if config.max_heading_error_deg <= 0.0:
            raise ValueError(
                "max_heading_error_deg must be positive"
            )

        if config.violation_frames <= 0:
            raise ValueError(
                "violation_frames must be positive"
            )

        self.config = config

        self.violation_count = 0
        self.tripped = False

        self.reason = ""

    def reset(self) -> None:

        self.violation_count = 0
        self.tripped = False

        self.reason = ""

    def update(
        self,
        cross_track_error: float,
        heading_error: float,
    ) -> WatchdogStatus:

        # Once tripped, stay tripped until reset().
        if self.tripped:

            return WatchdogStatus(
                tripped=True,

                violation_count=(
                    self.violation_count
                ),

                reason=self.reason,
            )

        heading_error_deg = abs(
            math.degrees(
                heading_error
            )
        )

        cte_violation = (
            abs(cross_track_error)
            > self.config.max_cross_track_error
        )

        heading_violation = (
            heading_error_deg
            > self.config.max_heading_error_deg
        )

        unsafe = (
            cte_violation
            or heading_violation
        )

        if unsafe:

            self.violation_count += 1

        else:

            # Unsafe condition must persist.
            self.violation_count = 0

        if (
            self.violation_count
            >= self.config.violation_frames
        ):

            self.tripped = True

            if (
                cte_violation
                and heading_violation
            ):

                self.reason = (
                    "cross-track and heading "
                    "errors exceeded limits"
                )

            elif cte_violation:

                self.reason = (
                    "cross-track error "
                    "exceeded limit"
                )

            else:

                self.reason = (
                    "heading error "
                    "exceeded limit"
                )

        return WatchdogStatus(
            tripped=self.tripped,

            violation_count=(
                self.violation_count
            ),

            reason=self.reason,
        )