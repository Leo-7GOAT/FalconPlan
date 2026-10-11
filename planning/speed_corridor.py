from dataclasses import dataclass

from .speed_profile import SpeedProfile
from .st_graph import STBoundary


@dataclass(frozen=True)
class SpeedCorridorPoint:

    t: float

    reference_s: float

    lower_s: float

    upper_s: float

    def __post_init__(self):

        if self.lower_s > self.upper_s:
            raise ValueError(
                "corridor lower_s must not "
                "exceed upper_s"
            )


@dataclass(frozen=True)
class SpeedCorridor:

    points: tuple[SpeedCorridorPoint, ...]

    def __post_init__(self):

        if len(self.points) == 0:
            raise ValueError(
                "SpeedCorridor must not be empty"
            )

        for i in range(
            len(self.points) - 1
        ):
            if (
                self.points[i + 1].t
                <= self.points[i].t
            ):
                raise ValueError(
                    "SpeedCorridor time must "
                    "be strictly increasing"
                )

    @property
    def times(self) -> tuple[float, ...]:
        return tuple(
            point.t
            for point in self.points
        )

    @property
    def reference_distances(
        self,
    ) -> tuple[float, ...]:
        return tuple(
            point.reference_s
            for point in self.points
        )

    @property
    def lower_bounds(
        self,
    ) -> tuple[float, ...]:
        return tuple(
            point.lower_s
            for point in self.points
        )

    @property
    def upper_bounds(
        self,
    ) -> tuple[float, ...]:
        return tuple(
            point.upper_s
            for point in self.points
        )

    @property
    def duration(self) -> float:
        return (
            self.points[-1].t
            - self.points[0].t
        )


def build_speed_corridor(
    *,
    reference_profile: SpeedProfile,

    boundaries: (
        tuple[STBoundary, ...]
        | list[STBoundary]
    ),

    min_s: float,

    max_s: float,

    boundary_epsilon: float = 0.0,
) -> SpeedCorridor:
    """Build a longitudinal s-corridor around a reference profile.

    For every reference time ``t`` the free-s interval is the intersection
    of ``[min_s, max_s]`` with the side of every ST boundary that the
    reference profile already occupies.  The reference is assumed to be a
    feasible (collision-free) plan, so it tells us whether to cut the
    corridor above or below each obstacle.
    """

    if min_s > max_s:
        raise ValueError(
            "min_s must not exceed max_s"
        )

    if boundary_epsilon < 0.0:
        raise ValueError(
            "boundary_epsilon must be "
            "non-negative"
        )

    points = []

    for ref_point in reference_profile.points:

        t = ref_point.t
        ref_s = ref_point.s

        lower_s = min_s
        upper_s = max_s

        for boundary in boundaries:

            limit = boundary.boundary_at(t)

            if limit is None:
                continue

            b_lo, b_hi = limit

            if ref_s <= b_lo:
                # Reference travels below (behind) the
                # obstacle: cut the upper bound.
                upper_s = min(
                    upper_s,
                    b_lo - boundary_epsilon,
                )

            elif ref_s >= b_hi:
                # Reference travels above (ahead of)
                # the obstacle: cut the lower bound.
                lower_s = max(
                    lower_s,
                    b_hi + boundary_epsilon,
                )

            else:
                raise ValueError(
                    "reference profile collides "
                    "with boundary "
                    f"'{boundary.obstacle_id}' "
                    f"at t={t}"
                )

        if lower_s > upper_s + 1e-9:
            raise ValueError(
                "speed corridor is infeasible: "
                f"lower_s={lower_s} > "
                f"upper_s={upper_s} at t={t}"
            )

        points.append(
            SpeedCorridorPoint(
                t=t,
                reference_s=ref_s,
                lower_s=lower_s,
                upper_s=upper_s,
            )
        )

    return SpeedCorridor(
        points=tuple(points)
    )
