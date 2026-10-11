
import math
from collections.abc import Sequence

from .speed_profile import SpeedProfile
from .st_graph import STBoundary


def is_speed_profile_st_safe(
    profile: SpeedProfile,
    boundaries: Sequence[STBoundary],
    sample_dt: float = 0.01,
    tolerance: float = 1e-7,
) -> bool:
    """
    Check ST obstacle occupancy along piecewise
    constant-jerk trajectories.

    The check uses dense temporal sampling.
    It is not a formal continuous-time
    collision-free certificate.
    """

    if sample_dt <= 0.0:
        raise ValueError("sample_dt must be positive")

    if tolerance < 0.0:
        raise ValueError("tolerance must be non-negative")

    points = profile.points

    # Even a one-point profile must be collision-free.
    if len(points) == 1:
        point = points[0]

        for boundary in boundaries:
            interval = boundary.boundary_at(point.t)

            if interval is None:
                continue

            lower, upper = interval

            if lower - tolerance <= point.s <= upper + tolerance:
                return False

        return True

    for i in range(len(points) - 1):

        start = points[i]
        end = points[i + 1]

        dt = end.t - start.t

        if dt <= 0.0:
            raise ValueError("profile time must increase")

        # Constant jerk within this segment.
        jerk = (end.a - start.a) / dt

        num_segments = max(
            1,
            math.ceil(dt / sample_dt),
        )

        for j in range(num_segments + 1):

            tau = dt * j / num_segments
            t = start.t + tau

            # Cubic longitudinal position.
            s = (
                start.s
                + start.v * tau
                + 0.5 * start.a * tau**2
                + jerk * tau**3 / 6.0
            )

            for boundary in boundaries:

                interval = boundary.boundary_at(t)

                if interval is None:
                    continue

                lower, upper = interval

                if lower - tolerance <= s <= upper + tolerance:
                    return False

    return True
