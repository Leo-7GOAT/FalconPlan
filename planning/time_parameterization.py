from bisect import bisect_right
from dataclasses import dataclass
import math

from .speed_profile import SpeedProfile
from .trajectory import FrenetTrajectory


@dataclass(frozen=True)
class TimeParameterizedTrajectoryPoint:
    t: float
    progress_s: float
    path_s: float
    x: float
    y: float
    yaw: float
    curvature: float
    speed: float
    acceleration: float


@dataclass(frozen=True)
class TimeParameterizedTrajectory:
    points: tuple[
        TimeParameterizedTrajectoryPoint,
        ...
    ]

    def __post_init__(self):
        if len(self.points) == 0:
            raise ValueError(
                "time-parameterized trajectory must not be empty"
            )
        for i in range(len(self.points) - 1):
            if self.points[i + 1].t <= self.points[i].t:
                raise ValueError(
                    "trajectory time must be strictly increasing"
                )
            if (
                self.points[i + 1].progress_s
                < self.points[i].progress_s
            ):
                raise ValueError(
                    "trajectory progress must be non-decreasing"
                )

    @property
    def times(self) -> tuple[float, ...]:
        return tuple(point.t for point in self.points)

    @property
    def x(self) -> tuple[float, ...]:
        return tuple(point.x for point in self.points)

    @property
    def y(self) -> tuple[float, ...]:
        return tuple(point.y for point in self.points)

    @property
    def yaw(self) -> tuple[float, ...]:
        return tuple(point.yaw for point in self.points)

    @property
    def curvature(self) -> tuple[float, ...]:
        return tuple(point.curvature for point in self.points)

    @property
    def speed(self) -> tuple[float, ...]:
        return tuple(point.speed for point in self.points)

    @property
    def acceleration(self) -> tuple[float, ...]:
        return tuple(point.acceleration for point in self.points)


def _normalize_angle(
    angle: float,
) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def _validate_geometry(
    trajectory: FrenetTrajectory,
) -> None:
    fields = {
        "s": trajectory.s,
        "x": trajectory.x,
        "y": trajectory.y,
        "yaw": trajectory.yaw,
        "curvature": trajectory.curvature,
    }
    lengths = {
        name: len(values)
        for name, values in fields.items()
    }

    if len(set(lengths.values())) != 1:
        raise ValueError(
            "W04 geometry sample lengths "
            f"do not match: {lengths}"
        )
    if len(trajectory.s) < 2:
        raise ValueError(
            "W04 geometry needs at least two samples"
        )
    for i in range(len(trajectory.s) - 1):
        if trajectory.s[i + 1] <= trajectory.s[i]:
            raise ValueError(
                "W04 trajectory s must be strictly increasing"
            )


def _find_segment(
    samples_s: list[float],
    query_s: float,
) -> int:
    if (
        query_s < samples_s[0]
        or query_s > samples_s[-1]
    ):
        raise ValueError(
            "query s lies outside W04 trajectory range"
        )
    if query_s == samples_s[-1]:
        return len(samples_s) - 2

    index = bisect_right(
        samples_s,
        query_s,
    ) - 1
    return max(
        0,
        min(
            index,
            len(samples_s) - 2,
        ),
    )


def _interpolation_ratio(
    s0: float,
    s1: float,
    query_s: float,
) -> float:
    if s1 <= s0:
        raise ValueError(
            "interpolation samples must have increasing s"
        )
    return (
        (query_s - s0)
        / (s1 - s0)
    )


def _lerp(
    value0: float,
    value1: float,
    alpha: float,
) -> float:
    return value0 + alpha * (
        value1 - value0
    )


def _lerp_angle(
    yaw0: float,
    yaw1: float,
    alpha: float,
) -> float:
    delta_yaw = _normalize_angle(
        yaw1 - yaw0
    )
    return _normalize_angle(
        yaw0 + alpha * delta_yaw
    )


def parameterize_geometry_with_speed(
    geometry: FrenetTrajectory,
    speed_profile: SpeedProfile,
) -> TimeParameterizedTrajectory:
    _validate_geometry(
        geometry
    )

    geometry_start_s = geometry.s[0]
    available_progress = (
        geometry.s[-1] - geometry_start_s
    )
    requested_progress = (
        speed_profile.distances[-1]
    )

    if requested_progress > available_progress + 1e-9:
        raise ValueError(
            "SpeedProfile extends beyond available W04 geometry"
        )

    points = []

    for profile_point in speed_profile.points:
        path_s = (
            geometry_start_s
            + profile_point.s
        )

        segment_index = _find_segment(
            samples_s=geometry.s,
            query_s=path_s,
        )

        s0 = geometry.s[segment_index]
        s1 = geometry.s[segment_index + 1]

        alpha = _interpolation_ratio(
            s0=s0,
            s1=s1,
            query_s=path_s,
        )

        x = _lerp(
            geometry.x[segment_index],
            geometry.x[segment_index + 1],
            alpha,
        )
        y = _lerp(
            geometry.y[segment_index],
            geometry.y[segment_index + 1],
            alpha,
        )
        yaw = _lerp_angle(
            geometry.yaw[segment_index],
            geometry.yaw[segment_index + 1],
            alpha,
        )
        curvature = _lerp(
            geometry.curvature[segment_index],
            geometry.curvature[segment_index + 1],
            alpha,
        )

        points.append(
            TimeParameterizedTrajectoryPoint(
                t=profile_point.t,
                progress_s=profile_point.s,
                path_s=path_s,
                x=float(x),
                y=float(y),
                yaw=float(yaw),
                curvature=float(curvature),
                speed=profile_point.v,
                acceleration=profile_point.a,
            )
        )

    return TimeParameterizedTrajectory(
        points=tuple(points)
    )
