from dataclasses import dataclass

from .st_dp import DPSpeedPlan
from .st_transition import (
    edge_speed,
    initial_edge_acceleration,
    transition_acceleration,
)


@dataclass(frozen=True)
class SpeedProfilePoint:
    t: float
    s: float
    v: float
    a: float


@dataclass(frozen=True)
class SpeedProfile:
    points: tuple[SpeedProfilePoint, ...]

    def __post_init__(self):
        if len(self.points) == 0:
            raise ValueError(
                "SpeedProfile must not be empty"
            )
        for i in range(len(self.points) - 1):
            if self.points[i + 1].t <= self.points[i].t:
                raise ValueError(
                    "SpeedProfile time must be strictly increasing"
                )
            if self.points[i + 1].s < self.points[i].s:
                raise ValueError(
                    "SpeedProfile distance must be non-decreasing"
                )
        for point in self.points:
            if point.v < 0.0:
                raise ValueError(
                    "SpeedProfile speed must be non-negative"
                )

    @property
    def times(self) -> tuple[float, ...]:
        return tuple(point.t for point in self.points)

    @property
    def distances(self) -> tuple[float, ...]:
        return tuple(point.s for point in self.points)

    @property
    def speeds(self) -> tuple[float, ...]:
        return tuple(point.v for point in self.points)

    @property
    def accelerations(self) -> tuple[float, ...]:
        return tuple(point.a for point in self.points)

    @property
    def duration(self) -> float:
        return self.points[-1].t - self.points[0].t


def speed_profile_from_dp_plan(
    plan: DPSpeedPlan,
    initial_speed: float,
    initial_acceleration: float = 0.0,
) -> SpeedProfile:
    if initial_speed < 0.0:
        raise ValueError(
            "initial_speed must be non-negative"
        )
    if len(plan.nodes) < 2:
        raise ValueError(
            "DP speed plan needs at least two nodes"
        )

    points = []

    first_node = plan.nodes[0]
    points.append(
        SpeedProfilePoint(
            t=first_node.t,
            s=first_node.s,
            v=initial_speed,
            a=initial_acceleration,
        )
    )

    second_node = plan.nodes[1]
    first_speed = edge_speed(
        start=first_node,
        end=second_node,
    )
    first_acceleration = initial_edge_acceleration(
        initial_speed=initial_speed,
        start=first_node,
        end=second_node,
    )
    points.append(
        SpeedProfilePoint(
            t=second_node.t,
            s=second_node.s,
            v=first_speed,
            a=first_acceleration,
        )
    )

    for i in range(2, len(plan.nodes)):
        previous = plan.nodes[i - 2]
        current = plan.nodes[i - 1]
        next_node = plan.nodes[i]

        speed = edge_speed(
            start=current,
            end=next_node,
        )
        acceleration = transition_acceleration(
            previous=previous,
            current=current,
            next_node=next_node,
        )

        points.append(
            SpeedProfilePoint(
                t=next_node.t,
                s=next_node.s,
                v=speed,
                a=acceleration,
            )
        )

    return SpeedProfile(
        points=tuple(points)
    )
