from dataclasses import dataclass

from .st_grid import STGridNode


@dataclass(frozen=True)
class STKinematicLimits:
    min_speed: float = 0.0
    max_speed: float = 25.0
    max_accel: float = 3.0
    max_decel: float = 6.0

    def __post_init__(self):
        if self.min_speed < 0.0:
            raise ValueError("min_speed must be non-negative")
        if self.max_speed <= 0.0:
            raise ValueError("max_speed must be positive")
        if self.min_speed > self.max_speed:
            raise ValueError("min_speed must not exceed max_speed")
        if self.max_accel <= 0.0:
            raise ValueError("max_accel must be positive")
        if self.max_decel <= 0.0:
            raise ValueError("max_decel must be positive")


def edge_speed(
    start: STGridNode,
    end: STGridNode,
) -> float:
    dt = end.t - start.t
    if dt <= 0.0:
        raise ValueError(
            "edge end time must be greater than start time"
        )
    return (end.s - start.s) / dt


def initial_edge_acceleration(
    initial_speed: float,
    start: STGridNode,
    end: STGridNode,
) -> float:
    if initial_speed < 0.0:
        raise ValueError("initial_speed must be non-negative")

    dt = end.t - start.t
    if dt <= 0.0:
        raise ValueError(
            "edge end time must be greater than start time"
        )

    average_speed = edge_speed(
        start=start,
        end=end,
    )

    return (
        average_speed - initial_speed
    ) / (0.5 * dt)


def is_initial_edge_acceleration_feasible(
    initial_speed: float,
    start: STGridNode,
    end: STGridNode,
    limits: STKinematicLimits,
) -> bool:
    acceleration = initial_edge_acceleration(
        initial_speed=initial_speed,
        start=start,
        end=end,
    )
    return (
        -limits.max_decel
        <= acceleration
        <= limits.max_accel
    )


def is_edge_speed_feasible(
    start: STGridNode,
    end: STGridNode,
    limits: STKinematicLimits,
) -> bool:
    speed = edge_speed(
        start=start,
        end=end,
    )
    return (
        limits.min_speed
        <= speed
        <= limits.max_speed
    )


def transition_acceleration(
    previous: STGridNode,
    current: STGridNode,
    next_node: STGridNode,
) -> float:
    previous_speed = edge_speed(
        start=previous,
        end=current,
    )
    next_speed = edge_speed(
        start=current,
        end=next_node,
    )

    previous_mid_time = 0.5 * (previous.t + current.t)
    next_mid_time = 0.5 * (current.t + next_node.t)
    delta_time = next_mid_time - previous_mid_time

    if delta_time <= 0.0:
        raise ValueError("transition time must increase")

    return (
        next_speed - previous_speed
    ) / delta_time


def is_transition_acceleration_feasible(
    previous: STGridNode,
    current: STGridNode,
    next_node: STGridNode,
    limits: STKinematicLimits,
) -> bool:
    acceleration = transition_acceleration(
        previous=previous,
        current=current,
        next_node=next_node,
    )
    return (
        -limits.max_decel
        <= acceleration
        <= limits.max_accel
    )


def is_transition_kinematically_feasible(
    current: STGridNode,
    next_node: STGridNode,
    limits: STKinematicLimits,
    previous: STGridNode | None = None,
) -> bool:
    if not is_edge_speed_feasible(
        start=current,
        end=next_node,
        limits=limits,
    ):
        return False

    if previous is None:
        return True

    return is_transition_acceleration_feasible(
        previous=previous,
        current=current,
        next_node=next_node,
        limits=limits,
    )
