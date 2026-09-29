import math
from collections.abc import Sequence

from .st_graph import STBoundary
from .st_grid import STGrid, STGridNode


def is_node_blocked(
    node: STGridNode,
    boundaries: Sequence[STBoundary],
) -> bool:
    for boundary in boundaries:
        interval = boundary.boundary_at(node.t)
        if interval is None:
            continue
        lower_s, upper_s = interval
        if lower_s <= node.s <= upper_s:
            return True
    return False


def build_blocked_mask(
    grid: STGrid,
    boundaries: Sequence[STBoundary],
) -> tuple[tuple[bool, ...], ...]:
    rows = []
    for time_index in range(grid.num_time_steps):
        row = []
        for distance_index in range(grid.num_distance_steps):
            node = grid.node(
                time_index=time_index,
                distance_index=distance_index,
            )
            row.append(
                is_node_blocked(
                    node=node,
                    boundaries=boundaries,
                )
            )
        rows.append(tuple(row))
    return tuple(rows)


def interpolate_edge_s(
    start: STGridNode,
    end: STGridNode,
    t: float,
) -> float:
    if end.t <= start.t:
        raise ValueError(
            "edge end time must be greater than start time"
        )
    if not start.t <= t <= end.t:
        raise ValueError("query time must lie inside edge")

    alpha = (t - start.t) / (end.t - start.t)
    s = start.s + alpha * (end.s - start.s)
    return float(s)


def is_edge_blocked(
    start: STGridNode,
    end: STGridNode,
    boundaries: Sequence[STBoundary],
    sample_dt: float = 0.05,
) -> bool:
    if end.t <= start.t:
        raise ValueError(
            "edge end time must be greater than start time"
        )
    if sample_dt <= 0.0:
        raise ValueError("sample_dt must be positive")

    duration = end.t - start.t
    num_segments = max(
        1,
        math.ceil(duration / sample_dt),
    )

    for sample_index in range(num_segments + 1):
        alpha = sample_index / num_segments
        t = start.t + alpha * duration
        ego_s = start.s + alpha * (end.s - start.s)

        for boundary in boundaries:
            interval = boundary.boundary_at(t)
            if interval is None:
                continue
            lower_s, upper_s = interval
            if lower_s <= ego_s <= upper_s:
                return True

    return False
