from collections.abc import Sequence
from dataclasses import dataclass

from .st_graph import STBoundary
from .st_grid import STGrid, STGridNode
from .st_occupancy import is_edge_blocked, is_node_blocked
from .st_transition import (
    STKinematicLimits,
    edge_speed,
    initial_edge_acceleration,
    is_edge_speed_feasible,
    is_initial_edge_acceleration_feasible,
    is_transition_acceleration_feasible,
    transition_acceleration,
)


@dataclass(frozen=True)
class DPCostWeights:
    speed_error: float = 1.0
    acceleration: float = 0.2

    def __post_init__(self):
        if self.speed_error < 0.0:
            raise ValueError(
                "speed_error weight must be non-negative"
            )
        if self.acceleration < 0.0:
            raise ValueError(
                "acceleration weight must be non-negative"
            )


@dataclass(frozen=True)
class DPStateKey:
    time_index: int
    previous_distance_index: int
    distance_index: int


@dataclass(frozen=True)
class DPRecord:
    cost: float
    parent: DPStateKey | None


@dataclass(frozen=True)
class DPSpeedPlan:
    nodes: tuple[STGridNode, ...]
    total_cost: float

    @property
    def speeds(self) -> tuple[float, ...]:
        return tuple(
            edge_speed(
                start=self.nodes[i],
                end=self.nodes[i + 1],
            )
            for i in range(len(self.nodes) - 1)
        )

    @property
    def accelerations(self) -> tuple[float, ...]:
        return tuple(
            transition_acceleration(
                previous=self.nodes[i],
                current=self.nodes[i + 1],
                next_node=self.nodes[i + 2],
            )
            for i in range(len(self.nodes) - 2)
        )


class DPSpeedPlanner:

    def __init__(
        self,
        grid: STGrid,
        boundaries: Sequence[STBoundary],
        limits: STKinematicLimits,
        initial_speed: float,
        desired_speed: float,
        weights: DPCostWeights = DPCostWeights(),
        edge_sample_dt: float = 0.05,
        start_distance_index: int = 0,
    ):
        if grid.num_time_steps < 2:
            raise ValueError(
                "DP grid needs at least two time steps"
            )
        if initial_speed < 0.0:
            raise ValueError(
                "initial_speed must be non-negative"
            )
        if desired_speed < 0.0:
            raise ValueError(
                "desired_speed must be non-negative"
            )
        if edge_sample_dt <= 0.0:
            raise ValueError(
                "edge_sample_dt must be positive"
            )
        if not 0 <= start_distance_index < grid.num_distance_steps:
            raise ValueError(
                "start_distance_index out of range"
            )

        self.grid = grid
        self.boundaries = tuple(boundaries)
        self.limits = limits
        self.initial_speed = initial_speed
        self.desired_speed = desired_speed
        self.weights = weights
        self.edge_sample_dt = edge_sample_dt
        self.start_distance_index = start_distance_index

    def _edge_cost(
        self,
        start: STGridNode,
        end: STGridNode,
    ) -> float:
        speed = edge_speed(
            start=start,
            end=end,
        )
        dt = end.t - start.t
        speed_error = speed - self.desired_speed
        return (
            self.weights.speed_error
            * speed_error
            * speed_error
            * dt
        )

    def _initial_acceleration_cost(
        self,
        start: STGridNode,
        end: STGridNode,
    ) -> float:
        acceleration = initial_edge_acceleration(
            initial_speed=self.initial_speed,
            start=start,
            end=end,
        )
        effective_dt = 0.5 * (end.t - start.t)
        return (
            self.weights.acceleration
            * acceleration
            * acceleration
            * effective_dt
        )

    def _acceleration_cost(
        self,
        previous: STGridNode,
        current: STGridNode,
        next_node: STGridNode,
    ) -> float:
        acceleration = transition_acceleration(
            previous=previous,
            current=current,
            next_node=next_node,
        )
        previous_mid_time = 0.5 * (
            previous.t + current.t
        )
        next_mid_time = 0.5 * (
            current.t + next_node.t
        )
        dt = next_mid_time - previous_mid_time
        return (
            self.weights.acceleration
            * acceleration
            * acceleration
            * dt
        )

    def plan(self) -> DPSpeedPlan:
        start = self.grid.node(
            time_index=0,
            distance_index=self.start_distance_index,
        )

        if is_node_blocked(
            node=start,
            boundaries=self.boundaries,
        ):
            raise RuntimeError(
                "DP start node is blocked"
            )

        records: dict[
            DPStateKey,
            DPRecord,
        ] = {}
        frontier: set[
            DPStateKey
        ] = set()

        for distance_index in range(
            self.grid.num_distance_steps
        ):
            current = self.grid.node(
                time_index=1,
                distance_index=distance_index,
            )

            if is_node_blocked(
                node=current,
                boundaries=self.boundaries,
            ):
                continue

            if not is_edge_speed_feasible(
                start=start,
                end=current,
                limits=self.limits,
            ):
                continue

            if not is_initial_edge_acceleration_feasible(
                initial_speed=self.initial_speed,
                start=start,
                end=current,
                limits=self.limits,
            ):
                continue

            if is_edge_blocked(
                start=start,
                end=current,
                boundaries=self.boundaries,
                sample_dt=self.edge_sample_dt,
            ):
                continue

            cost = (
                self._edge_cost(
                    start=start,
                    end=current,
                )
                + self._initial_acceleration_cost(
                    start=start,
                    end=current,
                )
            )

            key = DPStateKey(
                time_index=1,
                previous_distance_index=self.start_distance_index,
                distance_index=distance_index,
            )

            records[key] = DPRecord(
                cost=cost,
                parent=None,
            )
            frontier.add(key)

        if len(frontier) == 0:
            raise RuntimeError(
                "no feasible DP state at first time step"
            )

        for next_time_index in range(
            2,
            self.grid.num_time_steps,
        ):
            next_frontier: set[
                DPStateKey
            ] = set()

            for current_key in frontier:
                current_record = records[current_key]

                previous = self.grid.node(
                    time_index=current_key.time_index - 1,
                    distance_index=(
                        current_key.previous_distance_index
                    ),
                )
                current = self.grid.node(
                    time_index=current_key.time_index,
                    distance_index=current_key.distance_index,
                )

                for next_distance_index in range(
                    self.grid.num_distance_steps
                ):
                    next_node = self.grid.node(
                        time_index=next_time_index,
                        distance_index=next_distance_index,
                    )

                    if is_node_blocked(
                        node=next_node,
                        boundaries=self.boundaries,
                    ):
                        continue

                    if not is_edge_speed_feasible(
                        start=current,
                        end=next_node,
                        limits=self.limits,
                    ):
                        continue

                    if not is_transition_acceleration_feasible(
                        previous=previous,
                        current=current,
                        next_node=next_node,
                        limits=self.limits,
                    ):
                        continue

                    if is_edge_blocked(
                        start=current,
                        end=next_node,
                        boundaries=self.boundaries,
                        sample_dt=self.edge_sample_dt,
                    ):
                        continue

                    incremental_cost = (
                        self._edge_cost(
                            start=current,
                            end=next_node,
                        )
                        + self._acceleration_cost(
                            previous=previous,
                            current=current,
                            next_node=next_node,
                        )
                    )

                    total_cost = (
                        current_record.cost
                        + incremental_cost
                    )

                    next_key = DPStateKey(
                        time_index=next_time_index,
                        previous_distance_index=(
                            current_key.distance_index
                        ),
                        distance_index=next_distance_index,
                    )

                    existing = records.get(next_key)
                    if (
                        existing is None
                        or total_cost < existing.cost
                    ):
                        records[next_key] = DPRecord(
                            cost=total_cost,
                            parent=current_key,
                        )

                    next_frontier.add(next_key)

            if len(next_frontier) == 0:
                raise RuntimeError(
                    "DP became infeasible at "
                    f"time index {next_time_index}"
                )

            frontier = next_frontier

        final_key = min(
            frontier,
            key=lambda key: records[key].cost,
        )
        final_cost = records[final_key].cost

        distance_indices: dict[
            int,
            int,
        ] = {}

        key: DPStateKey | None = final_key
        while key is not None:
            distance_indices[
                key.time_index
            ] = key.distance_index
            distance_indices[
                key.time_index - 1
            ] = key.previous_distance_index
            key = records[key].parent

        nodes = tuple(
            self.grid.node(
                time_index=time_index,
                distance_index=distance_indices[time_index],
            )
            for time_index in range(
                self.grid.num_time_steps
            )
        )

        return DPSpeedPlan(
            nodes=nodes,
            total_cost=final_cost,
        )
