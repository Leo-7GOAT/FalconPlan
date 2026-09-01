from dataclasses import dataclass

from coordinate_system.reference_line import (
    ReferenceLine,
)

from .collision import (
    CircularObstacle,
    CollisionParams,
    filter_collision_free_trajectories,
)

from .constraints import (
    TrajectoryConstraints,
    filter_feasible_trajectories,
)

from .cost import (
    TrajectoryCostBreakdown,
    TrajectoryCostWeights,
    select_best_trajectory,
)

from .sampling import (
    LateralSamplingConfig,
    sample_lateral_targets,
)

from .trajectory import (
    FrenetTrajectory,
    generate_candidate_trajectories,
)

from .world_constraints import (
    WorldTrajectoryConstraints,
    filter_world_feasible_trajectories,
)

from .world_projection import (
    project_trajectory_to_world,
)


@dataclass(frozen=True)
class FrenetPlanningInput:
    s0: float
    s_d0: float
    s_dd0: float

    d0: float
    d_d0: float
    d_dd0: float

    lane_centers: list[float]

    target_speed_values: list[float]
    duration_values: list[float]

    desired_d: float
    desired_speed: float

    obstacles: list[CircularObstacle]


@dataclass(frozen=True)
class FrenetPlanningResult:
    best: FrenetTrajectory
    best_cost: TrajectoryCostBreakdown

    candidate_count: int
    frenet_feasible_count: int
    world_feasible_count: int
    collision_free_count: int


class FrenetLatticePlanner:

    def __init__(
        self,
        reference_line: ReferenceLine,
        trajectory_constraints: TrajectoryConstraints,
        world_constraints: WorldTrajectoryConstraints,
        collision_params: CollisionParams,
        cost_weights: TrajectoryCostWeights,
        lateral_sampling: LateralSamplingConfig = LateralSamplingConfig(),
        dt: float = 0.1,
    ):

        if dt <= 0.0:
            raise ValueError(
                "dt must be positive"
            )

        self.reference_line = reference_line
        self.trajectory_constraints = trajectory_constraints
        self.world_constraints = world_constraints
        self.collision_params = collision_params
        self.cost_weights = cost_weights
        self.lateral_sampling = lateral_sampling
        self.dt = dt

    def plan(
        self,
        data: FrenetPlanningInput,
    ) -> FrenetPlanningResult:

        # ----------------------------------------------------
        # 1. Lane-aware lateral sampling
        # ----------------------------------------------------

        target_d_values = sample_lateral_targets(
            lane_centers=data.lane_centers,
            config=self.lateral_sampling,
        )

        # ----------------------------------------------------
        # 2. Generate lattice candidates
        # ----------------------------------------------------

        candidates = generate_candidate_trajectories(
            s0=data.s0,
            s_d0=data.s_d0,
            s_dd0=data.s_dd0,

            d0=data.d0,
            d_d0=data.d_d0,
            d_dd0=data.d_dd0,

            target_d_values=target_d_values,

            target_speed_values=(
                data.target_speed_values
            ),

            duration_values=(
                data.duration_values
            ),

            dt=self.dt,
        )

        if len(candidates) == 0:
            raise RuntimeError(
                "no candidate trajectories generated"
            )

        # ----------------------------------------------------
        # 3. Frenet dynamic constraints
        # ----------------------------------------------------

        frenet_feasible = (
            filter_feasible_trajectories(
                trajectories=candidates,
                constraints=(
                    self.trajectory_constraints
                ),
            )
        )

        if len(frenet_feasible) == 0:
            raise RuntimeError(
                "no Frenet-feasible trajectory"
            )

        # ----------------------------------------------------
        # 4. Frenet -> World
        # ----------------------------------------------------

        for trajectory in frenet_feasible:

            project_trajectory_to_world(
                trajectory=trajectory,
                reference_line=self.reference_line,
            )

        # ----------------------------------------------------
        # 5. World curvature constraints
        # ----------------------------------------------------

        world_feasible = (
            filter_world_feasible_trajectories(
                trajectories=frenet_feasible,
                constraints=self.world_constraints,
            )
        )

        if len(world_feasible) == 0:
            raise RuntimeError(
                "no World-feasible trajectory"
            )

        # ----------------------------------------------------
        # 6. Collision filtering
        # ----------------------------------------------------

        collision_free = (
            filter_collision_free_trajectories(
                trajectories=world_feasible,
                obstacles=data.obstacles,
                params=self.collision_params,
            )
        )

        if len(collision_free) == 0:
            raise RuntimeError(
                "no collision-free trajectory"
            )

        # ----------------------------------------------------
        # 7. Cost ranking
        # ----------------------------------------------------

        best, best_cost = (
            select_best_trajectory(
                trajectories=collision_free,

                desired_d=data.desired_d,
                desired_speed=data.desired_speed,

                weights=self.cost_weights,
            )
        )

        return FrenetPlanningResult(
            best=best,
            best_cost=best_cost,

            candidate_count=len(
                candidates
            ),

            frenet_feasible_count=len(
                frenet_feasible
            ),

            world_feasible_count=len(
                world_feasible
            ),

            collision_free_count=len(
                collision_free
            ),
        )