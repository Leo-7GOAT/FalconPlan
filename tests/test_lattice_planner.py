import pytest

from coordinate_system.reference_line import (
    ReferenceLine,
)

from planning.collision import (
    CircularObstacle,
    CollisionParams,
)

from planning.constraints import (
    TrajectoryConstraints,
)

from planning.cost import (
    TrajectoryCostWeights,
)

from planning.lattice_planner import (
    FrenetLatticePlanner,
    FrenetPlanningInput,
)

from planning.sampling import (
    LateralSamplingConfig,
)

from planning.world_constraints import (
    WorldTrajectoryConstraints,
)


def make_reference_line():

    return ReferenceLine(
        reference_path=[
            (0.0, 0.0),
            (25.0, 0.0),
            (50.0, 3.0),
            (75.0, 10.0),
            (100.0, 20.0),
            (130.0, 30.0),
        ]
    )


def make_planner():

    return FrenetLatticePlanner(
        reference_line=make_reference_line(),

        trajectory_constraints=(
            TrajectoryConstraints(
                max_speed=30.0,

                max_longitudinal_accel=3.0,
                max_longitudinal_jerk=5.0,

                max_lateral_accel=2.5,
                max_lateral_jerk=8.0,
            )
        ),

        world_constraints=(
            WorldTrajectoryConstraints(
                max_curvature=0.03,
            )
        ),

        collision_params=(
            CollisionParams(
                vehicle_radius=1.0,
                safety_margin=0.3,
            )
        ),

        cost_weights=(
            TrajectoryCostWeights()
        ),

        lateral_sampling=(
            LateralSamplingConfig(
                offsets=(
                    -0.5,
                    0.0,
                    0.5,
                )
            )
        ),

        dt=0.1,
    )


def make_input(
    obstacles=None,
):

    if obstacles is None:
        obstacles = []

    return FrenetPlanningInput(
        s0=0.0,
        s_d0=20.0,
        s_dd0=0.0,

        d0=0.0,
        d_d0=0.0,
        d_dd0=0.0,

        lane_centers=[
            0.0,
            3.5,
        ],

        target_speed_values=[
            23.0,
            25.0,
            27.0,
        ],

        duration_values=[
            2.5,
            3.0,
            3.5,
            4.0,
        ],

        desired_d=3.5,
        desired_speed=25.0,

        obstacles=obstacles,
    )


def test_no_obstacle_selects_lane_change():

    planner = make_planner()

    result = planner.plan(
        make_input()
    )

    assert (
        result.best.target_d
        == pytest.approx(3.5)
    )

    assert (
        result.best.target_speed
        == pytest.approx(25.0)
    )

    assert result.candidate_count == 72

    assert len(result.best.x) > 0
    assert len(result.best.y) > 0
    assert len(result.best.yaw) > 0
    assert len(result.best.curvature) > 0


def test_obstacle_causes_safe_fallback():

    planner = make_planner()

    # First find the original no-obstacle best.
    original = planner.plan(
        make_input()
    )

    best = original.best

    mid_index = len(best.x) // 2

    obstacle = CircularObstacle(
        x=best.x[mid_index],
        y=best.y[mid_index],
        radius=0.5,
    )

    result = planner.plan(
        make_input(
            obstacles=[
                obstacle,
            ]
        )
    )

    # Original desired left-lane trajectory is blocked.
    # Planner should fall back toward the current lane.
    assert result.best.target_d in (
        -0.5,
        0.0,
    )

    assert (
        result.collision_free_count
        < result.world_feasible_count
    )


def test_all_candidates_blocked_raises():

    planner = make_planner()

    # Huge obstacle near the beginning of the road.
    obstacle = CircularObstacle(
        x=10.0,
        y=0.0,
        radius=20.0,
    )

    with pytest.raises(
        RuntimeError,
        match="no collision-free trajectory",
    ):

        planner.plan(
            make_input(
                obstacles=[
                    obstacle,
                ]
            )
        )


def test_best_trajectory_respects_curvature_limit():

    planner = make_planner()

    result = planner.plan(
        make_input()
    )

    max_curvature = max(
        abs(kappa)
        for kappa in result.best.curvature
    )

    assert (
        max_curvature
        <= 0.03
    )