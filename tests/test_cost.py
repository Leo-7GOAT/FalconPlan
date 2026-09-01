import pytest

from planning.constraints import (
    TrajectoryConstraints,
    filter_feasible_trajectories,
)

from planning.cost import (
    TrajectoryCostWeights,
    select_best_trajectory,
)

from planning.trajectory import (
    generate_candidate_trajectories,
)


def test_select_best_reference_trajectory():

    trajectories = generate_candidate_trajectories(
        s0=0.0,
        s_d0=20.0,
        s_dd0=0.0,

        d0=0.0,
        d_d0=0.0,
        d_dd0=0.0,

        target_d_values=[
            3.2,
            3.5,
            3.8,
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

        dt=0.1,
    )

    constraints = TrajectoryConstraints(
        max_speed=30.0,

        max_longitudinal_accel=3.0,
        max_longitudinal_jerk=5.0,

        max_lateral_accel=2.5,
        max_lateral_jerk=8.0,
    )

    feasible = filter_feasible_trajectories(
        trajectories,
        constraints,
    )

    best, cost = select_best_trajectory(
        trajectories=feasible,

        desired_d=3.5,
        desired_speed=25.0,

        weights=TrajectoryCostWeights(),
    )

    assert best.target_d == pytest.approx(
        3.5
    )

    assert best.target_speed == pytest.approx(
        25.0
    )

    assert best.duration == pytest.approx(
        3.5
    )

    assert cost.total == pytest.approx(
        2.1241288943,
    )


def test_select_best_empty_input_rejected():

    with pytest.raises(ValueError):

        select_best_trajectory(
            trajectories=[],

            desired_d=3.5,
            desired_speed=25.0,

            weights=TrajectoryCostWeights(),
        )