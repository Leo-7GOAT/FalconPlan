
import numpy as np
import pytest

from planning.speed_corridor import (
    SpeedCorridor,
    SpeedCorridorPoint,
)
from planning.speed_profile import (
    SpeedProfile,
    SpeedProfilePoint,
)
from planning.speed_qp import (
    SpeedQPConstraints,
    SpeedQPIndex,
    SpeedQPWeights,
    build_speed_qp_problem,
    solve_speed_qp,
)


def make_profile():
    return SpeedProfile(
        points=(
            SpeedProfilePoint(t=0.0, s=0.0, v=10.0, a=0.0),
            SpeedProfilePoint(t=1.0, s=10.0, v=10.0, a=0.0),
            SpeedProfilePoint(t=2.0, s=20.0, v=10.0, a=0.0),
        )
    )


def make_corridor():
    return SpeedCorridor(
        points=(
            SpeedCorridorPoint(
                t=0.0, reference_s=0.0,
                lower_s=0.0, upper_s=30.0,
            ),
            SpeedCorridorPoint(
                t=1.0, reference_s=10.0,
                lower_s=0.0, upper_s=30.0,
            ),
            SpeedCorridorPoint(
                t=2.0, reference_s=20.0,
                lower_s=0.0, upper_s=30.0,
            ),
        )
    )


def make_problem():
    return build_speed_qp_problem(
        reference_profile=make_profile(),
        corridor=make_corridor(),
        constraints=SpeedQPConstraints(
            min_speed=0.0,
            max_speed=25.0,
            max_accel=3.0,
            max_decel=6.0,
            max_jerk=4.0,
        ),
        weights=SpeedQPWeights(
            position_tracking=1.0,
            speed_tracking=1.0,
            acceleration=0.2,
            jerk=1.0,
        ),
    )


# ============================================================
# 1. Variable layout
# ============================================================

def test_qp_variable_index_layout():
    index = SpeedQPIndex(num_points=3)

    assert index.dimension == 9

    assert index.s(0) == 0
    assert index.s(2) == 2

    assert index.v(0) == 3
    assert index.v(2) == 5

    assert index.a(0) == 6
    assert index.a(2) == 8


# ============================================================
# 2. Matrix dimensions
# ============================================================

def test_qp_problem_has_expected_shape():
    problem = make_problem()

    assert problem.P.shape == (9, 9)
    assert problem.q.shape == (9,)

    # 3 initial + 4 dynamics + 9 bounds + 2 jerk = 18
    assert problem.A.shape == (18, 9)
    assert problem.l.shape == (18,)
    assert problem.u.shape == (18,)


# ============================================================
# 3. Initial state equality
# ============================================================

def test_qp_initial_state_is_fixed():
    problem = make_problem()
    A = problem.A.toarray()
    index = problem.index

    assert A[0, index.s(0)] == pytest.approx(1.0)
    assert problem.l[0] == pytest.approx(0.0)
    assert problem.u[0] == pytest.approx(0.0)

    assert A[1, index.v(0)] == pytest.approx(1.0)
    assert problem.l[1] == pytest.approx(10.0)
    assert problem.u[1] == pytest.approx(10.0)

    assert A[2, index.a(0)] == pytest.approx(1.0)
    assert problem.l[2] == pytest.approx(0.0)
    assert problem.u[2] == pytest.approx(0.0)


# ============================================================
# 4. Position dynamics, constant jerk
# ============================================================

def test_qp_position_dynamics_row_is_correct():
    problem = make_problem()
    A = problem.A.toarray()
    index = problem.index

    # s1 - s0 - v0*dt
    # - dt^2/3*a0 - dt^2/6*a1 = 0
    # dt = 1

    row = A[3]

    assert row[index.s(0)] == pytest.approx(-1.0)
    assert row[index.s(1)] == pytest.approx(1.0)
    assert row[index.v(0)] == pytest.approx(-1.0)

    assert row[index.a(0)] == pytest.approx(-1.0 / 3.0)
    assert row[index.a(1)] == pytest.approx(-1.0 / 6.0)

    assert problem.l[3] == pytest.approx(0.0)
    assert problem.u[3] == pytest.approx(0.0)


# ============================================================
# 5. Velocity dynamics, constant jerk
# ============================================================

def test_qp_velocity_dynamics_row_is_correct():
    problem = make_problem()
    A = problem.A.toarray()
    index = problem.index

    # v1 - v0 - 0.5*dt*(a0+a1) = 0

    row = A[4]

    assert row[index.v(0)] == pytest.approx(-1.0)
    assert row[index.v(1)] == pytest.approx(1.0)

    assert row[index.a(0)] == pytest.approx(-0.5)
    assert row[index.a(1)] == pytest.approx(-0.5)


# ============================================================
# 6. Corridor bounds
# ============================================================

def test_qp_corridor_bounds_are_encoded():
    problem = make_problem()
    A = problem.A.toarray()
    index = problem.index

    # Rows 0..2: initial state
    # Rows 3..6: dynamics
    # Row 7: first corridor bound

    row = 7

    assert A[row, index.s(0)] == pytest.approx(1.0)
    assert problem.l[row] == pytest.approx(0.0)
    assert problem.u[row] == pytest.approx(30.0)


# ============================================================
# 7. Objective symmetry
# ============================================================

def test_qp_objective_matrix_is_symmetric():
    problem = make_problem()
    P = problem.P.toarray()

    assert np.allclose(P, P.T)


# ============================================================
# 8. Input contract
# ============================================================

def test_qp_rejects_mismatched_corridor_time():
    profile = make_profile()

    corridor = SpeedCorridor(
        points=(
            SpeedCorridorPoint(
                t=0.0, reference_s=0.0,
                lower_s=0.0, upper_s=30.0,
            ),
            SpeedCorridorPoint(
                t=1.5, reference_s=10.0,
                lower_s=0.0, upper_s=30.0,
            ),
            SpeedCorridorPoint(
                t=2.0, reference_s=20.0,
                lower_s=0.0, upper_s=30.0,
            ),
        )
    )

    with pytest.raises(ValueError):
        build_speed_qp_problem(
            reference_profile=profile,
            corridor=corridor,
            constraints=SpeedQPConstraints(),
        )


# ============================================================
# 9. Constant-speed solution
# ============================================================

def test_qp_solver_solves_constant_speed_profile():
    profile = make_profile()

    problem = build_speed_qp_problem(
        reference_profile=profile,
        corridor=make_corridor(),
        constraints=SpeedQPConstraints(),
    )

    solution = solve_speed_qp(
        problem=problem,
        reference_profile=profile,
    )

    assert solution.status.lower() == "solved"

    assert solution.profile.distances == pytest.approx(
        (0.0, 10.0, 20.0), abs=1e-4
    )

    assert solution.profile.speeds == pytest.approx(
        (10.0, 10.0, 10.0), abs=1e-4
    )

    assert solution.profile.accelerations == pytest.approx(
        (0.0, 0.0, 0.0), abs=1e-4
    )


# ============================================================
# 10. Solver corridor constraints
# ============================================================

def test_qp_solver_respects_corridor():
    profile = make_profile()

    corridor = SpeedCorridor(
        points=(
            SpeedCorridorPoint(
                t=0.0, reference_s=0.0,
                lower_s=0.0, upper_s=0.0,
            ),
            SpeedCorridorPoint(
                t=1.0, reference_s=10.0,
                lower_s=9.0, upper_s=11.0,
            ),
            SpeedCorridorPoint(
                t=2.0, reference_s=20.0,
                lower_s=19.0, upper_s=21.0,
            ),
        )
    )

    problem = build_speed_qp_problem(
        reference_profile=profile,
        corridor=corridor,
        constraints=SpeedQPConstraints(),
    )

    solution = solve_speed_qp(
        problem=problem,
        reference_profile=profile,
    )

    for point, bound in zip(
        solution.profile.points, corridor.points
    ):
        assert point.s >= bound.lower_s - 1e-4
        assert point.s <= bound.upper_s + 1e-4


# ============================================================
# 11. Acceleration hard limits
# ============================================================

def test_qp_solver_respects_acceleration_bounds():
    profile = make_profile()

    problem = build_speed_qp_problem(
        reference_profile=profile,
        corridor=make_corridor(),
        constraints=SpeedQPConstraints(
            max_accel=3.0,
            max_decel=6.0,
        ),
    )

    solution = solve_speed_qp(
        problem=problem,
        reference_profile=profile,
    )

    assert all(
        -6.0 - 1e-4 <= a <= 3.0 + 1e-4
        for a in solution.profile.accelerations
    )


# ============================================================
# 12. Initial state preservation
# ============================================================

def test_qp_solver_preserves_initial_state():
    profile = make_profile()

    solution = solve_speed_qp(
        problem=make_problem(),
        reference_profile=profile,
    )

    first = solution.profile.points[0]

    assert first.s == pytest.approx(0.0, abs=1e-4)
    assert first.v == pytest.approx(10.0, abs=1e-4)
    assert first.a == pytest.approx(0.0, abs=1e-4)


# ============================================================
# 13. Hard jerk matrix
# ============================================================

def test_qp_hard_jerk_matrix_rows():
    problem = make_problem()

    A = problem.A.toarray()
    index = problem.index

    # N = 3, jerk rows are 16 and 17

    assert A[16, index.a(0)] == pytest.approx(-1.0)
    assert A[16, index.a(1)] == pytest.approx(1.0)

    assert problem.l[16] == pytest.approx(-4.0)
    assert problem.u[16] == pytest.approx(4.0)

    assert A[17, index.a(1)] == pytest.approx(-1.0)
    assert A[17, index.a(2)] == pytest.approx(1.0)


# ============================================================
# 14. Hard jerk solver acceptance
# ============================================================

def test_qp_solver_respects_hard_jerk():
    profile = make_profile()

    problem = build_speed_qp_problem(
        reference_profile=profile,
        corridor=make_corridor(),
        constraints=SpeedQPConstraints(
            max_jerk=0.5,
        ),
    )

    solution = solve_speed_qp(
        problem=problem,
        reference_profile=profile,
    )

    points = solution.profile.points

    jerks = [
        (points[i + 1].a - points[i].a)
        / (points[i + 1].t - points[i].t)
        for i in range(len(points) - 1)
    ]

    assert all(
        abs(jerk) <= 0.5 + 1e-4
        for jerk in jerks
    )


# ============================================================
# 15. Invalid jerk constraint
# ============================================================

def test_qp_rejects_invalid_max_jerk():
    with pytest.raises(ValueError):
        SpeedQPConstraints(max_jerk=0.0)

    with pytest.raises(ValueError):
        SpeedQPConstraints(max_jerk=-1.0)
