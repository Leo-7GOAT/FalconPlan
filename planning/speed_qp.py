
from dataclasses import dataclass

import numpy as np
import osqp
from scipy import sparse

from .speed_corridor import SpeedCorridor
from .speed_profile import SpeedProfile, SpeedProfilePoint


@dataclass(frozen=True)
class SpeedQPWeights:
    position_tracking: float = 1.0
    speed_tracking: float = 1.0
    acceleration: float = 0.2
    jerk: float = 1.0

    def __post_init__(self):
        values = (
            self.position_tracking,
            self.speed_tracking,
            self.acceleration,
            self.jerk,
        )
        if any(not np.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("QP weights must be non-negative")


@dataclass(frozen=True)
class SpeedQPConstraints:
    min_speed: float = 0.0
    max_speed: float = 25.0

    max_accel: float = 3.0
    max_decel: float = 6.0
    max_jerk: float = 4.0

    def __post_init__(self):
        if any(not np.isfinite(value) for value in vars(self).values()):
            raise ValueError("QP limits must be finite")
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
        if self.max_jerk <= 0.0:
            raise ValueError("max_jerk must be positive")


@dataclass(frozen=True)
class SpeedQPIndex:
    num_points: int

    def __post_init__(self):
        if self.num_points < 2:
            raise ValueError("QP needs at least two points")

    @property
    def dimension(self) -> int:
        return 3 * self.num_points

    def _check(self, k: int):
        if not 0 <= k < self.num_points:
            raise IndexError("QP index out of range")

    def s(self, k: int) -> int:
        self._check(k)
        return k

    def v(self, k: int) -> int:
        self._check(k)
        return self.num_points + k

    def a(self, k: int) -> int:
        self._check(k)
        return 2 * self.num_points + k


@dataclass(frozen=True)
class SpeedQPProblem:
    P: sparse.csc_matrix
    q: np.ndarray

    A: sparse.csc_matrix
    l: np.ndarray
    u: np.ndarray

    index: SpeedQPIndex


@dataclass(frozen=True)
class SpeedQPSolution:
    profile: SpeedProfile
    objective_value: float
    status: str


def _validate_inputs(
    reference_profile: SpeedProfile,
    corridor: SpeedCorridor,
):
    if len(reference_profile.points) != len(corridor.points):
        raise ValueError("profile and corridor lengths must match")

    if len(reference_profile.points) < 2:
        raise ValueError("QP needs at least two samples")

    for p, c in zip(reference_profile.points, corridor.points):
        if not np.isclose(p.t, c.t, atol=1e-9, rtol=0.0):
            raise ValueError("profile and corridor times must match")

        if not np.isclose(
            p.s, c.reference_s, atol=1e-9, rtol=0.0
        ):
            raise ValueError("corridor reference must match profile")


def build_speed_qp_problem(
    *,
    reference_profile: SpeedProfile,
    corridor: SpeedCorridor,
    constraints: SpeedQPConstraints,
    weights: SpeedQPWeights = SpeedQPWeights(),
) -> SpeedQPProblem:

    _validate_inputs(reference_profile, corridor)

    n = len(reference_profile.points)
    index = SpeedQPIndex(n)
    dimension = index.dimension

    # ======================================================
    # OBJECTIVE
    #
    # min 0.5*x.T*P*x + q.T*x
    #
    # x = [s0...sN, v0...vN, a0...aN]
    # ======================================================

    P = sparse.lil_matrix((dimension, dimension), dtype=float)
    q = np.zeros(dimension, dtype=float)

    for k, point in enumerate(reference_profile.points):

        si = index.s(k)
        vi = index.v(k)
        ai = index.a(k)

        # Position tracking:
        # w_s * (s - s_ref)^2
        P[si, si] += 2.0 * weights.position_tracking
        q[si] -= 2.0 * weights.position_tracking * point.s

        # Speed tracking:
        # w_v * (v - v_ref)^2
        P[vi, vi] += 2.0 * weights.speed_tracking
        q[vi] -= 2.0 * weights.speed_tracking * point.v

        # Acceleration magnitude:
        # w_a * a^2
        P[ai, ai] += 2.0 * weights.acceleration

    # Jerk smoothness:
    #
    # j[k] = (a[k+1] - a[k]) / dt
    #
    # cost = w_j * j[k]^2

    for k in range(n - 1):

        dt = (
            reference_profile.points[k + 1].t
            - reference_profile.points[k].t
        )

        if dt <= 0.0:
            raise ValueError("time must increase")

        coefficient = weights.jerk / (dt * dt)

        a0 = index.a(k)
        a1 = index.a(k + 1)

        P[a0, a0] += 2.0 * coefficient
        P[a1, a1] += 2.0 * coefficient

        P[a0, a1] -= 2.0 * coefficient
        P[a1, a0] -= 2.0 * coefficient

    # ======================================================
    # CONSTRAINT MATRIX
    #
    # l <= A*x <= u
    # ======================================================

    num_initial_rows = 3
    num_dynamic_rows = 2 * (n - 1)
    num_bound_rows = 3 * n
    num_jerk_rows = n - 1

    num_rows = (
        num_initial_rows
        + num_dynamic_rows
        + num_bound_rows
        + num_jerk_rows
    )

    A = sparse.lil_matrix((num_rows, dimension), dtype=float)

    l = np.zeros(num_rows, dtype=float)
    u = np.zeros(num_rows, dtype=float)

    row = 0

    # ======================================================
    # A. INITIAL STATE
    #
    # s0, v0, a0 are fixed.
    # ======================================================

    initial = reference_profile.points[0]

    for variable_index, value in (
        (index.s(0), initial.s),
        (index.v(0), initial.v),
        (index.a(0), initial.a),
    ):
        A[row, variable_index] = 1.0
        l[row] = value
        u[row] = value
        row += 1

    # ======================================================
    # B. PIECEWISE CONSTANT JERK DYNAMICS
    #
    # Acceleration changes linearly within each interval.
    #
    # s[k+1] = s[k] + v[k]*dt
    #          + (2*a[k]+a[k+1])*dt^2/6
    #
    # v[k+1] = v[k] + (a[k]+a[k+1])*dt/2
    # ======================================================

    for k in range(n - 1):

        t0 = reference_profile.points[k].t
        t1 = reference_profile.points[k + 1].t

        dt = t1 - t0

        if dt <= 0.0:
            raise ValueError("time must increase")

        # Position dynamics
        A[row, index.s(k)] = -1.0
        A[row, index.s(k + 1)] = 1.0

        A[row, index.v(k)] = -dt

        A[row, index.a(k)] = -(dt * dt) / 3.0
        A[row, index.a(k + 1)] = -(dt * dt) / 6.0

        l[row] = 0.0
        u[row] = 0.0
        row += 1

        # Velocity dynamics
        A[row, index.v(k)] = -1.0
        A[row, index.v(k + 1)] = 1.0

        A[row, index.a(k)] = -0.5 * dt
        A[row, index.a(k + 1)] = -0.5 * dt

        l[row] = 0.0
        u[row] = 0.0
        row += 1

    # ======================================================
    # C. ST SAFE CORRIDOR
    # ======================================================

    for k, point in enumerate(corridor.points):

        A[row, index.s(k)] = 1.0

        l[row] = point.lower_s
        u[row] = point.upper_s

        row += 1

    # ======================================================
    # D. SPEED BOUNDS
    # ======================================================

    for k in range(n):

        A[row, index.v(k)] = 1.0

        l[row] = constraints.min_speed
        u[row] = constraints.max_speed

        row += 1

    # ======================================================
    # E. ACCELERATION BOUNDS
    # ======================================================

    for k in range(n):

        A[row, index.a(k)] = 1.0

        l[row] = -constraints.max_decel
        u[row] = constraints.max_accel

        row += 1

    # ======================================================
    # F. HARD JERK CONSTRAINTS
    #
    # -j_max <= (a[k+1]-a[k])/dt <= j_max
    # ======================================================

    for k in range(n - 1):

        dt = (
            reference_profile.points[k + 1].t
            - reference_profile.points[k].t
        )

        A[row, index.a(k)] = -1.0 / dt
        A[row, index.a(k + 1)] = 1.0 / dt

        l[row] = -constraints.max_jerk
        u[row] = constraints.max_jerk

        row += 1

    if row != num_rows:
        raise RuntimeError("QP constraint row count mismatch")

    return SpeedQPProblem(
        P=P.tocsc(),
        q=q,
        A=A.tocsc(),
        l=l,
        u=u,
        index=index,
    )


def solve_speed_qp(
    *,
    problem: SpeedQPProblem,
    reference_profile: SpeedProfile,
    eps_abs: float = 1e-6,
    eps_rel: float = 1e-6,
    max_iter: int = 10000,
) -> SpeedQPSolution:

    if eps_abs <= 0.0 or eps_rel <= 0.0:
        raise ValueError("solver tolerances must be positive")

    if max_iter <= 0:
        raise ValueError("max_iter must be positive")

    if len(reference_profile.points) != problem.index.num_points:
        raise ValueError("reference profile length mismatch")

    solver = osqp.OSQP()

    solver.setup(
        P=problem.P,
        q=problem.q,
        A=problem.A,
        l=problem.l,
        u=problem.u,
        verbose=False,
        eps_abs=eps_abs,
        eps_rel=eps_rel,
        max_iter=max_iter,
    )

    # Explicit argument avoids OSQP's default-value warning
    # in versions supporting raise_error.
    try:
        result = solver.solve(raise_error=False)
    except TypeError:
        result = solver.solve()

    status = str(result.info.status)

    if result.x is None or status.lower() != "solved":
        raise RuntimeError(
            f"OSQP failed to solve speed QP: {status}"
        )

    x = np.asarray(result.x, dtype=float)

    if not np.all(np.isfinite(x)):
        raise RuntimeError("OSQP returned non-finite values")

    # Verify the numerical result against the original
    # constraint matrix, not merely the solver status.
    Ax = np.asarray(problem.A @ x).reshape(-1)

    lower_violation = np.max(
        np.maximum(problem.l - Ax, 0.0)
    )
    upper_violation = np.max(
        np.maximum(Ax - problem.u, 0.0)
    )

    if max(lower_violation, upper_violation) > 1e-4:
        raise RuntimeError(
            "OSQP solution violates QP constraints"
        )

    points = []

    for k, reference_point in enumerate(reference_profile.points):

        s = float(x[problem.index.s(k)])
        v = float(x[problem.index.v(k)])
        a = float(x[problem.index.a(k)])

        if abs(s) < 1e-8:
            s = 0.0
        if abs(v) < 1e-8:
            v = 0.0
        if abs(a) < 1e-8:
            a = 0.0

        points.append(
            SpeedProfilePoint(
                t=reference_point.t,
                s=s,
                v=v,
                a=a,
            )
        )

    return SpeedQPSolution(
        profile=SpeedProfile(points=tuple(points)),
        objective_value=float(result.info.obj_val),
        status=status,
    )
