"""FalconPlan W07 longitudinal MPC: state (s,v,a), control jerk."""

from dataclasses import dataclass
import math
from collections.abc import Sequence

import casadi as ca


@dataclass(frozen=True)
class LongitudinalState:
    s: float
    v: float
    a: float


@dataclass(frozen=True)
class MPCReferencePoint:
    t: float
    s: float
    v: float
    a: float


@dataclass(frozen=True)
class LongitudinalMPCConfig:
    dt: float = 0.2
    horizon_steps: int = 8
    max_speed: float = 25.0
    max_accel: float = 3.0
    max_decel: float = 6.0
    max_jerk: float = 4.0
    weight_s: float = 0.8
    weight_v: float = 4.0
    weight_a: float = 0.2
    weight_j: float = 0.2
    terminal_weight: float = 3.0

    def __post_init__(self):
        if not math.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("dt must be positive and finite")
        if not isinstance(self.horizon_steps, int) or self.horizon_steps < 1:
            raise ValueError("horizon_steps must be >= 1")
        for name in ("max_speed", "max_accel", "max_decel", "max_jerk"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in ("weight_s", "weight_v", "weight_a", "weight_j", "terminal_weight"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        if self.weight_s + self.weight_v + self.weight_a + self.weight_j <= 0:
            raise ValueError("at least one objective weight must be positive")


@dataclass(frozen=True)
class MPCResult:
    jerk_command: float
    predicted_states: tuple[LongitudinalState, ...]
    predicted_jerks: tuple[float, ...]
    objective_value: float
    status: str


def propagate_constant_jerk(
    state: LongitudinalState, jerk: float, dt: float,
) -> LongitudinalState:
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be positive and finite")
    if not math.isfinite(jerk) or not all(
        math.isfinite(value) for value in (state.s, state.v, state.a)
    ):
        raise ValueError("state and jerk must be finite")
    return LongitudinalState(
        s=state.s + state.v * dt + 0.5 * state.a * dt**2 + jerk * dt**3 / 6.0,
        v=state.v + state.a * dt + 0.5 * jerk * dt**2,
        a=state.a + jerk * dt,
    )


class LongitudinalMPC:
    """Solve a finite-horizon constrained optimization at each control step."""

    def __init__(self, config: LongitudinalMPCConfig):
        self.config = config

    def solve(
        self,
        current: LongitudinalState,
        reference: Sequence[MPCReferencePoint],
        corridor: Sequence[tuple[float, float]] | None = None,
    ) -> MPCResult:
        cfg = self.config
        N, dt = cfg.horizon_steps, cfg.dt
        if len(reference) != N + 1:
            raise ValueError(f"reference needs exactly {N + 1} points")
        if corridor is not None and len(corridor) != N + 1:
            raise ValueError(f"corridor needs exactly {N + 1} intervals")
        if not all(math.isfinite(v) for v in (current.s, current.v, current.a)):
            raise ValueError("current state must be finite")
        if not (0 <= current.v <= cfg.max_speed):
            raise ValueError("initial speed violates configured limits")
        if not (-cfg.max_decel <= current.a <= cfg.max_accel):
            raise ValueError("initial acceleration violates configured limits")
        for k, p in enumerate(reference):
            if not all(math.isfinite(v) for v in (p.t, p.s, p.v, p.a)):
                raise ValueError("reference must be finite")
            if k and not math.isclose(p.t - reference[k-1].t, dt, rel_tol=0, abs_tol=1e-7):
                raise ValueError("reference times must be spaced by config.dt")
        if corridor is not None:
            for lower, upper in corridor:
                if not all(math.isfinite(v) for v in (lower, upper)) or lower > upper:
                    raise ValueError("invalid corridor interval")
            if not (corridor[0][0] - 1e-7 <= current.s <= corridor[0][1] + 1e-7):
                raise ValueError("current s is outside initial corridor")

        opti = ca.Opti()
        X = opti.variable(3, N + 1)  # rows s,v,a
        U = opti.variable(1, N)      # jerk commands
        opti.subject_to(X[:, 0] == ca.vertcat(current.s, current.v, current.a))

        objective = 0
        for k in range(N + 1):
            r = reference[k]
            terminal = cfg.terminal_weight if k == N else 1.0
            objective += terminal * (
                cfg.weight_s * (X[0,k] - r.s)**2
                + cfg.weight_v * (X[1,k] - r.v)**2
                + cfg.weight_a * (X[2,k] - r.a)**2
            )
            opti.subject_to(opti.bounded(0.0, X[1,k], cfg.max_speed))
            opti.subject_to(opti.bounded(-cfg.max_decel, X[2,k], cfg.max_accel))
            if corridor is not None:
                lower, upper = corridor[k]
                opti.subject_to(opti.bounded(lower, X[0,k], upper))
            if k < N:
                j = U[0,k]
                objective += cfg.weight_j * j**2
                opti.subject_to(opti.bounded(-cfg.max_jerk, j, cfg.max_jerk))
                opti.subject_to(X[0,k+1] == X[0,k] + X[1,k]*dt + 0.5*X[2,k]*dt**2 + j*dt**3/6)
                opti.subject_to(X[1,k+1] == X[1,k] + X[2,k]*dt + 0.5*j*dt**2)
                opti.subject_to(X[2,k+1] == X[2,k] + j*dt)
        opti.minimize(objective)
        for k, p in enumerate(reference):
            opti.set_initial(X[:,k], ca.vertcat(p.s, p.v, p.a))
        opti.set_initial(U, 0)
        opti.solver(
            "ipopt",
            {"expand": True, "print_time": False},
            {"print_level": 0, "sb": "yes", "max_iter": 500, "tol": 1e-8},
        )
        try:
            sol = opti.solve()
        except RuntimeError as exc:
            status = opti.stats().get("return_status", "unknown")
            raise RuntimeError(f"longitudinal MPC failed ({status})") from exc

        states = tuple(
            LongitudinalState(
                s=float(sol.value(X[0,k])),
                v=float(sol.value(X[1,k])),
                a=float(sol.value(X[2,k])),
            ) for k in range(N+1)
        )
        jerks = tuple(float(sol.value(U[0,k])) for k in range(N))
        return MPCResult(
            jerk_command=jerks[0],
            predicted_states=states,
            predicted_jerks=jerks,
            objective_value=float(sol.value(objective)),
            status=str(opti.stats().get("return_status", "success")),
        )
