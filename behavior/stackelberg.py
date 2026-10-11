"""Finite-horizon two-vehicle merge game, with explicit model boundaries.

This is an analytical planning experiment, not CARLA execution. The leader
commits to acceleration and a smooth lateral merge; the follower observes it
and selects a best response. Ties are pessimistic for the leader. A separate
robust collision veto checks every follower action before allowing the merge.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class MergeState:
    ego_s: float
    ego_v: float
    other_s: float
    other_v: float

    def __post_init__(self):
        if any(not math.isfinite(x) for x in vars(self).values()):
            raise ValueError("merge state must be finite")
        if min(self.ego_v, self.other_v) < 0:
            raise ValueError("speed cannot be negative")


@dataclass(frozen=True)
class MergeAction:
    acceleration: float
    merge: bool

    def __post_init__(self):
        if not math.isfinite(self.acceleration) or not -4 <= self.acceleration <= 2:
            raise ValueError("acceleration must be finite in [-4,2] m/s²")
        if not isinstance(self.merge, bool):
            raise ValueError("merge must be bool")


@dataclass(frozen=True)
class GameConfig:
    horizon: float = 4.0
    dt: float = 0.05
    merge_start: float = 0.5
    merge_duration: float = 2.5
    lane_width: float = 3.5
    collision_distance: float = 5.0
    desired_speed: float = 14.0
    speed_weight: float = 1.0
    effort_weight: float = 0.3
    merge_reward: float = 15.0
    collision_cost: float = 10000.0

    def __post_init__(self):
        if any(not math.isfinite(x) or x <= 0 for x in vars(self).values()):
            raise ValueError("configuration must be positive finite")
        if self.dt > 0.1 or self.merge_start+self.merge_duration > self.horizon:
            raise ValueError("invalid sampling/merge horizon")


@dataclass(frozen=True)
class Payoff:
    leader: float
    follower: float
    clearance: float
    collision: bool


@dataclass(frozen=True)
class GameDecision:
    action: MergeAction
    predicted_response: float
    leader_payoff: float
    follower_payoff: float
    predicted_clearance: float
    safety_veto: bool
    matrix: tuple


def longitudinal(s, v, a, times):
    """Exact nonreversing constant-acceleration longitudinal trajectory."""
    moving = np.minimum(times, v/-a) if a < 0 else times
    return s+v*moving+0.5*a*moving*moving, np.maximum(0.0, v+a*times)


def trajectory(state, action, other_acceleration, config=GameConfig()):
    if not math.isfinite(other_acceleration) or not -4 <= other_acceleration <= 2:
        raise ValueError("opponent acceleration outside action space")
    times = np.linspace(0, config.horizon, math.ceil(config.horizon/config.dt)+1)
    ego_s, ego_v = longitudinal(state.ego_s, state.ego_v, action.acceleration, times)
    other_s, other_v = longitudinal(state.other_s, state.other_v, other_acceleration, times)
    phase = np.clip((times-config.merge_start)/config.merge_duration, 0, 1)
    lateral = config.lane_width*(1-(10*phase**3-15*phase**4+6*phase**5)) if action.merge else np.full_like(times, config.lane_width)
    return times, ego_s, ego_v, lateral, other_s, other_v


def continuous_clearance(ego_s, lateral, other_s, config=GameConfig()):
    arrays = [np.asarray(x, dtype=float) for x in (ego_s, lateral, other_s)]
    if any(x.ndim != 1 or len(x) < 2 or not np.isfinite(x).all() for x in arrays) or len({len(x) for x in arrays}) != 1:
        raise ValueError("collision trajectories must have matching finite samples")
    ego_s, lateral, other_s = arrays
    # Ellipse: 5 m longitudinal and 1.8 m lateral center separation. Two
    # adjacent-lane cars are safe even when longitudinal coordinates coincide.
    relative = np.column_stack(((ego_s-other_s)/config.collision_distance, lateral/1.8))
    start, delta = relative[:-1], np.diff(relative, axis=0)
    norm = np.sum(delta*delta, axis=1)
    fractions = np.divide(-np.sum(start*delta, axis=1), norm, out=np.zeros_like(norm), where=norm>0)
    nearest = start+np.clip(fractions, 0, 1)[:, None]*delta
    return float(np.sqrt(np.min(np.sum(nearest*nearest, axis=1))))


def payoff(state, action, other_acceleration, config=GameConfig(), follower_speed=None, follower_effort=0.3):
    target = config.desired_speed if follower_speed is None else follower_speed
    if not math.isfinite(target) or target <= 0 or not math.isfinite(follower_effort) or follower_effort < 0:
        raise ValueError("invalid opponent preference")
    t, s, v, lateral, other_s, other_v = trajectory(state, action, other_acceleration, config)
    clearance = continuous_clearance(s, lateral, other_s, config)
    collision = clearance < 1.0
    penalty = config.collision_cost if collision else 0.0
    leader = (s[-1]-s[0])/config.horizon - config.speed_weight*float(np.mean((v-config.desired_speed)**2)) - config.effort_weight*action.acceleration**2 + config.merge_reward*action.merge - penalty
    follower = (other_s[-1]-other_s[0])/config.horizon - float(np.mean((other_v-target)**2)) - follower_effort*other_acceleration**2 - penalty
    return Payoff(float(leader), float(follower), clearance, collision)


def solve_stackelberg(state, config=GameConfig(), follower_speed=None, follower_effort=0.3, robust_gate=True):
    """Enumerate the actual follower best response for each leader strategy.

    Among equal follower utilities choose the lowest leader utility, then the
    first acceleration. A robustness veto uses the full bounded opponent action
    set; it neither assumes cooperation nor changes physical collision limits.
    """
    if not isinstance(robust_gate, bool):
        raise ValueError("robust_gate must be bool")
    accelerations = (-4.0, -2.0, 0.0, 2.0)
    rows = []
    choices = []
    for merge in (False, True):
        for acceleration in accelerations:
            action = MergeAction(acceleration, merge)
            outcomes = [payoff(state, action, a, config, follower_speed, follower_effort) for a in accelerations]
            best = max(p.follower for p in outcomes)
            ties = [i for i, p in enumerate(outcomes) if abs(p.follower-best) <= 1e-10]
            response = min(ties, key=lambda i: (outcomes[i].leader, i))
            p = outcomes[response]
            veto = bool(robust_gate and merge and any(x.collision for x in outcomes))
            rows.append((merge, acceleration, tuple((a, q.leader, q.follower, q.clearance) for a, q in zip(accelerations, outcomes)), response, veto))
            if not p.collision and not veto:
                choices.append((action, accelerations[response], p))
    if not choices:
        raise RuntimeError("no collision-free action, including remaining in separate lane")
    action, response, result = max(choices, key=lambda x: x[2].leader)
    unconstrained_merge = max((row for row in rows if not payoff(state, MergeAction(row[1], row[0]), accelerations[row[3]], config, follower_speed, follower_effort).collision), key=lambda row: row[2][row[3]][1])[0]
    return GameDecision(action, response, result.leader, result.follower, result.clearance, bool(unconstrained_merge and not action.merge), tuple(rows))
