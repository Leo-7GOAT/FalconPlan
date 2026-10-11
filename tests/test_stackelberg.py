import numpy as np
import pytest
from behavior.stackelberg import (MergeAction, MergeState, GameConfig,
    continuous_clearance, payoff, solve_stackelberg, longitudinal)


def test_continuous_collision_catches_between_knot_crossing():
    # Both knots look far apart, but exchanging order on one lane collides.
    assert continuous_clearance(np.array([-10., 10.]), np.zeros(2), np.zeros(2)) == 0
    assert continuous_clearance(np.array([-10., 10.]), np.full(2, 3.5), np.zeros(2)) > 1


def test_stopping_trajectory_does_not_reverse():
    positions, speeds = longitudinal(0, 2, -2, np.array([0., 1., 2., 4.]))
    np.testing.assert_allclose(positions, [0, 1, 1, 1])
    np.testing.assert_allclose(speeds, [2, 0, 0, 0])


def test_best_responses_and_leader_optimum_against_independent_enumeration():
    state = MergeState(0, 11, -16, 13)
    result = solve_stackelberg(state, robust_gate=False)
    alternatives = []
    for merge in (False, True):
        for a in (-4., -2., 0., 2.):
            values = [payoff(state, MergeAction(a, merge), b) for b in (-4., -2., 0., 2.)]
            best_follower = max(v.follower for v in values)
            best_response = min((v for v in values if abs(v.follower-best_follower)<1e-10), key=lambda v: v.leader)
            if not best_response.collision: alternatives.append(best_response.leader)
    assert result.leader_payoff == max(alternatives)


def test_uncooperative_follower_is_gated_without_changing_collision_threshold():
    state = MergeState(0, 12, -9, 14)
    decision = solve_stackelberg(state)
    if decision.action.merge:
        assert all(not payoff(state, decision.action, a).collision for a in (-4, -2, 0, 2))
    assert not payoff(state, decision.action, decision.predicted_response).collision


@pytest.mark.parametrize('factory', [lambda: MergeState(0, -1, 0, 1), lambda: MergeState(float('nan'), 1, 0, 1), lambda: MergeAction(3, True), lambda: MergeAction(0, 1), lambda: GameConfig(dt=.2), lambda: GameConfig(merge_duration=10)])
def test_invalid_contracts(factory):
    with pytest.raises(ValueError): factory()


@pytest.mark.parametrize('x,y,z', [([0],[0],[0]),([0,1],[0,1,2],[0,1]),([0,float('nan')],[0,0],[1,2]),([[0,1]],[0,1],[0,1])])
def test_collision_contract_fails_closed(x,y,z):
    with pytest.raises(ValueError): continuous_clearance(x,y,z)
