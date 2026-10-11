import pytest
from planning.longitudinal_mpc import (
    LongitudinalMPC, LongitudinalMPCConfig,
    LongitudinalState, MPCReferencePoint, propagate_constant_jerk,
)


def references(config, start_s=0.0, speed=10.0):
    return [
        MPCReferencePoint(
            t=k*config.dt, s=start_s + speed*k*config.dt,
            v=speed, a=0.0,
        ) for k in range(config.horizon_steps + 1)
    ]


def test_propagation_exact_constant_jerk():
    actual = propagate_constant_jerk(
        LongitudinalState(s=0.0, v=10.0, a=1.0), jerk=2.0, dt=0.5
    )
    assert actual.s == pytest.approx(5 + 0.125 + 2 * .5**3 / 6)
    assert actual.v == pytest.approx(10 + .5 + 2*.5**2/2)
    assert actual.a == pytest.approx(2.0)


def test_rejects_invalid_config():
    with pytest.raises(ValueError): LongitudinalMPCConfig(dt=0)
    with pytest.raises(ValueError): LongitudinalMPCConfig(horizon_steps=0)
    with pytest.raises(ValueError): LongitudinalMPCConfig(max_jerk=0)


def test_rejects_reference_length_and_spacing():
    cfg = LongitudinalMPCConfig(horizon_steps=3, dt=0.2)
    mpc = LongitudinalMPC(cfg)
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,10,0), references(cfg)[:-1])
    bad = references(cfg)
    bad[2] = MPCReferencePoint(t=0.45, s=4.0, v=10.0, a=0)
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,10,0), bad)


def test_rejects_illegal_initial_conditions():
    cfg = LongitudinalMPCConfig(horizon_steps=3)
    mpc = LongitudinalMPC(cfg)
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,30,0), references(cfg))
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,10,-10), references(cfg))


def test_equilibrium_tracking_has_zero_jerk():
    cfg = LongitudinalMPCConfig(horizon_steps=5, dt=0.2)
    result = LongitudinalMPC(cfg).solve(
        LongitudinalState(0, 10, 0), references(cfg)
    )
    assert result.status == 'Solve_Succeeded'
    assert result.jerk_command == pytest.approx(0.0, abs=1e-3)
    assert result.predicted_states[-1].v == pytest.approx(10, abs=1e-3)


def test_tracks_higher_reference_speed_with_acceleration():
    cfg = LongitudinalMPCConfig(horizon_steps=8, dt=.2)
    result = LongitudinalMPC(cfg).solve(
        LongitudinalState(0,5,0), references(cfg, speed=12)
    )
    assert result.jerk_command > 0
    assert result.predicted_states[-1].v > 5


def test_brakes_for_slower_reference():
    cfg = LongitudinalMPCConfig(horizon_steps=8, dt=.2)
    result = LongitudinalMPC(cfg).solve(
        LongitudinalState(0,12,0), references(cfg, speed=6)
    )
    assert result.jerk_command < 0
    assert result.predicted_states[-1].v < 12


def test_hard_constraints_hold_for_whole_prediction():
    cfg = LongitudinalMPCConfig(horizon_steps=7, dt=.2, max_jerk=2, max_accel=2, max_decel=3)
    result = LongitudinalMPC(cfg).solve(
        LongitudinalState(0,10,0), references(cfg, speed=3)
    )
    assert all(abs(j) <= 2.0001 for j in result.predicted_jerks)
    assert all(0 <= x.v <= cfg.max_speed + 1e-4 for x in result.predicted_states)
    assert all(-cfg.max_decel-1e-4 <= x.a <= cfg.max_accel+1e-4 for x in result.predicted_states)


def test_corridor_respected():
    cfg = LongitudinalMPCConfig(horizon_steps=5, dt=.2)
    ref = references(cfg)
    corridor = [(k*2-1, k*2+1) for k in range(6)]
    result = LongitudinalMPC(cfg).solve(LongitudinalState(0,10,0),ref,corridor)
    assert all(lo-1e-4 <= s.s <= hi+1e-4 for s,(lo,hi) in zip(result.predicted_states,corridor))


def test_receding_horizon_replans_after_disturbance():
    cfg = LongitudinalMPCConfig(horizon_steps=5, dt=.2)
    mpc = LongitudinalMPC(cfg)
    state = LongitudinalState(0,8,0)
    ref = references(cfg, speed=10)
    first = mpc.solve(state, ref)
    state = propagate_constant_jerk(state,first.jerk_command,.2)
    # Measurement perturbation: update the observed state before re-solving.
    disturbed = LongitudinalState(state.s, state.v-.4, state.a)
    new_ref = [MPCReferencePoint(t=p.t+.2, s=p.s+2, v=p.v,a=p.a) for p in ref]
    second = mpc.solve(disturbed,new_ref)
    assert second.predicted_states[0].v == pytest.approx(disturbed.v,abs=1e-5)
    assert second.jerk_command > 0


def test_executing_first_control_matches_predicted_next_state():
    cfg = LongitudinalMPCConfig(horizon_steps=5, dt=.2)
    current = LongitudinalState(0.0, 8.0, 0.0)
    solution = LongitudinalMPC(cfg).solve(current, references(cfg, speed=10.0))
    actual = propagate_constant_jerk(current, solution.jerk_command, cfg.dt)
    expected = solution.predicted_states[1]
    assert (actual.s, actual.v, actual.a) == pytest.approx((expected.s, expected.v, expected.a), abs=1e-4)


def test_rejects_invalid_corridor():
    cfg = LongitudinalMPCConfig(horizon_steps=3, dt=.2)
    mpc = LongitudinalMPC(cfg)
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,10,0), references(cfg), [(0,1)]*3)
    with pytest.raises(ValueError):
        mpc.solve(LongitudinalState(0,10,0), references(cfg), [(1,3)]*4)
