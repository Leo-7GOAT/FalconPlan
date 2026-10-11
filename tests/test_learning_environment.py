import math
import numpy as np
import pytest
from falcon.learning.environment import DrivingEnv,EpisodeConfig,ENGINE
from falcon.learning.baseline import ClassicalBaseline

def test_seeded_dynamics_transition_and_observation_contract():
    a=DrivingEnv(EpisodeConfig(steps=30));b=DrivingEnv(EpisodeConfig(steps=30))
    assert np.array_equal(a.reset(7,True),b.reset(7,True))
    for _ in range(30):
        x=a.step(2,a.target_lane);y=b.step(2,b.target_lane)
        assert np.array_equal(x[0],y[0]) and x[1]==y[1]
        assert x[0].shape==(17,) and np.isfinite(x[0]).all()
    assert a.metrics()['engine']==ENGINE
    with pytest.raises(RuntimeError):a.step(0,0)

@pytest.mark.parametrize('fault',['state_timeout','command_timeout','estop'])
def test_policy_cannot_bypass_latched_supervisor(fault):
    env=DrivingEnv(EpisodeConfig(steps=30));env.reset(19)
    _,_,_,info=env.step(3,env.target_lane,fault=fault)
    assert info['safety']=='stopped' and info['command_acceleration']==-6
    _,_,_,info=env.step(3,env.target_lane)
    assert info['safety']=='stopped'

def test_collision_termination_is_failure_not_a_successful_short_run():
    env=DrivingEnv();env.reset(7);env.traffic[0].x=env.state.x+1;env.traffic[0].lane=env.current_lane
    _,_,done,_=env.step(0,env.current_lane)
    assert done and env.metrics()['collision_count']==1 and env.metrics()['status']=='FAIL'

def test_invalid_actions_and_domain_values():
    with pytest.raises(ValueError):EpisodeConfig(delay=-1)
    with pytest.raises(ValueError):EpisodeConfig(friction=float('nan'))
    env=DrivingEnv();env.reset(7)
    with pytest.raises(ValueError):env.step(float('nan'),0)
    with pytest.raises(ValueError):env.step(0,2)
    with pytest.raises(ValueError):env.step(0,True)
    with pytest.raises(ValueError):env.step(0,1.)
    with pytest.raises(ValueError):EpisodeConfig(steps=True)
    with pytest.raises(ValueError):EpisodeConfig(steps=400.)

def test_near_zero_braking_preserves_measured_jerk_bound():
    env=DrivingEnv(EpisodeConfig(steps=100));env.reset(7,scenario='free')
    for _ in range(100):env.step(-6,env.target_lane)
    q=env.metrics()
    assert q['max_jerk_m_s3']<=6+1e-6 and q['min_acceleration_m_s2']>=-6-1e-6

def test_classical_baseline_really_evaluates_mobil():
    env=DrivingEnv(EpisodeConfig(steps=50));env.reset(7);baseline=ClassicalBaseline(8)
    while not env.finished:env.step(*baseline.act(env))
    assert baseline.mobil_evaluations>0
    assert env.metrics()['collision_count']==0

def test_speed_ceiling_filter_avoids_overspeed_without_jerk_impulse():
    env=DrivingEnv(EpisodeConfig(steps=300));env.reset(7,scenario='free')
    while not env.finished:env.step(3,env.target_lane)
    assert max(r['v'] for r in env.rows)<=12+1e-9
    assert env.metrics()['max_jerk_m_s3']<=6+1e-6
    assert all(r['safety']=='active' for r in env.rows)

def test_reward_versions_retain_failure_reproduction():
    a=DrivingEnv(EpisodeConfig(reward_version=1,actuator_envelope_version=1))
    b=DrivingEnv(EpisodeConfig(reward_version=2,actuator_envelope_version=1))
    a.reset(7,scenario='free');b.reset(7,scenario='free')
    from vehicle_model.dynamic_bicycle import DynamicState
    a.state=DynamicState(y=a.state.y,v=10);b.state=DynamicState(y=b.state.y,v=10)
    _,r1,_,_=a.step(0,a.target_lane);_,r2,_,_=b.step(0,b.target_lane)
    assert r1-r2==pytest.approx(.125)
