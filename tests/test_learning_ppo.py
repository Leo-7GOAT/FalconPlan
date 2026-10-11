import numpy as np
import pytest
torch=pytest.importorskip('torch',reason='optional PPO experiment requires rl extra')
from falcon.learning.ppo import ActorCritic,seed_everything,generalized_advantage,ppo_loss,load_policy

def test_gae_terminal_mask_and_bootstrap_numerical():
    a,r=generalized_advantage([[1.],[2.]],[[.5],[1.]],[[0.],[1.]],np.array([99.]),gamma=1,lam=1)
    assert np.array_equal(a,np.array([[2.5],[1.]],dtype=np.float32))
    assert np.array_equal(r,np.array([[3.],[2.]],dtype=np.float32))
    with pytest.raises(ValueError):generalized_advantage([[float('nan')]],[[0]],[[0]],np.array([0]))

def test_mixed_policy_density_and_seed_determinism():
    seed_everything(42);model=ActorCritic();obs=torch.zeros(8,17)
    a,l,z,lp,v=model.sample(obs)
    actual,_,_=model.evaluate(obs,z,l)
    assert torch.allclose(actual,lp) and torch.isfinite(lp).all()
    assert ((a>=-6)&(a<=3)).all() and set(l.tolist())<={0,1}
    assert torch.equal(model.sample(obs,True)[0],model.sample(obs,True)[0])
    with pytest.raises(ValueError):model.sample(torch.full((1,17),float('nan')))

def test_actual_ppo_gradient_changes_weights_and_roundtrip(tmp_path):
    seed_everything(7);model=ActorCritic();obs=torch.randn(32,17)
    _,lane,z,old,_=model.sample(obs);before=model.mean.weight.detach().clone()
    optimizer=torch.optim.Adam(model.parameters(),lr=.001)
    loss,stats=ppo_loss(model,obs,z,lane,old,torch.linspace(-1,1,32),torch.ones(32))
    loss.backward();optimizer.step()
    assert not torch.equal(before,model.mean.weight) and np.isfinite(stats['value_loss'])
    path=tmp_path/'policy.pt';torch.save(dict(algorithm='PPO_CLIP_MIXED_ACTION_V1',state_dict=model.state_dict()),path)
    restored,_=load_policy(path)
    assert torch.equal(model.sample(obs,True)[0],restored.sample(obs,True)[0])

def test_training_pipeline_reproduces_tensor_weights_for_fixed_seed(tmp_path):
    from demos.learning.train_ppo import train
    a=train(tmp_path/'a',seed=19,total_steps=64,env_count=2,rollout_steps=32,epochs=1)
    b=train(tmp_path/'b',seed=19,total_steps=64,env_count=2,rollout_steps=32,epochs=1)
    assert a['trained_steps']==64 and a['parameter_delta_l2']>0
    x,_=load_policy(tmp_path/'a'/'policy.pt');y,_=load_policy(tmp_path/'b'/'policy.pt')
    assert all(torch.equal(x.state_dict()[k],y.state_dict()[k]) for k in x.state_dict())

@pytest.mark.parametrize('kwargs',[{'total_steps':513},{'total_steps':True},{'env_count':1.5},{'epochs':False},{'rollout_steps':0},{'seed':True},{'seed':-1},{'reward_version':True},{'actuator_envelope_version':True}])
def test_training_rejects_invalid_sizes_before_creating_artifacts(tmp_path,kwargs):
    from demos.learning.train_ppo import train
    target=tmp_path/'invalid'
    with pytest.raises(ValueError):train(target,**kwargs)
    assert not target.exists()
