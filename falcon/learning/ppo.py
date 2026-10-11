"""PPO-Clip with a tanh-Gaussian acceleration and categorical target lane.

Log probabilities include the tanh Jacobian. Entropy regularization uses the
base Normal entropy plus categorical entropy, a stated approximation. No rule
policy or random policy is substituted for learned weights.
"""
import math
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal,Categorical
from .environment import OBSERVATION_SIZE

def seed_everything(seed):
    np.random.seed(seed);torch.manual_seed(seed);torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)

class ActorCritic(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(OBSERVATION_SIZE,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh())
        self.mean=nn.Linear(64,1);self.lanes=nn.Linear(64,2);self.log_std=nn.Parameter(torch.tensor([-.5]))
        self.critic=nn.Sequential(nn.Linear(OBSERVATION_SIZE,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        for module in self.modules():
            if isinstance(module,nn.Linear):nn.init.orthogonal_(module.weight,math.sqrt(2));nn.init.zeros_(module.bias)
        nn.init.orthogonal_(self.mean.weight,.01);nn.init.constant_(self.mean.bias,.2)
        nn.init.orthogonal_(self.lanes.weight,.01);nn.init.orthogonal_(self.critic[-1].weight,1)
    def distributions(self,obs):
        if not torch.isfinite(obs).all():raise ValueError('non-finite observation')
        features=self.actor(obs)
        return Normal(self.mean(features).squeeze(-1),self.log_std.clamp(-4,1).exp()),Categorical(logits=self.lanes(features))
    def value(self,obs):return self.critic(obs).squeeze(-1)
    def evaluate(self,obs,latent,lane):
        gaussian,categorical=self.distributions(obs)
        logprob=gaussian.log_prob(latent)-torch.log(1-torch.tanh(latent).square()+1e-6)+categorical.log_prob(lane)
        return logprob,gaussian.entropy()+categorical.entropy(),self.value(obs)
    @torch.no_grad()
    def sample(self,obs,deterministic=False):
        gaussian,categorical=self.distributions(obs)
        latent=gaussian.mean if deterministic else gaussian.sample()
        lane=categorical.logits.argmax(-1) if deterministic else categorical.sample()
        logprob,_,value=self.evaluate(obs,latent,lane)
        bounded=torch.tanh(latent)
        accel=torch.where(bounded>=0,3*bounded,6*bounded)
        return accel,lane,latent,logprob,value

def generalized_advantage(rewards,values,dones,last_values,gamma=.995,lam=.95):
    rewards=np.asarray(rewards,dtype=np.float32);values=np.asarray(values,dtype=np.float32);dones=np.asarray(dones,dtype=np.float32)
    if rewards.shape!=values.shape or values.shape!=dones.shape or rewards.ndim!=2:raise ValueError('rollout arrays must share [time,env] shape')
    if not 0<=gamma<=1 or not 0<=lam<=1:raise ValueError('invalid discount')
    if not all(np.isfinite(a).all() for a in (rewards,values,dones,last_values)):raise ValueError('non-finite rollout')
    advantages=np.zeros_like(rewards);running=np.zeros(rewards.shape[1],dtype=np.float32)
    for t in reversed(range(len(rewards))):
        next_value=last_values if t==len(rewards)-1 else values[t+1]
        mask=1-dones[t]
        delta=rewards[t]+gamma*next_value*mask-values[t]
        running=delta+gamma*lam*mask*running;advantages[t]=running
    return advantages,advantages+values

def ppo_loss(model,observations,latent,lanes,old_logprob,advantages,returns,clip=.2,entropy_coef=.01):
    logprob,entropy,values=model.evaluate(observations,latent,lanes)
    ratio=(logprob-old_logprob).exp()
    policy=-torch.minimum(ratio*advantages,ratio.clamp(1-clip,1+clip)*advantages).mean()
    value=.5*(values-returns).square().mean();loss=policy+.5*value-entropy_coef*entropy.mean()
    if not torch.isfinite(loss):raise RuntimeError('non-finite PPO loss')
    return loss,dict(policy_loss=float(policy.detach()),value_loss=float(value.detach()),entropy=float(entropy.mean().detach()),approx_kl=float((old_logprob-logprob).mean().detach()),clip_fraction=float(((ratio-1).abs()>clip).float().mean().detach()))

def load_policy(path):
    checkpoint=torch.load(path,map_location='cpu',weights_only=False)
    if checkpoint.get('algorithm')!='PPO_CLIP_MIXED_ACTION_V1':raise ValueError('unsupported policy format')
    model=ActorCritic();model.load_state_dict(checkpoint['state_dict']);model.eval()
    return model,checkpoint
