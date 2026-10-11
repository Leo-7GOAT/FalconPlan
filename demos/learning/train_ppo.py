"""Actual deterministic CPU PPO training on explicitly analytical dynamics."""
import argparse
from dataclasses import asdict
import hashlib,json,time,platform
from pathlib import Path
import numpy as np
import torch
from numbers import Integral
from falcon.learning.environment import DrivingEnv,EpisodeConfig,ENGINE
from falcon.learning.ppo import ActorCritic,seed_everything,generalized_advantage,ppo_loss

def train(output,seed=7,total_steps=131072,env_count=8,rollout_steps=256,epochs=4,reward_version=2,actuator_envelope_version=None):
    sizes=(total_steps,env_count,rollout_steps,epochs)
    if any(isinstance(v,bool) or not isinstance(v,Integral) or v<=0 for v in sizes):raise ValueError('positive integer training sizes required')
    if isinstance(seed,bool) or not isinstance(seed,Integral) or seed<0:raise ValueError('nonnegative integer seed required')
    if total_steps%env_count:raise ValueError('total_steps must be divisible by env_count')
    if isinstance(reward_version,bool) or reward_version not in (1,2):raise ValueError('unsupported reward version')
    if actuator_envelope_version is not None and (isinstance(actuator_envelope_version,bool) or actuator_envelope_version not in (1,2,3)):raise ValueError('unsupported actuator envelope version')
    output=Path(output);output.mkdir(parents=True,exist_ok=True);seed_everything(seed)
    if actuator_envelope_version is None:actuator_envelope_version=3 if reward_version==2 else 1
    config=dict(seed=seed,total_steps=total_steps,env_count=env_count,rollout_steps=rollout_steps,epochs=epochs,gamma=.995,gae_lambda=.95,clip=.2,learning_rate=.0003,entropy_coef=.01,hidden=[64,64],device='cpu',domain_randomization=True,engine=ENGINE,torch_version=torch.__version__,numpy_version=np.__version__,platform=platform.platform(),deterministic_algorithms=True,reward_version=reward_version,actuator_envelope_version=actuator_envelope_version)
    (output/'training_config.json').write_text(json.dumps(config,indent=2))
    model=ActorCritic();initial=torch.cat([p.detach().flatten().clone() for p in model.parameters()]);optimizer=torch.optim.Adam(model.parameters(),lr=config['learning_rate'],eps=1e-5)
    envs=[DrivingEnv(EpisodeConfig(reward_version=reward_version,actuator_envelope_version=actuator_envelope_version)) for _ in range(env_count)];episode_ids=[0]*env_count
    observations=np.stack([e.reset(seed*100000+i,True) for i,e in enumerate(envs)])
    episodes=[];history=[];started=time.perf_counter();steps=0;update=0
    with (output/'episodes.jsonl').open('w') as episode_log,(output/'updates.jsonl').open('w') as update_log:
        while steps<total_steps:
            length=min(rollout_steps,max(1,(total_steps-steps)//env_count));buffers={key:[] for key in ['obs','latent','lane','logprob','value','reward','done']}
            for _ in range(length):
                obs_tensor=torch.from_numpy(observations)
                acceleration,lane,latent,logprob,value=model.sample(obs_tensor)
                buffers['obs'].append(observations.copy());buffers['latent'].append(latent.numpy());buffers['lane'].append(lane.numpy());buffers['logprob'].append(logprob.numpy());buffers['value'].append(value.numpy())
                rewards=[];dones=[];next_obs=[]
                for i,env in enumerate(envs):
                    ob,reward,done,_=env.step(float(acceleration[i]),int(lane[i]));rewards.append(reward);dones.append(done)
                    if done:
                        metrics=env.metrics();metrics.update(training_step=steps+i,episode_seed=seed*100000+i+episode_ids[i]*env_count)
                        episode_log.write(json.dumps(metrics)+'\n');episodes.append(metrics);episode_ids[i]+=1
                        ob=env.reset(seed*100000+i+episode_ids[i]*env_count,True)
                    next_obs.append(ob)
                observations=np.stack(next_obs);buffers['reward'].append(rewards);buffers['done'].append(dones);steps+=env_count
            with torch.no_grad():last_values=model.value(torch.from_numpy(observations)).numpy()
            values=np.asarray(buffers['value']);advantages,returns=generalized_advantage(buffers['reward'],values,buffers['done'],last_values)
            flat=lambda key:torch.as_tensor(np.asarray(buffers[key]).reshape((-1,)+np.asarray(buffers[key]).shape[2:]))
            obs=flat('obs').float();z=flat('latent').float();lanes=flat('lane').long();old=flat('logprob').float();adv=torch.from_numpy(advantages.reshape(-1));ret=torch.from_numpy(returns.reshape(-1));adv=(adv-adv.mean())/(adv.std(unbiased=False)+1e-8)
            stats=[]
            for _ in range(epochs):
                for batch in torch.randperm(len(obs)).split(256):
                    loss,info=ppo_loss(model,obs[batch],z[batch],lanes[batch],old[batch],adv[batch],ret[batch]);optimizer.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),.5);optimizer.step();stats.append(info)
            update+=1;recent=episodes[-40:];record=dict(update=update,steps=steps,completed_episodes=len(episodes),mean_recent_return=float(np.mean([e['return_sum'] for e in recent])) if recent else None,recent_collision_rate=float(np.mean([e['collision_count']>0 for e in recent])) if recent else None,wall_s=time.perf_counter()-started,**{k:float(np.mean([s[k] for s in stats])) for k in stats[0]})
            update_log.write(json.dumps(record)+'\n');update_log.flush();episode_log.flush();history.append(record)
            if update%4==0:print(json.dumps(record),flush=True)
            torch.save(dict(algorithm='PPO_CLIP_MIXED_ACTION_V1',state_dict=model.state_dict(),optimizer_state=optimizer.state_dict(),config=config,trained_steps=steps,completed_updates=update),output/'policy.pt')
    final=torch.cat([p.detach().flatten() for p in model.parameters()]);delta=float(torch.linalg.vector_norm(final-initial))
    assert delta>1e-6 and len(history)>0,'policy weights did not train'
    report=dict(status='PASS_TRAINING_PROCESS',engine=ENGINE,seed=seed,trained_steps=steps,updates=update,completed_episodes=len(episodes),wall_s=time.perf_counter()-started,parameter_delta_l2=delta,weights_sha256=hashlib.sha256((output/'policy.pt').read_bytes()).hexdigest(),performance_acceptance='PENDING_EVALUATION')
    (output/'training_summary.json').write_text(json.dumps(report,indent=2))
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True);axes[0].plot([h['steps'] for h in history],[h['mean_recent_return'] for h in history]);axes[0].set_ylabel('Mean last 40 episodic returns');axes[1].plot([h['steps'] for h in history],[h['recent_collision_rate'] for h in history]);axes[1].set_ylabel('Training collision fraction');axes[1].set_xlabel('Actual model transitions')
    for axis in axes:axis.grid()
    fig.tight_layout();fig.savefig(output/'reward_curve.png',dpi=150);plt.close(fig)
    print(json.dumps(report,indent=2),flush=True);return report

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int,default=7);parser.add_argument('--steps',type=int,default=131072);parser.add_argument('--reward-version',type=int,choices=[1,2],default=2);parser.add_argument('--actuator-envelope-version',type=int,choices=[1,2,3]);parser.add_argument('--output',default='artifacts/learning/ppo_seed7');args=parser.parse_args();train(args.output,args.seed,args.steps,reward_version=args.reward_version,actuator_envelope_version=args.actuator_envelope_version)
if __name__=='__main__':main()
