"""Paired ID/OOD evaluation of learned weights and actual FSM/IDM/MOBIL.

All metrics are measured on the analytical model, never reported as CARLA.
An unshielded ablation keeps angle/rate/jerk/SafetySupervisor limits but removes
predictive collision/lane-change gating; it exposes raw learned behavior.
"""
import argparse,gzip,json,time,platform
from pathlib import Path
import numpy as np
import torch
from dataclasses import replace
from falcon.learning.environment import DrivingEnv,EpisodeConfig,ENGINE
from falcon.learning.baseline import ClassicalBaseline
from falcon.learning.ppo import load_policy,seed_everything

CASES={
    'ID':EpisodeConfig(),
    'OOD_delay':EpisodeConfig(delay=.25),
    'OOD_friction':EpisodeConfig(friction=.4),
    'OOD_gain_mass':EpisodeConfig(acceleration_gain=1.4,mass_scale=1.3),
    'OOD_combined':EpisodeConfig(desired_speed=12,friction=.4,delay=.25,mass_scale=1.3,acceleration_gain=1.3),
}

def evaluate(weights,output,seeds=(101,211,307,401,503),cases=tuple(CASES),scenarios=('free','mixed','braking')):
    if not seeds or not cases or not scenarios or any(c not in CASES for c in cases) or any(s not in ('free','mixed','braking') for s in scenarios):raise ValueError('invalid evaluation selection')
    seed_everything(0);model,checkpoint=load_policy(weights);output=Path(output);output.mkdir(parents=True,exist_ok=True);results=[];started=time.perf_counter()
    for case in cases:
        config=CASES[case]
        config=replace(config,reward_version=checkpoint['config'].get('reward_version',1),actuator_envelope_version=checkpoint['config'].get('actuator_envelope_version',1))
        for scenario in scenarios:
            for seed in seeds:
                for policy in ('FSM_IDM_MOBIL','PPO_guarded','PPO_without_predictive_gate'):
                    env=DrivingEnv(config,shield=policy!='PPO_without_predictive_gate');env.reset(seed,scenario=scenario);baseline=ClassicalBaseline(config.desired_speed);inference=[]
                    while not env.finished:
                        tick=time.perf_counter_ns()
                        if policy=='FSM_IDM_MOBIL':action=baseline.act(env)
                        else:
                            acceleration,lane,*_=model.sample(torch.from_numpy(env.observation()[None,:]),True);action=float(acceleration[0]),int(lane[0])
                        inference.append((time.perf_counter_ns()-tick)/1000);env.step(*action)
                    metrics=env.metrics();metrics.update(case=case,scenario=scenario,episode_seed=seed,policy=policy,trained_policy_seed=checkpoint['config']['seed'],decision_runtime_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(inference,[50,95,99])))),mobil_evaluations=baseline.mobil_evaluations if policy=='FSM_IDM_MOBIL' else 0)
                    results.append(metrics)
                    # Keep representative paired raw traces; every run retains
                    # its summary. This avoids redundant multi-megabyte logs.
                    if seed==seeds[0]:
                        folder=output/'traces';folder.mkdir(exist_ok=True)
                        with gzip.open(folder/f'{case}_{scenario}_{policy}.jsonl.gz','wt') as stream:
                            for row in env.rows:stream.write(json.dumps(row)+'\n')
        print(json.dumps(dict(case=case,completed=len(results),wall_s=time.perf_counter()-started)),flush=True)
    grouped=[]
    for case in cases:
        for policy in ('FSM_IDM_MOBIL','PPO_guarded','PPO_without_predictive_gate'):
            runs=[r for r in results if r['case']==case and r['policy']==policy]
            grouped.append(dict(case=case,policy=policy,episodes=len(runs),model_acceptance_fraction=sum(r['status']=='PASS_MODEL_EXPERIMENT' for r in runs)/len(runs),collision_fraction=sum(r['collision_count']>0 for r in runs)/len(runs),offroad_fraction=sum(r['offroad_count']>0 for r in runs)/len(runs),mean_return=float(np.mean([r['return_sum'] for r in runs])),mean_cte_rmse_m=float(np.mean([r['cte_rmse_m'] for r in runs])),mean_speed_rmse_m_s=float(np.mean([r['speed_rmse_m_s'] for r in runs])),max_measured_jerk_m_s3=max(r['max_jerk_m_s3'] for r in runs),mean_shield_interventions=float(np.mean([r['shield_interventions'] for r in runs]))))
    guarded=[r for r in results if r['policy']=='PPO_guarded'];report=dict(status='PASS_MODEL_EXPERIMENT' if all(r['status']=='PASS_MODEL_EXPERIMENT' for r in guarded) else 'PARTIAL',process_status='PASS_EVALUATION_PROCESS',engine=ENGINE,weights=str(weights),trained_steps=checkpoint['trained_steps'],training_seed=checkpoint['config']['seed'],evaluation_seeds=list(seeds),scenarios=list(scenarios),cases=list(cases),episodes=len(results),paired_evaluation=True,platform=platform.platform(),wall_s=time.perf_counter()-started,acceptance=dict(collision_count=0,offroad_count=0,cte_rmse_below_m=1,max_steering_rad=.5,max_rate_rad_s=.8,max_accel_m_s2=3,max_decel_m_s2=6,max_jerk_m_s3=6,full_horizon_samples=400,actual_motion_over_m=3),groups=grouped,runs=results,physical_validation='PENDING_HARDWARE',carla_policy_evaluation='NOT_EXECUTED')
    (output/'evaluation.json').write_text(json.dumps(report,indent=2))
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4));cases=list(cases);x=np.arange(len(cases));colors=['#42566e','#007f72','#ca7a1b']
    for index,policy in enumerate(('FSM_IDM_MOBIL','PPO_guarded','PPO_without_predictive_gate')):
        groups=[next(g for g in grouped if g['case']==case and g['policy']==policy) for case in cases]
        for axis,key in zip(axes,['model_acceptance_fraction','collision_fraction','mean_return']):axis.bar(x+(index-1)*.24,[g[key] for g in groups],.24,label=policy,color=colors[index])
    for axis,title in zip(axes,['Measured model gate fraction','Collision episode fraction','Mean episodic return']):axis.set_xticks(x,cases,rotation=25,ha='right');axis.set_title(title);axis.grid(axis='y')
    axes[0].legend(fontsize=7);fig.suptitle('Analytical Dynamic Bicycle — not CARLA / hardware');fig.tight_layout();fig.savefig(output/'id_ood_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='runs'},indent=2),flush=True);return report

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--weights',required=True);parser.add_argument('--output',default='artifacts/learning/evaluation_seed7');parser.add_argument('--seeds',default='101,211,307,401,503');parser.add_argument('--cases',default=','.join(CASES));parser.add_argument('--scenarios',default='free,mixed,braking');parser.add_argument('--process-only',action='store_true',help='CI execution check only; JSON retains all failed performance gates');args=parser.parse_args();report=evaluate(args.weights,args.output,tuple(map(int,args.seeds.split(','))),tuple(args.cases.split(',')),tuple(args.scenarios.split(',')))
    if report['status']!='PASS_MODEL_EXPERIMENT' and not args.process_only:raise SystemExit(1)
if __name__=='__main__':main()
