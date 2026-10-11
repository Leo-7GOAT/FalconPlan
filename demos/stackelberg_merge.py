"""Reproducible numerical merge game, explicitly distinct from CARLA."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from behavior.stackelberg import (MergeState, MergeAction, payoff,
    solve_stackelberg, trajectory)


def cv_decision(state):
    """Greedy leader assumes CV opponent; identical payoffs/action space."""
    alternatives = [(MergeAction(a, merge), payoff(state, MergeAction(a, merge), 0))
                    for merge in (False, True) for a in (-4., -2., 0., 2.)]
    return max((x for x in alternatives if not x[1].collision), key=lambda x:x[1].leader)[0]


def follower_response(state, action, desired_speed):
    choices = [(a, payoff(state, action, a, follower_speed=desired_speed)) for a in (-4., -2., 0., 2.)]
    best = max(p.follower for _, p in choices)
    return min((x for x in choices if abs(x[1].follower-best)<1e-10), key=lambda x:(x[1].leader,x[0]))[0]


def run(output, seeds):
    output.mkdir(parents=True, exist_ok=False)
    records=[]
    with (output/'trajectories.jsonl').open('x',encoding='utf-8') as stream:
        for domain in ('id','ood_fast','ood_noncooperative'):
            for seed in seeds:
                rng=np.random.default_rng(seed)
                for scene in range(20):
                    state=MergeState(0,float(rng.uniform(8,14)),float(rng.uniform(-25,-3)),float(rng.uniform(8,14) if domain=='id' else rng.uniform(15,20)))
                    preference=14.0 if domain=='id' else 19.0
                    for policy in ('cv_greedy','stackelberg','stackelberg_robust'):
                        start=time.perf_counter_ns()
                        decision=None
                        if policy=='cv_greedy':action=cv_decision(state)
                        else:
                            decision=solve_stackelberg(state,robust_gate=policy=='stackelberg_robust')
                            action=decision.action
                        latency=(time.perf_counter_ns()-start)/1e6
                        response=2.0 if domain=='ood_noncooperative' else follower_response(state,action,preference)
                        actual=payoff(state,action,response,follower_speed=preference)
                        record={'domain':domain,'seed':seed,'scene':scene,'policy':policy,'state':vars(state),'action':vars(action),'actual_follower_acceleration':response,'assumed_follower_desired_speed':14.0,'actual_follower_desired_speed':preference,'leader_payoff':actual.leader,'follower_payoff':actual.follower,'min_normalized_separation':actual.clearance,'collision':actual.collision,'merge_complete':bool(action.merge and not actual.collision),'decision_ms':latency,'safety_veto':decision.safety_veto if decision else None,'predicted_response':decision.predicted_response if decision else 0.0,'payoff_matrix':decision.matrix if decision else None}
                        records.append(record)
                        t,s,v,l,other_s,other_v=trajectory(state,action,response)
                        for i in range(len(t)):
                            stream.write(json.dumps({'domain':domain,'seed':seed,'scene':scene,'policy':policy,'t_s':float(t[i]),'ego_s_m':float(s[i]),'ego_speed_mps':float(v[i]),'ego_lateral_m':float(l[i]),'other_s_m':float(other_s[i]),'other_speed_mps':float(other_v[i])})+'\n')
    summary={}
    for domain in ('id','ood_fast','ood_noncooperative'):
        summary[domain]={}
        for policy in ('cv_greedy','stackelberg','stackelberg_robust'):
            selected=[r for r in records if r['domain']==domain and r['policy']==policy]
            summary[domain][policy]={'runs':len(selected),'merges':sum(r['merge_complete'] for r in selected),'collisions':sum(r['collision'] for r in selected),'mean_leader_payoff':float(np.mean([r['leader_payoff'] for r in selected])),'min_normalized_separation':min(r['min_normalized_separation'] for r in selected),'decision_ms_p50_p95_p99':np.percentile([r['decision_ms'] for r in selected],[50,95,99]).tolist()}
    report={'status':'PASS_UNIT_ANALYTICAL_GAME_EXPERIMENT','high_fidelity_integration':'PARTIAL_NOT_RUN_IN_CARLA','model':'ANALYTICAL_CONSTANT_ACCELERATION_QUINTIC_MERGE_NOT_CARLA','seeds':seeds,'collision_threshold':'elliptical center separation <1; longitudinal axis 5m/lateral axis1.8m; piecewise-linear sampled relative segments','horizon_s':4,'dt_s':.05,'paired_scenarios':True,'opponent_action_set_mps2':[-4,-2,0,2],'assumption':'follower observes committed leader strategy; ID best response uses preference14; fast OOD best response uses19 while leader still assumes14; noncooperative OOD ignores intent and accelerates+2','summary':summary,'runs':records}
    (output/'benchmark.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    labels=['CV greedy','Stackelberg','Robust veto']
    for ax,domain in zip(axes,('id','ood_fast','ood_noncooperative')):
        values=[summary[domain][p] for p in ('cv_greedy','stackelberg','stackelberg_robust')]
        ax.bar(np.arange(3)-.18,[x['merges']/x['runs'] for x in values],.36,label='merge completed')
        ax.bar(np.arange(3)+.18,[x['collisions']/x['runs'] for x in values],.36,label='predicted collision')
        ax.set_xticks(range(3),labels,rotation=12);ax.set_ylim(0,1);ax.set_title(domain);ax.set_ylabel('fraction of paired scenarios');ax.legend()
    fig.suptitle('Analytical merge model: not CARLA physics');fig.tight_layout();fig.savefig(output/'benchmark.png',dpi=140);plt.close(fig)
    print(json.dumps({'model':report['model'],'summary':summary},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--seeds',type=int,nargs='+',default=[7,17,27]);args=parser.parse_args()
    run(args.output,args.seeds)
