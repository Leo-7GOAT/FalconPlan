"""Derive immutable evidence hashes and actor-only parameter changes locally."""
import hashlib,json
from pathlib import Path
import torch
from falcon.learning.ppo import ActorCritic,seed_everything,load_policy

root=Path(__file__).resolve().parents[2]
folder=root/'artifacts/learning'
training=[]
for name in ('ppo_seed7','ppo_v2_seed7','ppo_v2_seed19','ppo_v3_seed7','ppo_v3_seed19','ci_smoke'):
    path=folder/name
    report=json.loads((path/'training_summary.json').read_text())
    trained,checkpoint=load_policy(path/'policy.pt')
    seed_everything(checkpoint['config']['seed']);initial=ActorCritic()
    keys=[k for k in trained.state_dict() if not k.startswith('critic.')]
    delta=torch.cat([(trained.state_dict()[k]-initial.state_dict()[k]).flatten() for k in keys])
    report.update(directory=name,actor_parameter_delta_l2=float(torch.linalg.vector_norm(delta)),reward_version=checkpoint['config'].get('reward_version',1),actuator_envelope_version=checkpoint['config'].get('actuator_envelope_version',1))
    assert report['weights_sha256']==hashlib.sha256((path/'policy.pt').read_bytes()).hexdigest()
    training.append(report)
evaluations=[]
for name in ('evaluation_seed7','evaluation_v2_seed7','evaluation_v3_seed7','evaluation_v3_seed19','ci_smoke/evaluation'):
    path=folder/name/'evaluation.json';report=json.loads(path.read_text())
    evaluations.append(dict(directory=name,episodes=report['episodes'],status=report['status'],process_status=report.get('process_status','NOT_RECORDED_IN_LEGACY_REPORT'),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
sources=sorted([*root.glob('falcon/learning/*.py'),*root.glob('demos/learning/*.py'),*root.glob('tests/test_learning*.py')])
manifest=dict(status='PASS_EVIDENCE_INTEGRITY',policy_performance='PARTIAL',engine='ANALYTICAL_DYNAMIC_BICYCLE_NOT_CARLA',training=training,evaluations=evaluations,total_actual_training_transitions=sum(r['trained_steps'] for r in training),total_evaluation_episodes=sum(r['episodes'] for r in evaluations),final_source_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},source_hash_scope='final implementation; older artifacts select retained reward/envelope version branches, not historical source hashes',physical_validation='PENDING_HARDWARE',carla_policy_evaluation='NOT_EXECUTED')
(folder/'reproducibility_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest,indent=2))
