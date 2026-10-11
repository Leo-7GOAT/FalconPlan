"""Seeded batch parity and measured cost/collision/selection runtime.

python -m tools.cpp_planning_benchmark --library build/cpp/libfalcon_c.so
Timing includes Python packing, ctypes, native allocations and result unpacking.
This is the candidate evaluator only, not end-to-end planning latency.
"""
import argparse
import json
import platform
import time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from control.cpp_core import CppCore
from planning.collision import CircularObstacle,CollisionParams,trajectory_collides
from planning.cost import TrajectoryCostWeights,compute_trajectory_cost
from planning.trajectory import FrenetTrajectory

def python_rank(candidates,obstacles,params,weights,desired_d,desired_speed):
    costs=[compute_trajectory_cost(c,desired_d,desired_speed,weights) for c in candidates]
    blocked=[trajectory_collides(c,obstacles,params) for c in candidates]
    indices=[i for i,b in enumerate(blocked) if not b]
    return min(indices,key=lambda i:costs[i].total) if indices else None,costs,blocked

def random_batch(rng,batch):
    candidates=[]
    n=(1,2,41,81)[batch%4]
    for _ in range(8):
        x=np.linspace(0,40,n)+rng.normal(0,.02,n)
        y=np.linspace(0,float(rng.uniform(-5,5)),n)+rng.normal(0,.02,n)
        candidates.append(FrenetTrajectory(t=list(np.linspace(0,4,n)),x=list(x),y=list(y),d_ddd=list(rng.normal(0,4,n)),s_ddd=list(rng.normal(0,3,n)),target_d=float(rng.uniform(-4,4)),target_speed=float(rng.uniform(0,12)),duration=float(rng.uniform(.5,8))))
    obstacles=[CircularObstacle(float(rng.uniform(0,40)),float(rng.uniform(-6,6)),float(rng.uniform(0,1.5))) for _ in range(3)]
    params=CollisionParams(float(rng.uniform(0,1.2)),float(rng.uniform(0,.5)))
    weights=TrajectoryCostWeights(*map(float,rng.uniform(0,20,5)))
    return candidates,obstacles,params,weights,float(rng.uniform(-4,4)),float(rng.uniform(0,12))

def verify(library,output,batches=250,seed=42):
    native=CppCore(library);rng=np.random.default_rng(seed);error=0;py_times=[];cpp_times=[];blocked_count=0
    for batch in range(batches):
        args=random_batch(rng,batch)
        begin=time.perf_counter_ns();reference=python_rank(*args);py_times.append((time.perf_counter_ns()-begin)/1000)
        begin=time.perf_counter_ns();actual=native.rank_candidates(*args);cpp_times.append((time.perf_counter_ns()-begin)/1000)
        assert actual[0]==reference[0] and actual[2]==reference[2],f'selection/collision mismatch batch {batch}'
        blocked_count+=sum(actual[2])
        for expected,result in zip(reference[1],actual[1]):
            delta=np.max(np.abs(np.array(list(asdict(expected).values()))-np.array(list(asdict(result).values()))))
            error=max(error,float(delta))
    assert error<1e-9,f'numerical parity error {error}'
    # Real foreign calls reject bad numeric values rather than selecting NaN.
    args=list(random_batch(rng,2));args[0][0].x[0]=float('nan')
    try:native.rank_candidates(*args);raise AssertionError('NaN accepted')
    except ValueError:pass
    args=list(random_batch(rng,2));args[1]=[CircularObstacle(0,0,-1)]
    try:native.rank_candidates(*args);raise AssertionError('negative obstacle accepted')
    except ValueError:pass
    # Tangency between sparse samples and stable ties have separate unit tests.
    result=dict(status='PASS_UNIT',seed=seed,batches=batches,candidates=batches*8,blocked_candidates=blocked_count,max_absolute_cost_error=error,collision_mismatches=0,selection_mismatches=0,platform=platform.platform(),python_batch_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(py_times,[50,95,99])))),cpp_batch_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(cpp_times,[50,95,99])))),note='Both timings include a batch of 8 candidates. Native timing includes packing, ctypes, allocations and cost result construction. No full-stack real-time or target hardware claim.')
    Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(result,indent=2));return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--library',required=True);parser.add_argument('--output',default='artifacts/cpp_planning/parity_benchmark.json');parser.add_argument('--batches',type=int,default=250);args=parser.parse_args()
    if args.batches<=0:parser.error('--batches must be positive')
    print(json.dumps(verify(args.library,args.output,args.batches),indent=2))
if __name__=='__main__':main()
