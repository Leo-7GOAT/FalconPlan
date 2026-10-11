import argparse,json,time,platform
from pathlib import Path
import numpy as np
from control.cpp_core import CppCore
from control.stanley import StanleyController
from vehicle_model.models import VehicleState,VehicleCommand,VehicleParams
from vehicle_model.kinematic_bicycle import KinematicBicycleModel
def main():
    p=argparse.ArgumentParser();p.add_argument('--library',required=True);p.add_argument('--output',default='artifacts/cpp/binding.json');a=p.parse_args();cpp=CppCore(a.library);python=StanleyController(1.5,1,.5);model=KinematicBicycleModel(VehicleParams(2.8,.5,3,6,30))
    rng=np.random.default_rng(42);errors=[];pt=[];ct=[]
    for _ in range(10000):
        h,e,v=float(rng.uniform(-3,3)),float(rng.uniform(-10,10)),float(rng.uniform(0,30));start=time.perf_counter_ns();expected=python.update(h,e,v);pt.append((time.perf_counter_ns()-start)/1000);start=time.perf_counter_ns();actual=cpp.stanley(h,e,v);ct.append((time.perf_counter_ns()-start)/1000);errors.append(abs(expected-actual))
    q=VehicleState(0,0,.1,10);u=VehicleCommand(.1,1);expected=model.step(q,u,.05);actual=cpp.bicycle(q,u,.05);assert max(abs(x-y) for x,y in zip(vars(expected).values(),vars(actual).values()))<1e-12
    try:cpp.stanley(0,0,-1);raise AssertionError('invalid input accepted')
    except ValueError:pass
    try:cpp.stanley(float('nan'),0,1);raise AssertionError('nan accepted')
    except ValueError:pass
    try:cpp.bicycle(q,u,0);raise AssertionError('invalid dt accepted')
    except ValueError:pass
    import ctypes
    assert cpp.lib.falcon_stanley(0,0,1,1,1,.5,None)==-1
    assert cpp.lib.falcon_bicycle(None,None,.1,None)==-1
    result=dict(status='PASS_UNIT',platform=platform.platform(),samples=10000,seed=42,max_absolute_error=max(errors),python_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(pt,[50,95,99])))),cpp_ffi_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(ct,[50,95,99])))),note='In-process ctypes dispatch included; tiny scalar kernel can be slower than Python due to FFI overhead.')
    assert result['max_absolute_error']<1e-12;Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
