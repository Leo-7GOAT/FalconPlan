"""Executed Python/C++ RK4 and Stanley parity; deterministic numerical benchmark."""
import argparse,json,subprocess,time,platform
from pathlib import Path
import numpy as np
from vehicle_model.kinematic_bicycle import KinematicBicycleModel
from vehicle_model.models import VehicleState,VehicleCommand,VehicleParams
from control.stanley import StanleyController
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--probe',required=True);parser.add_argument('--output',default='artifacts/cpp/parity.json');a=parser.parse_args()
    rng=np.random.default_rng(42);inputs=rng.uniform(size=(1000,7))
    inputs[:,0:2]=inputs[:,0:2]*100-50;inputs[:,2]=inputs[:,2]*6-3;inputs[:,3]*=25;inputs[:,4]=inputs[:,4]-0.5;inputs[:,5]=inputs[:,5]*9-6;inputs[:,6]=inputs[:,6]*.09+.01
    model=KinematicBicycleModel(VehicleParams(2.8,.5,3,6,30));controller=StanleyController(1,1,.5);expected=[];timings=[]
    for x,y,yaw,v,d,acc,dt in inputs:
        start=time.perf_counter_ns();s=model.step(VehicleState(x,y,yaw,v),VehicleCommand(d,acc),dt);expected.append([s.x,s.y,s.psi,s.v,controller.update(yaw,y,v)]);timings.append((time.perf_counter_ns()-start)/1000)
    payload='\n'.join(' '.join(map(str,row)) for row in inputs)+'\n';start=time.perf_counter_ns();proc=subprocess.run([a.probe],input=payload,text=True,capture_output=True,check=True);elapsed=(time.perf_counter_ns()-start)/1e6
    actual=np.loadtxt(proc.stdout.splitlines());error=float(np.max(np.abs(actual-np.asarray(expected))));assert error<1e-11,error
    result=dict(status='PASS_UNIT',seed=42,samples=len(inputs),max_absolute_error=error,python_microseconds=dict(zip(['p50','p95','p99'],map(float,np.percentile(timings,[50,95,99])))),cpp_process_and_text_io_ms=elapsed,platform=platform.platform(),note='C++ total includes process launch and text I/O; not a kernel speedup claim')
    Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
