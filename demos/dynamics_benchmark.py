"""Analytical model perturbation benchmark, explicitly separate from real CARLA."""
import json,time,platform
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from vehicle_model.dynamic_bicycle import DynamicState,DynamicBicycleModel,DynamicsParams
from vehicle_model.models import VehicleState,VehicleCommand,VehicleParams
from vehicle_model.kinematic_bicycle import KinematicBicycleModel
from control.first_order_actuator import FirstOrderActuator,ActuatorConfig
from control.stanley import StanleyController
from control.pid import PIDController

def run(model_kind,delay,gain,friction,speed):
    dynamic=DynamicBicycleModel(DynamicsParams(friction=friction));kinematic=KinematicBicycleModel(VehicleParams(2.8,.5,3,6,30))
    state=DynamicState(y=1,v=speed) if model_kind=='dynamic' else VehicleState(0,1,0,speed)
    actuator=FirstOrderActuator(ActuatorConfig(-.5,.5,.8,.12,delay,.003));controller=StanleyController(gain,1,.5)
    pid=PIDController(1,.1,0,-6,3);rows=[]
    for _ in range(1000):
        start=time.perf_counter_ns();delta=actuator.update(controller.update(-state.psi,-state.y,speed),.01);a=pid.update(speed,state.v,.01)
        state=dynamic.step(state,VehicleCommand(delta,a),.01) if model_kind=='dynamic' else kinematic.step(state,VehicleCommand(delta,a),.01)
        rows.append([state.x,state.y,state.psi,state.v,delta,(time.perf_counter_ns()-start)/1000])
    q=np.array(rows)
    return dict(model=model_kind,delay_s=delay,gain=gain,friction=friction,speed_m_s=speed,cte_rmse_m=float(np.sqrt(np.mean(q[:,1]**2))),heading_rmse_rad=float(np.sqrt(np.mean(q[:,2]**2))),max_steering_rad=float(np.max(np.abs(q[:,4]))),controller_model_runtime_us=dict(zip(['p50','p95','p99'],map(float,np.percentile(q[:,5],[50,95,99])))),final_cte_m=float(q[-1,1])),q

def main():
    out=Path('artifacts/dynamics');out.mkdir(parents=True,exist_ok=True);results=[];fig,axes=plt.subplots(2,1,figsize=(10,7))
    for kind,delay,gain,mu,speed in [('kinematic',0,1,.9,10),('dynamic',0,1,.9,10),('dynamic',.2,1.5,.7,10),('dynamic',.2,.6,.7,10),('dynamic',.2,.6,.4,20)]:
        metrics,q=run(kind,delay,gain,mu,speed);metrics['status']='PASS_UNIT' if metrics['cte_rmse_m']<1 else 'FAIL';results.append(metrics)
        label=f'{kind}, delay={delay}, k={gain}, μ={mu}, v={speed}'
        axes[0].plot(np.arange(len(q))*.01,q[:,1],label=label);axes[1].plot(np.arange(len(q))*.01,q[:,4],label=label)
    axes[0].set_ylabel('Lateral error [m]');axes[1].set_ylabel('Steering [rad]');axes[1].set_xlabel('Time [s]')
    for ax in axes:ax.legend(fontsize=8);ax.grid()
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=150)
    report=dict(status='PASS_UNIT',engine='ANALYTICAL_BICYCLE_NOT_CARLA',samples_per_run=1000,dt=.01,platform=platform.platform(),runs=results,lateral_linear_eigenvalues={str(v):[[float(e.real),float(e.imag)] for e in DynamicBicycleModel().lateral_eigenvalues(v)] for v in [2,5,10,20,30]})
    (out/'benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
