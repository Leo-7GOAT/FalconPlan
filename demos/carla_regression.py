"""Real synchronous CARLA physics regression. Never falls back to a mock.

Run in WSL with the matching 0.9.15 client and a running Windows server.
"""
import argparse,json,math,time,platform,queue
from pathlib import Path
import numpy as np
from hal.carla_adapter import CarlaAdapter
from control.stanley import StanleyController
from control.pid import PIDController
from control.safety import SafetySupervisor,SafetyConfig
from control.first_order_actuator import ActuatorConfig,FirstOrderActuator
from vehicle_model.models import VehicleCommand
from behavior.idm import compute_idm_acceleration
from behavior.models import IDMParams

SCENARIOS=['straight','curve','static_obstacle','following','actuator_delay','state_timeout','command_timeout']
def angle(a):return math.atan2(math.sin(a),math.cos(a))
def make_routes(world):
    routes=[]
    for spawn in world.get_map().get_spawn_points():
        waypoint=world.get_map().get_waypoint(spawn.location);route=[waypoint]
        for _ in range(100):
            choices=route[-1].next(1.)
            if not choices:break
            route.append(min(choices,key=lambda w:abs(angle(math.radians(w.transform.rotation.yaw-route[-1].transform.rotation.yaw)))))
        if len(route)<70:continue
        curvature=sum(abs(angle(math.radians(b.transform.rotation.yaw-a.transform.rotation.yaw))) for a,b in zip(route[:60],route[1:60]))
        routes.append((curvature,route))
    if not routes:raise RuntimeError('no usable CARLA map route')
    return sorted(routes,key=lambda q:q[0])

def run_one(client,world,route,scenario,seed,steps,outdir,use_mpc=False,full_stack=False,cpp_lib=None):
    import carla
    rng=np.random.default_rng(seed);actors=[];collision=[];frames=queue.Queue(maxsize=8);estimator=None
    blueprint=world.get_blueprint_library().find('vehicle.tesla.model3')
    spawn=route[0].transform;spawn.location.z += .5
    ego=world.spawn_actor(blueprint,spawn);actors.append(ego)
    try:
        sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=ego);actors.append(sensor)
        sensor.listen(lambda e:collision.append(dict(frame=e.frame,other=e.other_actor.id)))
        rgb=world.get_blueprint_library().find('sensor.camera.rgb');rgb.set_attribute('image_size_x','640');rgb.set_attribute('image_size_y','360');rgb.set_attribute('sensor_tick','.2')
        camera=world.spawn_actor(rgb,carla.Transform(carla.Location(x=-6,z=3),carla.Rotation(pitch=-15)),attach_to=ego);actors.append(camera)
        def collect(img):
            try:frames.put_nowait(img)
            except queue.Full:pass
        camera.listen(collect)
        front=None
        if scenario in ('following','static_obstacle'):
            loc=route[28].transform;loc.location.z += .5
            front=world.spawn_actor(blueprint,loc);actors.append(front)
            if scenario=='static_obstacle':front.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
        for _ in range(20):world.tick()
        adapter=CarlaAdapter(client,world,ego);cfg=SafetyConfig(max_speed=12,max_steer=.5)
        from hal.carla_sensors import CarlaSensorEstimator
        estimator=CarlaSensorEstimator(world,ego,adapter)
        safety=SafetySupervisor(cfg);stanley=StanleyController(1.5,1,.5)
        cpp=None
        if cpp_lib:
            from control.cpp_core import CppCore
            cpp=CppCore(cpp_lib)
        if use_mpc:
            from planning.longitudinal_mpc import LongitudinalMPC,LongitudinalMPCConfig,LongitudinalState,MPCReferencePoint
            mpc=LongitudinalMPC(LongitudinalMPCConfig(dt=.1,horizon_steps=15,max_speed=12,max_accel=3,max_decel=6,max_jerk=6,weight_s=.05,weight_v=8))
        pid=PIDController(1.5,.2,.05,-6,3)
        steer_act=FirstOrderActuator(ActuatorConfig(-.5,.5,.8,time_constant=.15,delay=.15 if scenario=='actuator_delay' else 0))
        accel_act=FirstOrderActuator(ActuatorConfig(-6,3,10,time_constant=.2,delay=.1 if scenario=='actuator_delay' else 0))
        xy=np.array([[w.transform.location.x,-w.transform.location.y] for w in route])
        yaw=np.unwrap([-math.radians(w.transform.rotation.yaw) for w in route]);sref=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        if scenario=='static_obstacle' and not full_stack:
            # Deterministic smooth spatial detour, verified using real collision sensor.
            offset=3.5*np.where((sref>12)&(sref<46),np.sin(np.pi*np.clip((sref-12)/34,0,1))**2,0)
            xy[:,0]-=np.sin(yaw)*offset;xy[:,1]+=np.cos(yaw)*offset
            yaw=np.arctan2(np.gradient(xy[:,1]),np.gradient(xy[:,0]))
        rows=[];s=0.;acc=0.;previous_speed=adapter.get_state().v;start_stamp=adapter.timestamp;cmd=VehicleCommand(0,0);nearest=0;images=[]
        target=4.0 if scenario=='curve' else 6.0
        if scenario=='actuator_delay':target=5.0+float(rng.uniform(-.3,.3))
        plan=None;plan_time=0;plan_metadata={};estimated_acc=0
        if full_stack:
            from planning.online_stack import OnlinePlanningStack,sample_profile
            online_stack=OnlinePlanningStack(xy.tolist(),target)
        for tick in range(steps):
            loop_start=time.perf_counter();state=adapter.get_state();now=adapter.timestamp
            estimated_state=estimator.update(.05)
            previous_acc=acc;acc=(state.v-previous_speed)/.05;previous_speed=state.v;s+=state.v*.05
            estimated_acc=.8*estimated_acc+.2*acc
            if state.v<.1 and estimated_acc<0:estimated_acc=0
            if front is not None and scenario=='following':front.apply_control(carla.VehicleControl(throttle=.15,brake=0))
            point=np.array([state.x+adapter.wheelbase*math.cos(state.psi),state.y+adapter.wheelbase*math.sin(state.psi)])
            end=min(len(xy),nearest+30);nearest=nearest+int(np.argmin(np.sum((xy[nearest:end]-point)**2,axis=1)))
            cte=float(np.dot(xy[nearest]-point,[-math.sin(yaw[nearest]),math.cos(yaw[nearest])]))
            heading=angle(float(yaw[nearest])-state.psi);planning_start=time.perf_counter();speed_target=target;gap=None
            online=adapter.nearby_actors()
            if front is not None and scenario=='following':
                for actor in online:
                    if actor['id']==front.id:actor['motion_type']='moving'
            if full_stack and tick%10==0:
                try:
                    plan=online_stack.plan(state,online,now,float(np.clip(estimated_acc,-6,3)));plan_time=now;plan_metadata=plan.metadata
                    xy=np.array(list(zip(plan.geometry.x,plan.geometry.y)));yaw=np.array(plan.geometry.yaw);nearest=0
                except (ValueError,RuntimeError) as exc:
                    plan_metadata=dict(failure=str(exc));plan=None
            if full_stack and plan:
                point=np.array([state.x+adapter.wheelbase*math.cos(state.psi),state.y+adapter.wheelbase*math.sin(state.psi)])
                nearest=int(np.argmin(np.sum((xy-point)**2,axis=1)))
                cte=float(np.dot(xy[nearest]-point,[-math.sin(yaw[nearest]),math.cos(yaw[nearest])]))
                heading=angle(float(yaw[nearest])-state.psi)
                speed_target=sample_profile(plan.speed_profile,min(now-plan_time+.5,4)).v
            if scenario=='following' and front:
                f=next((o for o in online if o['id']==front.id),None)
                if f:
                    gap=(f['x']-state.x)*math.cos(state.psi)+(f['y']-state.y)*math.sin(state.psi)-5
                    if gap>0:
                        fv=math.hypot(f['vx'],f['vy']);idm=compute_idm_acceleration(state.v,gap,fv,IDMParams(desired_speed=target,max_accel=3,comfortable_decel=2,min_gap=4,time_headway=1.5))
                        speed_target=max(0,min(target,state.v+idm*.5))
            planner_ms=(time.perf_counter()-planning_start)*1000
            control_start=time.perf_counter();steering=cpp.stanley(heading,cte,max(0,state.v)) if cpp else stanley.update(heading,cte,max(0,state.v))
            solver_status='PID'
            if use_mpc:
                if full_stack and plan:
                    origin=online_stack.project(state).s-plan.origin_s
                    refs=[MPCReferencePoint(now+k*.1,r.s,r.v,r.a) for k in range(16) for r in [sample_profile(plan.speed_profile,now-plan_time+k*.1)]]
                else:
                    origin=s
                    refs=[MPCReferencePoint(now+k*.1,s+speed_target*k*.1,speed_target,0) for k in range(16)]
            if not use_mpc:
                cmd=VehicleCommand(steering,pid.update(speed_target,state.v,.05))
            elif tick%4==0:
                try:
                    r=mpc.solve(LongitudinalState(origin,max(0,state.v),float(np.clip(estimated_acc,-6,3))),refs)
                    cmd=VehicleCommand(steering,float(np.clip(r.predicted_states[2].a+.8*(r.predicted_states[4].v-state.v),-6,3)));solver_status=r.status
                except (ValueError,RuntimeError) as exc:
                    cmd=VehicleCommand(steering,-6);solver_status='FAIL:'+str(exc)
            else:cmd=VehicleCommand(steering,cmd.a)
            if full_stack and plan is None:cmd=VehicleCommand(0,-6);solver_status='FAIL:planner rejected live state'
            if gap is not None and gap<max(4,state.v*.8):cmd=VehicleCommand(steering,-6)
            result=safety.evaluate(state,cmd,now=now,state_time=now-.5 if scenario=='state_timeout' and tick>=steps//2 else now,command_time=now-.5 if scenario=='command_timeout' and tick>=steps//2 else now,dt=.05,collision=bool(collision))
            # Emergency safety output bypasses queued actuator commands.
            actual=result.command if result.mode.value=='stopped' else VehicleCommand(steer_act.update(result.command.delta,.05),accel_act.update(result.command.a,.05))
            adapter.apply_command(actual);controller_ms=(time.perf_counter()-control_start)*1000
            rows.append(dict(t=now-start_stamp,x=state.x,y=state.y,yaw=state.psi,v=state.v,estimated_x=estimated_state.x,estimated_y=estimated_state.y,estimated_v=estimated_state.v,position_estimation_error=math.hypot(state.x-estimated_state.x,state.y-estimated_state.y),target_speed=speed_target,cte=cte,heading_error=heading,steering=actual.delta,command_acceleration=actual.a,acceleration=float(acc),jerk=float((acc-previous_acc)/.05),safety=result.mode.value,reasons=result.reasons,planner_ms=planner_ms,controller_ms=controller_ms,solver=solver_status,actors=online,gap=gap,online_plan=plan_metadata,loop_wall_ms=(time.perf_counter()-loop_start)*1000))
            adapter.step(.05)
            while not frames.empty():
                img=frames.get_nowait()
                if tick%20==0:images.append(np.frombuffer(img.raw_data,dtype=np.uint8).reshape(img.height,img.width,4)[:,:,:3][:,:,::-1].copy())
        name=f'{scenario}_seed{seed}';folder=outdir/name;folder.mkdir(parents=True,exist_ok=True)
        (folder/'state.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
        if images:
            from PIL import Image
            pictures=[Image.fromarray(im) for im in images];pictures[-1].save(folder/'screenshot.png');pictures[0].save(folder/'simulation.gif',save_all=True,append_images=pictures[1:],duration=200,loop=0)
        rmse=lambda key:float(np.sqrt(np.mean([r[key]**2 for r in rows])))
        runtime=lambda key:dict(zip(['p50','p95','p99'],map(float,np.percentile([r[key] for r in rows],[50,95,99]))))
        summary=dict(scenario=scenario,seed=seed,steps=steps,simulation_seconds=steps*.05,status='PASS_REAL_SIM' if not collision and all(not r['solver'].startswith('FAIL') for r in rows) else 'FAIL',cte_rmse_m=rmse('cte'),heading_rmse_rad=rmse('heading_error'),speed_rmse_m_s=float(np.sqrt(np.mean([(r['v']-r['target_speed'])**2 for r in rows]))),max_steering_rad=max(abs(r['steering']) for r in rows),max_steering_rate_rad_s=max(abs(b['steering']-a['steering'])/.05 for a,b in zip(rows,rows[1:])),max_acceleration_m_s2=max(abs(r['acceleration']) for r in rows),max_jerk_m_s3=max(abs(r['jerk']) for r in rows),collision_count=len(collision),safety_intervention_count=safety.interventions,planner_ms=runtime('planner_ms'),controller_ms=runtime('controller_ms'),loop_wall_ms=runtime('loop_wall_ms'),reference=xy.tolist(),minimum_following_gap_m=min([r['gap'] for r in rows if r['gap'] is not None],default=None),final_speed=rows[-1]['v'],truth_source='CARLA actor ground truth; no sensor perception',config=dict(dt=.05,target_speed=target,steering_delay_s=.15 if scenario=='actuator_delay' else 0))
        displacement=math.hypot(rows[-1]['x']-rows[0]['x'],rows[-1]['y']-rows[0]['y'])
        acceptance=dict(no_collision=not collision,no_solver_failure=all(not r['solver'].startswith('FAIL') for r in rows),tracking_rmse=summary['cte_rmse_m']<1,steering_limit=summary['max_steering_rad']<=.5+1e-6,actual_motion=displacement>3)
        if scenario in ('state_timeout','command_timeout'):
            acceptance.update(fault_latched=rows[-1]['safety']=='stopped',stopped=rows[-1]['v']<.1)
        elif scenario=='static_obstacle':acceptance['passed_obstacle']=displacement>32
        elif scenario=='following':acceptance['gap_safe']=summary['minimum_following_gap_m'] is not None and summary['minimum_following_gap_m']>4
        summary.update(status='PASS_REAL_SIM' if all(acceptance.values()) else 'FAIL',acceptance=acceptance,displacement_m=displacement,full_stack=full_stack,longitudinal_controller='MPC' if use_mpc else 'PID',measured_acceleration_within_nominal_6_m_s2=summary['max_acceleration_m_s2']<=6,acceleration_note='Unclipped finite difference of simulator speed; nominal command limits do not certify physical acceleration.',steering_rate_note='Emergency output bypasses normal rate limit; physical hardware validation pending.')
        summary.update(real_sim_engine_status='PASS_REAL_SIM',motion_status=summary['status'],acceleration_envelope_status='PASS_REAL_SIM' if summary['max_acceleration_m_s2']<=6 else 'FAIL',jerk_envelope_status='PASS_REAL_SIM' if summary['max_jerk_m_s3']<=6 else 'PARTIAL',gnss_imu_position_rmse_m=rmse('position_estimation_error'),gnss_updates=estimator.accepted,gnss_outliers=estimator.rejected,imu_rejected_packets=estimator.invalid_imu,state_estimation_note='Real CARLA GNSS/IMU with local map georeference calibration; diagnostics only, controller localization uses ground truth.')
        summary['max_positive_acceleration_m_s2']=max(r['acceleration'] for r in rows)
        summary['min_acceleration_m_s2']=min(r['acceleration'] for r in rows)
        summary['acceleration_envelope_status']='PASS_REAL_SIM' if summary['max_positive_acceleration_m_s2']<=3+1e-5 and summary['min_acceleration_m_s2']>=-6-1e-5 else 'FAIL'
        summary['estimation_status']='PASS_REAL_SIM' if summary['gnss_imu_position_rmse_m']<.5 and estimator.accepted>steps*.5 else 'FAIL'
        summary['lateral_runtime']='C++17_IN_PROCESS' if cpp else 'PYTHON'
        summary['physical_constraint_status']='PASS_REAL_SIM' if summary['acceleration_envelope_status']=='PASS_REAL_SIM' and summary['jerk_envelope_status']=='PASS_REAL_SIM' else 'PARTIAL'
        if summary['status']=='PASS_REAL_SIM' and (summary['physical_constraint_status']=='PARTIAL' or summary['estimation_status']=='FAIL'):summary['status']='PARTIAL'
        (folder/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='reference'}),flush=True)
        return summary
    finally:
        if estimator is not None:estimator.close()
        for actor in reversed(actors):
            if hasattr(actor,'stop'):actor.stop()
            actor.destroy()

def main():
    import carla
    parser=argparse.ArgumentParser();parser.add_argument('--host',default='172.21.16.1');parser.add_argument('--port',type=int,default=2000);parser.add_argument('--steps',type=int,default=200);parser.add_argument('--seeds',default='7,19');parser.add_argument('--scenarios',default=','.join(SCENARIOS));parser.add_argument('--output',default='artifacts/carla');parser.add_argument('--mpc',action='store_true');parser.add_argument('--full-stack',action='store_true');parser.add_argument('--cpp-lib');a=parser.parse_args()
    client=carla.Client(a.host,a.port);client.set_timeout(20);world=client.get_world();original=world.get_settings();settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10;world.apply_settings(settings)
    output=Path(a.output);output.mkdir(parents=True,exist_ok=True)
    results=[]
    try:
        routes=make_routes(world)
        for scenario in a.scenarios.split(','):
            if scenario not in SCENARIOS:raise ValueError(scenario)
            # curve route begins with bend; straight is minimum accumulated yaw.
            route=routes[min(len(routes)-1,len(routes)//2)][1] if scenario=='curve' else routes[0][1]
            for seed in map(int,a.seeds.split(',')):results.append(run_one(client,world,route,scenario,seed,a.steps,output,a.mpc,a.full_stack,a.cpp_lib))
        report=dict(client_version=client.get_client_version(),server_version=client.get_server_version(),map=world.get_map().name,platform=platform.platform(),engine='REAL_CARLA',runs=results)
        (output/'regression.json').write_text(json.dumps(report,indent=2))
    finally:world.apply_settings(original)
    if any(r['status']!='PASS_REAL_SIM' for r in results):raise SystemExit(1)
if __name__=='__main__':main()
