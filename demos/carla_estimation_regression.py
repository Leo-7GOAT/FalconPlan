"""Actual CARLA GNSS/IMU estimated-state feedback regression; no fallback.

Route is static OpenDRIVE configuration. Real ego actor truth is evaluated only
AFTER issuing control and cannot enter initialization, steering, PID or servo.
"""
import argparse
import json
import math
import platform
import time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from hal.carla_adapter import CarlaAdapter
from hal.carla_sensors import CarlaSensorEstimator,SensorFaults
from vehicle_model.models import VehicleState,VehicleCommand
from control.stanley import StanleyController
from control.pid import PIDController
from control.safety import SafetySupervisor,SafetyConfig
from control.first_order_actuator import FirstOrderActuator,ActuatorConfig
from demos.carla_regression import make_routes,angle

PROFILES=('ideal_straight','ideal_curve','noisy_curve','short_dropout','timestamp_disorder','long_dropout')

class FeedbackVehicle:
    """Fails immediately if control requests any actor ground-truth state."""
    def __init__(self,actor):self.actor=actor;self.sealed=False;self.blocked_reads=0
    def _read(self,name):
        if self.sealed:
            self.blocked_reads+=1
            raise RuntimeError('GT_ACCESS_IN_ESTIMATED_FEEDBACK:'+name)
        return getattr(self.actor,name)()
    def get_transform(self):return self._read('get_transform')
    def get_velocity(self):return self._read('get_velocity')
    def get_acceleration(self):return self._read('get_acceleration')
    def get_physics_control(self):return self.actor.get_physics_control()
    def apply_control(self,command):self.actor.apply_control(command)
    def apply_ackermann_control(self,command):self.actor.apply_ackermann_control(command)

def evaluation_truth(actor,rear_offset):
    transform=actor.get_transform();velocity=actor.get_velocity();psi=-math.radians(transform.rotation.yaw)
    speed=velocity.x*math.cos(psi)-velocity.y*math.sin(psi)
    return VehicleState(transform.location.x-rear_offset*math.cos(psi),-transform.location.y-rear_offset*math.sin(psi),psi,speed)

def errors(state,xy,yaw,wheelbase):
    point=np.array([state.x+wheelbase*math.cos(state.psi),state.y+wheelbase*math.sin(state.psi)])
    nearest=int(np.argmin(np.sum((xy-point)**2,axis=1)))
    cte=float(np.dot(xy[nearest]-point,[-math.sin(yaw[nearest]),math.cos(yaw[nearest])]))
    return cte,angle(float(yaw[nearest])-state.psi)

def run_one(client,world,route,profile,seed,steps,output):
    import carla
    actors=[];estimator=None;collisions=[];rows=[];startup=[]
    folder=output/f'{profile}_seed{seed}';folder.mkdir(parents=True,exist_ok=True)
    try:
        transform=carla.Transform(route[0].transform.location,route[0].transform.rotation);transform.location.z+=.5
        ego=world.spawn_actor(world.get_blueprint_library().find('vehicle.tesla.model3'),transform);actors.append(ego)
        ego.apply_control(carla.VehicleControl(brake=1,hand_brake=True))
        collision_sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),carla.Transform(),attach_to=ego);actors.append(collision_sensor)
        collision_sensor.listen(lambda e:collisions.append({'frame':e.frame,'other_actor':e.other_actor.id}))
        for _ in range(20):world.tick() # physical stationary settle before sensor initialization
        guard=FeedbackVehicle(ego);adapter=CarlaAdapter(client,world,guard);guard.sealed=True
        faults=SensorFaults()
        if profile=='short_dropout':faults=SensorFaults(dropout_windows=((4.,4.1),(7.,7.1)))
        if profile=='long_dropout':faults=SensorFaults(dropout_windows=((6.,7.),))
        if profile=='timestamp_disorder':faults=SensorFaults(reorder_every=20,future_every=30)
        noise=profile in ('noisy_curve','short_dropout','timestamp_disorder','long_dropout')
        estimator=CarlaSensorEstimator(world,ego,adapter,noise=noise,seed=seed,faults=faults)
        for _ in range(40):
            frame=world.tick();now=world.get_snapshot().timestamp.elapsed_seconds
            state=estimator.update(now=now,frame=frame)
            gt=evaluation_truth(ego,adapter.rear_offset)
            startup.append({'frame':frame,'timestamp':now,'sensor_ready':estimator.ready,'offline_truth':asdict(gt)})
            if estimator.ready:break
        if not estimator.ready:raise RuntimeError('sensor-only stationary initialization timed out')
        adapter.use_estimated_feedback(estimator)
        ego.apply_control(carla.VehicleControl(brake=0,hand_brake=False,manual_gear_shift=True,gear=1))
        stanley=StanleyController(1.5,1,.5);pid=PIDController(1.5,.2,.05,-6,3);safety=SafetySupervisor(SafetyConfig(max_speed=12,max_steer=.5))
        steer=FirstOrderActuator(ActuatorConfig(-.5,.5,.8,time_constant=.15));accel=FirstOrderActuator(ActuatorConfig(-6,3,10,time_constant=.2))
        xy=np.array([[w.transform.location.x,-w.transform.location.y] for w in route]);yaw=np.unwrap([-math.radians(w.transform.rotation.yaw) for w in route])
        previous_gt=evaluation_truth(ego,adapter.rear_offset);previous_acc=0.;start_stamp=now
        target=4. if 'curve' in profile else 6.
        for tick in range(steps):
            wall_start=time.perf_counter();snapshot=world.get_snapshot();now=snapshot.timestamp.elapsed_seconds
            estimator.update(now=now,frame=snapshot.frame)
            state=adapter.get_state();cte,heading=errors(state,xy,yaw,adapter.wheelbase)
            command=VehicleCommand(stanley.update(heading,cte,max(0,state.v)),pid.update(target,state.v,.05))
            result=safety.evaluate(state,command,now=now,state_time=estimator.timestamp,command_time=now,dt=.05,collision=bool(collisions))
            actual=result.command if result.mode.value=='stopped' else VehicleCommand(steer.update(result.command.delta,.05),accel.update(result.command.a,.05))
            adapter.apply_command(actual)
            # The entire feedback decision above is complete before evaluation.
            gt=evaluation_truth(ego,adapter.rear_offset);gt_cte,gt_heading=errors(gt,xy,yaw,adapter.wheelbase)
            physical_acc=(gt.v-previous_gt.v)/.05;physical_jerk=(physical_acc-previous_acc)/.05
            rows.append({'frame':snapshot.frame,'t':now-start_stamp,'timestamp':now,'feedback_source':'GNSS_IMU_EKF','estimated_state':asdict(state),'feedback_timestamp':estimator.timestamp,'feedback_age_s':now-estimator.timestamp,'offline_truth':asdict(gt),'position_error_m':math.hypot(state.x-gt.x,state.y-gt.y),'speed_error_m_s':state.v-gt.v,'yaw_error_rad':angle(state.psi-gt.psi),'feedback_cte':cte,'offline_cte':gt_cte,'offline_heading_error':gt_heading,'target_speed':target,'steering':actual.delta,'command_acceleration':actual.a,'physical_acceleration':physical_acc,'physical_jerk':physical_jerk,'safety':result.mode.value,'safety_reasons':result.reasons,'loop_wall_ms':(time.perf_counter()-wall_start)*1000})
            previous_gt=gt;previous_acc=physical_acc
            world.tick()
        # Include the final physical tick rather than silently dropping its result.
        snapshot=world.get_snapshot();estimator.update(now=snapshot.timestamp.elapsed_seconds,frame=snapshot.frame)
        final_gt=evaluation_truth(ego,adapter.rear_offset)
        terminal_acc=(final_gt.v-previous_gt.v)/.05;terminal_jerk=(terminal_acc-previous_acc)/.05
        rmse=lambda key:float(np.sqrt(np.mean([r[key]**2 for r in rows])))
        acceleration=[r['physical_acceleration'] for r in rows]+[terminal_acc]
        jerk=[r['physical_jerk'] for r in rows]+[terminal_jerk]
        displacement=math.hypot(final_gt.x-rows[0]['offline_truth']['x'],final_gt.y-rows[0]['offline_truth']['y'])
        feedback_checks={'no_gt_reads':guard.blocked_reads==0,'no_collision':not collisions,'sensor_initialized':estimator.initialization['source']=='REAL_GNSS_IMU_COMPASS_STATIONARY_HOLD','position_rmse_under_0_5m':rmse('position_error_m')<.5,'speed_rmse_under_0_5m_s':rmse('speed_error_m_s')<.5,'yaw_rmse_under_0_05rad':rmse('yaw_error_rad')<.05,'tracking_rmse_under_1m':rmse('offline_cte')<1,'actual_motion_over_3m':displacement>3,'steering_limit':max(abs(r['steering']) for r in rows)<=.5+1e-6}
        if profile=='long_dropout':feedback_checks.update(timeout_latched=rows[-1]['safety']=='stopped' and any('state_timeout' in r['safety_reasons'] for r in rows),physical_stop=abs(final_gt.v)<.1)
        else:feedback_checks['no_unexpected_stop']=all(r['safety']=='active' for r in rows)
        if profile=='timestamp_disorder':feedback_checks.update(disorder_rejected=estimator.rejections['out_of_order']>0,future_rejected=estimator.rejections['future']>0)
        if 'dropout' in profile:feedback_checks['actual_callback_drops']=estimator.rejections['dropped']>0
        physical_checks={'max_acceleration_le_3':max(acceleration)<=3+1e-5,'min_acceleration_ge_minus_6':min(acceleration)>=-6-1e-5,'max_absolute_jerk_le_6':max(map(abs,jerk))<=6}
        feedback_pass=all(feedback_checks.values());physics_pass=all(physical_checks.values())
        summary={'profile':profile,'seed':seed,'steps':steps,'simulation_seconds':steps*.05,'status':'PASS_REAL_SIM' if feedback_pass and physics_pass else ('PARTIAL' if feedback_pass else 'FAIL'),'feedback_status':'PASS_REAL_SIM' if feedback_pass else 'FAIL','physical_constraints_status':'PASS_REAL_SIM' if physics_pass else 'PARTIAL','feedback_acceptance':feedback_checks,'physical_acceptance':physical_checks,'position_rmse_m':rmse('position_error_m'),'speed_rmse_m_s':rmse('speed_error_m_s'),'yaw_rmse_rad':rmse('yaw_error_rad'),'tracking_rmse_m':rmse('offline_cte'),'max_acceleration_m_s2':max(acceleration),'min_acceleration_m_s2':min(acceleration),'max_absolute_jerk_m_s3':max(map(abs,jerk)),'final_physical_speed_m_s':final_gt.v,'displacement_m':displacement,'gnss_accepted':estimator.accepted,'gnss_rejected':estimator.rejected,'invalid_imu':estimator.invalid_imu,'transport_rejections':estimator.rejections,'real_callback_counts':estimator.raw_counts,'initialization':estimator.initialization,'noise_blueprint_attributes':estimator.noise_attributes,'fault_configuration':asdict(faults),'feedback_gt_access_attempts':guard.blocked_reads,'control_source':'GNSS_IMU_EKF including low-level acceleration feedback; no actor ground truth','evaluation_source':'actor transform/velocity AFTER command; map route is static configuration','compass_limitation':'CARLA compass stays ideal even in noisy GNSS/accelerometer/gyroscope runs; not magnetometer calibration or real hardware','physical_measurement':'Every 50ms closed-loop physical speed difference, unclipped, including terminal tick; no excluded emergency/stop samples','terminal_physical_acceleration':terminal_acc,'terminal_physical_jerk':terminal_jerk,'loop_wall_ms':dict(zip(('p50','p95','p99'),map(float,np.percentile([r['loop_wall_ms'] for r in rows],[50,95,99]))))}
        (folder/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
        return summary
    except Exception as exc:
        failure={'profile':profile,'seed':seed,'status':'FAIL','error':f'{type(exc).__name__}: {exc}','rows_completed':len(rows)}
        (folder/'failure.json').write_text(json.dumps(failure,indent=2));raise
    finally:
        (folder/'state.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows));(folder/'startup.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in startup))
        if estimator is not None:
            estimator.close();(folder/'sensor_callbacks.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in estimator.packet_log))
        for actor in reversed(actors):
            if hasattr(actor,'stop'):actor.stop()
            actor.destroy()

def main():
    import carla
    p=argparse.ArgumentParser();p.add_argument('--host',default='172.21.16.1');p.add_argument('--port',type=int,default=2000);p.add_argument('--steps',type=int,default=400);p.add_argument('--seeds',default='7,19');p.add_argument('--profiles',default=','.join(PROFILES));p.add_argument('--output',default='artifacts/carla_estimation');a=p.parse_args()
    client=carla.Client(a.host,a.port);client.set_timeout(20);world=client.get_world();original=world.get_settings();baseline_ids=set(actor.id for actor in world.get_actors());settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;settings.substepping=True;settings.max_substep_delta_time=.01;settings.max_substeps=10;world.apply_settings(settings)
    output=Path(a.output);output.mkdir(parents=True,exist_ok=True);results=[]
    try:
        routes=make_routes(world)
        for profile in a.profiles.split(','):
            if profile not in PROFILES:raise ValueError(profile)
            route=routes[min(len(routes)-1,len(routes)//2)][1] if 'curve' in profile else routes[0][1]
            for seed in map(int,a.seeds.split(',')):results.append(run_one(client,world,route,profile,seed,a.steps,output))
        report={'engine':'REAL_CARLA','client_version':client.get_client_version(),'server_version':client.get_server_version(),'map':world.get_map().name,'platform':platform.platform(),'runs':results,'feedback_pass_count':sum(r['feedback_status']=='PASS_REAL_SIM' for r in results),'full_pass_count':sum(r['status']=='PASS_REAL_SIM' for r in results),'denominator':len(results)}
        (output/'regression.json').write_text(json.dumps(report,indent=2))
    finally:
        world.apply_settings(original)
        restored=world.get_settings();remaining=set(actor.id for actor in world.get_actors())-baseline_ids
        (output/'cleanup.json').write_text(json.dumps({'remaining_new_actor_ids':sorted(remaining),'all_owned_actors_destroyed':not remaining,'original_settings':{'synchronous_mode':original.synchronous_mode,'fixed_delta_seconds':original.fixed_delta_seconds},'restored_settings':{'synchronous_mode':restored.synchronous_mode,'fixed_delta_seconds':restored.fixed_delta_seconds},'settings_restored':restored.synchronous_mode==original.synchronous_mode and restored.fixed_delta_seconds==original.fixed_delta_seconds},indent=2))
    if any(r['status']!='PASS_REAL_SIM' for r in results):raise SystemExit(1)
if __name__=='__main__':main()
