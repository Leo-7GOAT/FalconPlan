"""Two-lane analytical dynamics experiment. This is NOT CARLA or physical I/O.

Learner chooses acceleration and an absolute target lane; inherited Stanley
tracks the lane center. All policies use identical actuator/safety contracts.
Traffic is target-level state truth, with scripted leaders and IDM followers.
"""
from dataclasses import dataclass,asdict
import math
import time
import numpy as np
from numbers import Integral
from vehicle_model.dynamic_bicycle import DynamicBicycleModel,DynamicState,DynamicsParams
from vehicle_model.models import VehicleCommand
from control.first_order_actuator import ActuatorConfig,FirstOrderActuator
from control.safety import SafetyConfig,SafetySupervisor,SafetyMode
from control.stanley import StanleyController
from behavior.models import IDMParams
from behavior.idm import compute_idm_acceleration
from planning.polynomial import QuinticPolynomial

ENGINE='ANALYTICAL_DYNAMIC_BICYCLE_NOT_CARLA'
OBSERVATION_SIZE=17

@dataclass(frozen=True)
class EpisodeConfig:
    dt: float=.05
    steps: int=400
    desired_speed: float=8.
    friction: float=.9
    delay: float=.05
    mass_scale: float=1.
    acceleration_gain: float=1.
    reward_version: int=2
    actuator_envelope_version: int=3
    def __post_init__(self):
        if any(isinstance(v,bool) or not isinstance(v,Integral) for v in (self.steps,self.reward_version,self.actuator_envelope_version)):raise ValueError('integer steps and versions required')
        if not all(math.isfinite(v) for v in asdict(self).values()):raise ValueError('finite configuration required')
        if not 0<self.dt<=.1 or self.steps<=0 or int(self.steps)!=self.steps or self.desired_speed<=0 or self.friction<=0 or self.delay<0 or self.mass_scale<=0 or self.acceleration_gain<=0:raise ValueError('invalid episode configuration')
        if self.reward_version not in (1,2) or self.actuator_envelope_version not in (1,2,3):raise ValueError('unsupported experiment version')

@dataclass
class Traffic:
    x: float
    v: float
    lane: int
    target: float
    follower: bool=False

class DrivingEnv:
    def __init__(self,config=EpisodeConfig(),*,shield=True):
        self.config=config;self.shield=shield;self.finished=True
    def reset(self,seed=0,domain_randomization=False,scenario='mixed'):
        self.rng=np.random.default_rng(seed)
        base=asdict(self.config)
        if domain_randomization:
            base.update(desired_speed=float(self.rng.uniform(6,10)),friction=float(self.rng.uniform(.8,1)),delay=float(self.rng.uniform(0,.1)),mass_scale=float(self.rng.uniform(.9,1.1)))
        self.episode=EpisodeConfig(**base);p=self.episode
        self.model=DynamicBicycleModel(DynamicsParams(mass=1500*p.mass_scale,inertia=2500*p.mass_scale,friction=p.friction))
        self.steer=FirstOrderActuator(ActuatorConfig(-.5,.5,.8,.12,p.delay,.003))
        self.accel=FirstOrderActuator(ActuatorConfig(-6,3,6,.15,p.delay,.02))
        self.safety=SafetySupervisor(SafetyConfig(max_speed=12,geofence=(-10,500,-1.5,5.)))
        self.stanley=StanleyController(.6,1,.5)
        lane=int(self.rng.integers(2));self.target_lane=lane
        self.state=DynamicState(y=lane*3.5+float(self.rng.uniform(-.1,.1)),v=float(self.rng.uniform(1,5)))
        lead_v=float(self.rng.uniform(2,6));lead_gap=float(self.rng.uniform(22,45))
        if scenario=='braking':lead_gap=35.;lead_v=6.
        if scenario=='free':lead_gap=120.;lead_v=p.desired_speed
        self.traffic=[Traffic(lead_gap,lead_v,lane,lead_v),Traffic(float(self.rng.uniform(35,90)),float(self.rng.uniform(6,10)),1-lane,float(self.rng.uniform(6,10))),Traffic(-float(self.rng.uniform(20,45)),float(self.rng.uniform(5,9)),1-lane,8.,True)]
        self.scenario=scenario;self.brake_at=float(self.rng.uniform(4,8));self.tick=0;self.last_accel_command=0.;self.last_lane_change=-100.;self.return_sum=0.;self.rows=[];self.finished=False;self.lane_curve=None;self.lane_origin=0.
        return self.observation()
    def neighbors(self,lane):
        front=[a for a in self.traffic if a.lane==lane and a.x>self.state.x]
        rear=[a for a in self.traffic if a.lane==lane and a.x<=self.state.x]
        return min(front,key=lambda a:a.x,default=None),max(rear,key=lambda a:a.x,default=None)
    @property
    def current_lane(self):return int(np.clip(round(self.state.y/3.5),0,1))
    def observation(self):
        s=self.state
        result=[s.v/12,s.y/3.5,math.sin(s.psi),s.yaw_rate/.5,self.accel.value/6,self.steer.value/.5,self.episode.desired_speed/12,float(self.target_lane)]
        for lane in (0,1):
            front,rear=self.neighbors(lane)
            result.extend([(front.x-s.x-4.5)/40 if front else 2.,(front.v-s.v)/12 if front else 0.])
        for lane in (0,1):
            front,rear=self.neighbors(lane)
            result.extend([(s.x-rear.x-4.5)/40 if rear else 2.,(rear.v-s.v)/12 if rear else 0.])
        result.append(self.last_accel_command/6)
        return np.clip(np.asarray(result,dtype=np.float32),-5,5)
    def lane_safe(self,lane):
        s=self.state;front,rear=self.neighbors(lane)
        if front and front.x-s.x-4.5<max(8,s.v*.8+max(0,s.v-front.v)*2):return False
        if rear and s.x-rear.x-4.5<max(8,rear.v*.8+max(0,rear.v-s.v)*2):return False
        return True
    def reference(self,x):
        if self.lane_curve is None:return self.target_lane*3.5,0.
        progress=float(np.clip(x-self.lane_origin,0,25))
        return self.lane_curve.position(progress),math.atan(self.lane_curve.velocity(progress))
    def _traffic_step(self):
        p=self.episode;t=self.tick*p.dt
        for actor in self.traffic:
            a=float(np.clip((actor.target-actor.v)*.8,-3,2))
            if actor is self.traffic[0] and self.scenario!='free' and self.brake_at<=t<self.brake_at+2.:a=-3.
            if actor.follower:
                fronts=[q for q in self.traffic if q is not actor and q.lane==actor.lane and q.x>actor.x]
                front=min(fronts,key=lambda q:q.x,default=None)
                if abs(self.state.y-actor.lane*3.5)<1.8 and self.state.x>actor.x and (front is None or self.state.x<front.x):
                    gap=self.state.x-actor.x-4.5;speed=self.state.v
                else:gap=front.x-actor.x-4.5 if front else None;speed=front.v if front else None
                a=float(np.clip(compute_idm_acceleration(actor.v,max(.01,gap) if gap is not None else None,speed,IDMParams(actor.target,2,2,4,1.5)),-6,2))
            previous=actor.v;actor.v=max(0,actor.v+a*p.dt);actor.x+=(previous+actor.v)*.5*p.dt
    def step(self,acceleration,lane,*,fault=None):
        if self.finished:raise RuntimeError('reset required after terminal episode')
        if isinstance(acceleration,bool) or not math.isfinite(acceleration) or isinstance(lane,bool) or not isinstance(lane,Integral) or lane not in (0,1):raise ValueError('finite acceleration and integer lane 0/1 required')
        started=time.perf_counter_ns();p=self.episode;s=self.state;now=self.tick*p.dt;requested=float(acceleration);lane_denied=False;shielded=False
        if lane!=self.target_lane:
            settled=abs(s.y-3.5*self.target_lane)<.3 and abs(s.psi)<.1
            if now-self.last_lane_change<3 or not settled or self.shield and not self.lane_safe(lane):lane_denied=True
            else:
                self.target_lane=lane;self.last_lane_change=now;self.lane_origin=s.x
                self.lane_curve=QuinticPolynomial(s.y,math.tan(s.psi),0,lane*3.5,0,0,25)
        if self.shield:
            # Predictive guard remains visible in metrics; it never replaces
            # policy training/log probabilities or hides raw requested action.
            relevant=[a for a in self.traffic if a.x>s.x and (abs(s.y-a.lane*3.5)<1.8 or a.lane==self.target_lane)]
            front=min(relevant,key=lambda a:a.x,default=None)
            if front:
                gap=front.x-s.x-4.5;closing=s.v-front.v
                if gap<4+1.2*s.v or closing>0 and gap/max(closing,1e-6)<3:
                    cap=compute_idm_acceleration(s.v,max(.01,gap),front.v,IDMParams(p.desired_speed,3,2,4,1.5))
                    acceleration=min(acceleration,cap);shielded=acceleration<requested
        # Same nominal acceleration/jerk bounds for learned and rule baseline.
        acceleration=float(np.clip(acceleration,-6,3))
        acceleration=float(np.clip(acceleration,self.last_accel_command-6*p.dt,self.last_accel_command+6*p.dt));self.last_accel_command=acceleration
        front_y,front_heading=self.reference(s.x+2.8*math.cos(s.psi))
        steering=self.stanley.update(front_heading-s.psi,front_y-(s.y+2.8*math.sin(s.psi)),s.v)
        command=VehicleCommand(steering,acceleration)
        safe=self.safety.evaluate(s.vehicle_state(),command,now=now,state_time=now-.5 if fault=='state_timeout' else now,command_time=now-.5 if fault=='command_timeout' else now,dt=p.dt,emergency_stop=fault=='estop')
        actuator_accel=self.accel.update(safe.command.a,p.dt)
        # Discrete jerk-limited brake release before the nonnegative-speed
        # contact boundary. Without this, v=0 clipping creates an impulse even
        # with a smooth actuator. This plant-aware analytical safety filter is
        # shared by every policy; unknown longitudinal gain remains an OOD test.
        brake_limit=math.sqrt((6*p.dt)**2+12*s.v)-6*p.dt
        actuator_accel=max(actuator_accel,-brake_limit)
        if p.actuator_envelope_version>=2:
            # Control target is strictly inside the unchanged hard bound.
            # v2 is retained to reproduce a floating-point overspeed latch.
            control_ceiling=self.safety.config.max_speed-(.001 if p.actuator_envelope_version==3 else 0)
            headroom=max(0,control_ceiling-s.v)
            acceleration_limit=math.sqrt((6*p.dt)**2+12*headroom)-6*p.dt
            actuator_accel=min(actuator_accel,acceleration_limit)
        self.accel.value=actuator_accel
        physical=VehicleCommand(self.steer.update(safe.command.delta,p.dt),actuator_accel*p.acceleration_gain)
        before=s;before_actors=[(a.x,a.lane*3.5) for a in self.traffic]
        self.state=self.model.step(s,physical,p.dt);self._traffic_step();s=self.state
        # Swept relative rectangles avoid tunnelling at a coarse policy period.
        collision=False
        for actor,(x0,y0) in zip(self.traffic,before_actors):
            for alpha in np.linspace(0,1,6):
                dx=(1-alpha)*(before.x-x0)+alpha*(s.x-actor.x)
                dy=(1-alpha)*(before.y-y0)+alpha*(s.y-actor.lane*3.5)
                collision |= abs(dx)<4.5 and abs(dy)<1.8
        offroad=not -1.5<=s.y<=5
        gaps=[a.x-s.x-4.5 for a in self.traffic if a.x>s.x and abs(s.y-a.lane*3.5)<1.8]
        gap=min(gaps,default=100.)
        a=(s.v-before.v)/p.dt;previous_a=self.rows[-1]['acceleration'] if self.rows else 0.;jerk=(a-previous_a)/p.dt
        ref_y,ref_heading=self.reference(s.x)
        progress=s.v/p.desired_speed
        if p.reward_version==2:progress=min(progress,1.)
        reward=.5*progress-.8*((s.v-p.desired_speed)/p.desired_speed)**2-.03*(s.y-ref_y)**2-.02*(a/3)**2-.02*(jerk/6)**2-.03*lane_denied-.02*shielded
        if p.reward_version==2 and safe.mode==SafetyMode.STOPPED:reward-=5
        if collision or offroad:reward-=100
        self.tick+=1;self.return_sum+=reward
        self.finished=bool(collision or offroad or self.tick>=p.steps or safe.mode==SafetyMode.STOPPED and s.v<.1)
        row=dict(t=self.tick*p.dt,x=s.x,y=s.y,psi=s.psi,v=s.v,target_speed=p.desired_speed,cte=s.y-ref_y,heading_error=s.psi-ref_heading,lane=self.current_lane,target_lane=self.target_lane,requested_lane=int(lane),requested_acceleration=requested,command_acceleration=safe.command.a,acceleration=a,jerk=jerk,steering=physical.delta,gap=gap,collision=bool(collision),offroad=offroad,shielded=bool(shielded),lane_denied=bool(lane_denied),safety=safe.mode.value,safety_reasons=safe.reasons,reward=reward,runtime_us=(time.perf_counter_ns()-started)/1000)
        self.rows.append(row)
        return self.observation(),float(reward),self.finished,row
    def metrics(self):
        if not self.rows:raise RuntimeError('no executed samples')
        q=self.rows;rmse=lambda k:float(np.sqrt(np.mean([r[k]**2 for r in q])))
        result=dict(engine=ENGINE,samples=len(q),return_sum=self.return_sum,displacement_m=q[-1]['x'],cte_rmse_m=rmse('cte'),heading_rmse_rad=rmse('heading_error'),speed_rmse_m_s=float(np.sqrt(np.mean([(r['v']-r['target_speed'])**2 for r in q]))),max_steering_rad=max(abs(r['steering']) for r in q),max_steering_rate_rad_s=max([abs(b['steering']-a['steering'])/self.episode.dt for a,b in zip(q,q[1:])],default=0),max_acceleration_m_s2=max(r['acceleration'] for r in q),min_acceleration_m_s2=min(r['acceleration'] for r in q),max_jerk_m_s3=max(abs(r['jerk']) for r in q),collision_count=sum(r['collision'] for r in q),offroad_count=sum(r['offroad'] for r in q),shield_interventions=sum(r['shielded'] or r['lane_denied'] for r in q),supervisor_interventions=self.safety.interventions,lane_changes=sum(a['target_lane']!=b['target_lane'] for a,b in zip(q,q[1:])),controller_model_runtime_us=dict(zip(['p50','p95','p99'],map(float,np.percentile([r['runtime_us'] for r in q],[50,95,99])))),episode_config=asdict(self.episode))
        passed=(len(q)==self.episode.steps and result['displacement_m']>3 and result['cte_rmse_m']<1 and result['collision_count']==0 and result['offroad_count']==0 and result['max_steering_rad']<=.5+1e-6 and result['max_steering_rate_rad_s']<=.8+1e-6 and result['max_acceleration_m_s2']<=3+1e-6 and result['min_acceleration_m_s2']>=-6-1e-6 and result['max_jerk_m_s3']<=6+1e-6)
        result['status']='PASS_MODEL_EXPERIMENT' if passed else 'FAIL'
        return result
