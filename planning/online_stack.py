"""Online orchestration of inherited behavior, Frenet lattice and ST-DP-QP.

Inputs are stamped actor states from the adapter, never a prewritten obstacle
trajectory. CV prediction is explicit target-level ground truth, not perception.
"""
from dataclasses import dataclass
import math,bisect
from coordinate_system.reference_line import ReferenceLine
from coordinate_system.frenet_transform import FrenetTransformer
from coordinate_system.models import WorldState,FrenetState
from behavior.planner import BehaviorPlanner
from behavior.models import IDMParams,MOBILParams,LongitudinalVehicleState
from .lattice_planner import FrenetLatticePlanner,FrenetPlanningInput
from .constraints import TrajectoryConstraints
from .world_constraints import WorldTrajectoryConstraints
from .collision import CircularObstacle,CollisionParams
from .cost import TrajectoryCostWeights
from .sampling import LateralSamplingConfig
from .st_graph import DynamicObstacle,build_constant_velocity_st_boundary
from .st_grid import build_st_grid
from .st_transition import STKinematicLimits
from .st_dp import DPSpeedPlanner,DPCostWeights
from .speed_profile import speed_profile_from_dp_plan
from .speed_corridor import build_speed_corridor
from .speed_qp import SpeedQPConstraints,SpeedQPWeights,build_speed_qp_problem,solve_speed_qp
from .speed_validation import is_speed_profile_st_safe
from .time_parameterization import parameterize_geometry_with_speed
from .longitudinal_mpc import LongitudinalState,propagate_constant_jerk

def sample_profile(profile,t):
    if not 0 <= t <= profile.times[-1]+1e-7:raise ValueError('expired speed profile')
    if t>=profile.times[-1]:
        p=profile.points[-1];return LongitudinalState(p.s,p.v,p.a)
    k=max(0,bisect.bisect_right(profile.times,t)-1);p,q=profile.points[k:k+2]
    if t-p.t<1e-10:return LongitudinalState(p.s,p.v,p.a)
    return propagate_constant_jerk(LongitudinalState(p.s,p.v,p.a),(q.a-p.a)/(q.t-p.t),t-p.t)

@dataclass(frozen=True)
class OnlinePlan:
    geometry: object
    speed_profile: object
    parameterized: object
    origin_s: float
    metadata: dict

class OnlinePlanningStack:
    def __init__(self,waypoints,desired_speed=6):
        self.reference=ReferenceLine(waypoints);self.transform=FrenetTransformer(self.reference)
        self.behavior=BehaviorPlanner(IDMParams(desired_speed,3,2,4,1.5),MOBILParams(.2,4,.1))
        self.lattice=FrenetLatticePlanner(self.reference,TrajectoryConstraints(12,3,6,2.5,8),WorldTrajectoryConstraints(.18),CollisionParams(1,.2),TrajectoryCostWeights(),LateralSamplingConfig((0,)),dt=.1)
        self.desired_speed=desired_speed
    def project(self,state):return self.transform.world_to_frenet(WorldState(state.x,state.y,state.psi))
    def plan(self,state,actors,now,acceleration=0):
        if state.v<-.1:raise ValueError('reverse is outside the forward-planning ODD')
        # Small signed rollback while the simulator engages first gear is
        # retained in truth logs; the forward-only optimizer models standstill.
        planning_speed=max(0,state.v)
        if any(now-o['timestamp']>.2 or o['timestamp']>now+1e-6 for o in actors):raise ValueError('stale actor state')
        ego=self.project(state);static=[];moving=[];fronts=[];static_ahead=False
        for actor in actors:
            pose=self.transform.world_to_frenet(WorldState(actor['x'],actor['y'],state.psi));speed=actor['vx']*math.cos(state.psi)+actor['vy']*math.sin(state.psi)
            if abs(pose.d-ego.d)<2 and pose.s>ego.s:fronts.append((pose.s-ego.s,pose,speed,actor))
            if abs(speed)<.5 and actor.get('motion_type')!='moving':
                static.append(CircularObstacle(actor['x'],actor['y'],1.5))
                static_ahead |= pose.s+4>ego.s and abs(pose.d)<2
            else:moving.append((pose,speed,actor))
        front=min(fronts,key=lambda x:x[0]) if fronts else None
        b=self.behavior.step(LongitudinalVehicleState(ego.s,planning_speed),LongitudinalVehicleState(front[1].s,front[2]) if front else None,left_lane_exists=bool(static))
        desired_d=3.5 if static_ahead else 0
        # Geometry alone uses a small speed floor to parameterize d(s) at rest.
        # ST-QP and MPC retain the measured initial speed, including exactly zero.
        geometry=self.lattice.plan(FrenetPlanningInput(ego.s,max(2,state.v),0,ego.d,ego.d_prime*state.v,0,[0,3.5] if static else [0],[self.desired_speed],[4,5,6,8],desired_d,self.desired_speed,static)).best
        # Extend the selected collision-checked maneuver along its terminal lane
        # to provide sufficient geometry for the independent ST speed horizon.
        while geometry.s[-1] < min(ego.s+34,self.reference.length-.01):
            tail_s=min(geometry.s[-1]+.5,self.reference.length-.01)
            pose=self.transform.frenet_to_world(FrenetState(tail_s,geometry.target_d,0))
            geometry.s.append(tail_s);geometry.x.append(pose.x);geometry.y.append(pose.y);geometry.yaw.append(pose.theta);geometry.curvature.append(self.reference.query(tail_s).kappa)
        from .collision import trajectory_collides
        if trajectory_collides(geometry,static,self.lattice.collision_params):raise RuntimeError('extended geometry rejected collision check')
        boundaries=[]
        for pose,speed,actor in moving:
            if abs(pose.d-geometry.target_d)<2 and pose.s>ego.s:
                boundaries.append(build_constant_velocity_st_boundary(DynamicObstacle(str(actor['id']),pose.s-ego.s,max(0,speed),4,3),4,.1))
        grid=build_st_grid(4,1,32,.5)
        dp=DPSpeedPlanner(grid,boundaries,STKinematicLimits(0,12,3,6),planning_speed,self.desired_speed,DPCostWeights(1,.5),edge_sample_dt=.05).plan()
        coarse=speed_profile_from_dp_plan(dp,planning_speed,float(max(-6,min(3,acceleration))))
        corridor=build_speed_corridor(reference_profile=coarse,boundaries=boundaries,min_s=0,max_s=32,boundary_epsilon=.3)
        qp=solve_speed_qp(problem=build_speed_qp_problem(reference_profile=coarse,corridor=corridor,constraints=SpeedQPConstraints(0,12,3,6,6),weights=SpeedQPWeights(1,2,.3,3)),reference_profile=coarse).profile
        if not is_speed_profile_st_safe(qp,boundaries,.005):raise RuntimeError('dense ST check rejected online QP')
        parameterized=parameterize_geometry_with_speed(geometry,qp)
        return OnlinePlan(geometry,qp,parameterized,ego.s,dict(behavior=b.state.value,risk=b.risk.name,target_d=geometry.target_d,actor_ids=[o['id'] for o in actors],st_boundary_count=len(boundaries),dp_nodes=len(dp.nodes),qp_status='solved',dense_st_safe=True,geometry_speed_floor_m_s=2))
