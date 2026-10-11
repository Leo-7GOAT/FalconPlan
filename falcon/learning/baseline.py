"""Actual inherited FSM/IDM/MOBIL decisions, under the same environment gate."""
from behavior.planner import BehaviorPlanner
from behavior.models import IDMParams,MOBILParams,LongitudinalVehicleState
class ClassicalBaseline:
    def __init__(self,desired_speed):
        self.planner=BehaviorPlanner(IDMParams(desired_speed,3,2,4,1.5),MOBILParams(.2,4,.1))
        self.mobil_evaluations=0
    def act(self,env):
        s=env.state;lane=env.current_lane
        state=lambda a:LongitudinalVehicleState(a.x-4.5 if a.x>s.x else a.x+4.5,a.v) if a else None
        front,rear=env.neighbors(lane);other_front,other_rear=env.neighbors(1-lane)
        # Lane impact equations need strictly ordered states even at a close
        # contact boundary; the experiment terminates collisions separately.
        current_front=state(front);current_rear=state(rear);new_front=state(other_front);new_rear=state(other_rear)
        for value,is_front in ((current_front,True),(new_front,True),(current_rear,False),(new_rear,False)):
            if value and (value.s<=s.x if is_front else value.s>=s.x):
                return -6,env.target_lane
        kwargs=dict(ego=LongitudinalVehicleState(s.x,s.v),front_current_lane=current_front,rear_current_lane=current_rear,lane_change_completed=abs(s.y-env.target_lane*3.5)<.3 and abs(s.psi)<.1)
        if lane==0:kwargs.update(left_lane_exists=True,front_left_lane=new_front,rear_left_lane=new_rear)
        else:kwargs.update(right_lane_exists=True,front_right_lane=new_front,rear_right_lane=new_rear)
        result=self.planner.step(**kwargs)
        self.mobil_evaluations+=int(result.left_eval is not None or result.right_eval is not None)
        target=env.target_lane
        if result.state.value=='lane_change_left':target=1
        if result.state.value=='lane_change_right':target=0
        return (-6 if result.state.value=='emergency_brake' else result.idm_acceleration),target
