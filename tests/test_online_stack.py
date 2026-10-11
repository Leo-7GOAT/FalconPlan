import pytest
from planning.online_stack import OnlinePlanningStack,sample_profile
from vehicle_model.models import VehicleState
from planning.collision import trajectory_collides,CircularObstacle,CollisionParams
@pytest.fixture
def stack():return OnlinePlanningStack([(float(x),0) for x in range(150)])
def test_actual_initial_state_flows_dp_qp_parameterization(stack):
    p=stack.plan(VehicleState(0,0,0,0),[],0)
    assert p.speed_profile.points[0].v==pytest.approx(0,abs=1e-6)
    assert p.parameterized.speed==p.speed_profile.speeds
    assert p.metadata['qp_status']=='solved'
    assert sample_profile(p.speed_profile,.5).v>0
    with pytest.raises(ValueError):sample_profile(p.speed_profile,5)
def test_online_dynamic_projection_is_truth_driven(stack):
    actor=dict(id=19,x=28,y=0,vx=2,vy=0,timestamp=1,motion_type='moving')
    p=stack.plan(VehicleState(0,0,0,6),[actor],1)
    assert p.metadata['st_boundary_count']==1 and p.metadata['actor_ids']==[19]
    assert p.metadata['dense_st_safe']
    actor['timestamp']=0
    with pytest.raises(ValueError):stack.plan(VehicleState(0,0,0,6),[actor],1)
def test_static_geometry_obstacle_avoidance(stack):
    actor=dict(id=2,x=28,y=0,vx=0,vy=0,timestamp=0)
    p=stack.plan(VehicleState(0,0,0,2),[actor],0)
    assert p.geometry.target_d==3.5
    assert not trajectory_collides(p.geometry,[CircularObstacle(28,0,1.5)],CollisionParams(1,.2))
def test_infeasible_close_obstacle_has_explicit_failure(stack):
    with pytest.raises(RuntimeError):stack.plan(VehicleState(0,0,0,6),[dict(id=1,x=1,y=0,vx=0,vy=0,timestamp=0)],0)

def test_forward_planner_standstill_projection_and_reverse_rejection(stack):
    p=stack.plan(VehicleState(0,0,0,-.01),[],0)
    assert p.speed_profile.points[0].v==pytest.approx(0,abs=1e-6)
    with pytest.raises(ValueError):stack.plan(VehicleState(0,0,0,-1),[],0)
