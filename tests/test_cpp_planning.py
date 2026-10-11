"""Native integration tests; build library and set FALCON_PLANNING_CPP_LIBRARY."""
import os
from pathlib import Path
import pytest
from control.cpp_core import CppCore
from planning.collision import CircularObstacle,CollisionParams
from planning.cost import TrajectoryCostWeights
from planning.trajectory import FrenetTrajectory
from planning.online_stack import OnlinePlanningStack
from vehicle_model.models import VehicleState
from tools.cpp_planning_benchmark import verify

@pytest.fixture
def native():
    root=Path(__file__).resolve().parents[1]
    name='falcon_c.dll' if os.name=='nt' else 'libfalcon_c.so'
    defaults=[root/'build'/'cpp'/name,root/'cpp'/'build'/name]
    library=os.environ.get('FALCON_PLANNING_CPP_LIBRARY',str(next((p for p in defaults if p.is_file()),defaults[0])))
    if not Path(library).is_file():
        if 'FALCON_PLANNING_CPP_LIBRARY' in os.environ:pytest.fail('explicit native library path does not exist')
        pytest.skip('native planning library not built on this platform')
    return CppCore(library)

def candidate(y=0):
    return FrenetTrajectory(t=[0,1],x=[0,10],y=[y,y],d_ddd=[2,4],s_ddd=[3,5],target_d=0,target_speed=6,duration=4)

def test_native_sparse_segment_tangency_and_tie(native):
    args=([candidate(),candidate(4)],[CircularObstacle(5,2,1)],CollisionParams(1),TrajectoryCostWeights(),0,6)
    winner,costs,blocked=native.rank_candidates(*args)
    assert winner is None and blocked==[True,True]
    args=list(args);args[1]=[CircularObstacle(5,0,1)]
    assert native.rank_candidates(*args)[0]==1
    args[1]=[]
    assert native.rank_candidates(*args)[0]==0
    assert native.rank_candidates(*args)[1][0].total==pytest.approx(3.35)

@pytest.mark.parametrize('fault',['nan','length','radius','weight','duration','empty'])
def test_native_invalid_data_fails(native,fault):
    args=[[candidate()],[],CollisionParams(1),TrajectoryCostWeights(),0,6]
    if fault=='nan':args[0][0].d_ddd[0]=float('nan')
    if fault=='length':args[0][0].x=[]
    if fault=='radius':args[1]=[CircularObstacle(0,0,-1)]
    if fault=='weight':args[3]=TrajectoryCostWeights(lateral_offset=float('inf'))
    if fault=='duration':args[0][0].duration=0
    if fault=='empty':args[0]=[]
    with pytest.raises(ValueError):native.rank_candidates(*args)

def test_native_online_planner_actual_selection(native,monkeypatch):
    monkeypatch.delenv('FALCON_PLANNING_CPP_LIBRARY',raising=False)
    waypoints=[(float(x),0.) for x in range(121)]
    python=OnlinePlanningStack(waypoints)
    cpp=OnlinePlanningStack(waypoints,cpp_core=native)
    actor=dict(id=9,x=28,y=0,vx=0,vy=0,timestamp=0)
    # Compare inherited Python route with actual C++ evaluator selected route.
    a=python.plan(VehicleState(0,0,0,2),[actor],0)
    b=cpp.plan(VehicleState(0,0,0,2),[actor],0)
    assert b.metadata['candidate_evaluator']=='C++17_IN_PROCESS'
    assert b.metadata['collision_free']>0 and b.geometry.target_d==3.5
    assert b.geometry.x==pytest.approx(a.geometry.x,abs=1e-10)
    assert b.geometry.y==pytest.approx(a.geometry.y,abs=1e-10)
    assert b.geometry.cost==pytest.approx(a.geometry.cost,abs=1e-10)
    with pytest.raises(RuntimeError):cpp.plan(VehicleState(0,0,0,6),[dict(actor,x=1)],0)

def test_native_seeded_parity_small(native,tmp_path):
    # Full 2,000-candidate study is reproducible via the standalone tool.
    result=verify(native.lib._name,tmp_path/'parity.json',batches=12)
    assert result['max_absolute_cost_error']<1e-9
