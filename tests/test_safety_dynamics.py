import math
import pytest
import numpy as np
from control.safety import SafetySupervisor,SafetyConfig,SafetyMode
from control.first_order_actuator import FirstOrderActuator,ActuatorConfig
from vehicle_model.dynamic_bicycle import DynamicBicycleModel,DynamicState,DynamicsParams
from vehicle_model.models import VehicleState,VehicleCommand

@pytest.mark.parametrize('fault', ['state','command','future','estop','collision','geofence','speed'])
def test_safety_fault_latches_and_explicit_reset(fault):
    supervisor=SafetySupervisor(SafetyConfig(geofence=(-10,10,-10,10)))
    state=VehicleState(11 if fault=='geofence' else 0,0,0,21 if fault=='speed' else 2)
    out=supervisor.evaluate(state,VehicleCommand(.2,1),now=1,state_time=0 if fault=='state' else (2 if fault=='future' else 1),command_time=0 if fault=='command' else 1,dt=.05,emergency_stop=fault=='estop',collision=fault=='collision')
    assert out.mode==SafetyMode.STOPPED and out.command.a == -6
    assert supervisor.evaluate(VehicleState(0,0,0,0),VehicleCommand(0,1),now=2,state_time=2,command_time=2,dt=.05).mode==SafetyMode.STOPPED
    with pytest.raises(ValueError):supervisor.reset(state,explicit=True)
    supervisor.reset(VehicleState(0,0,0,0),explicit=True)
    assert supervisor.mode==SafetyMode.ACTIVE

def test_manual_constraints_and_timestamps():
    s=SafetySupervisor()
    out=s.evaluate(VehicleState(0,0,0,0),VehicleCommand(0,0),now=0,state_time=0,command_time=0,dt=.1,manual=VehicleCommand(.5,9))
    assert out.command.delta==pytest.approx(.08) and out.command.a==3 and out.mode==SafetyMode.MANUAL
    with pytest.raises(ValueError):s.evaluate(VehicleState(0,0,0,0),VehicleCommand(0,0),now=0,state_time=0,command_time=0,dt=float('nan'))

def test_actuator_analytical_response_delay_deadzone_rate():
    a=FirstOrderActuator(ActuatorConfig(-10,10,100,time_constant=.2))
    for _ in range(10): value=a.update(1,.02)
    assert value==pytest.approx(1-math.exp(-1))
    b=FirstOrderActuator(ActuatorConfig(-1,1,.5,delay=.1,deadzone=.05))
    assert b.update(1,.05)==0
    assert b.update(1,.05)==0
    assert b.update(1,.05)==pytest.approx(.025)
    with pytest.raises(ValueError):b.update(float('inf'),.05)

def test_dynamic_straight_numerical_and_stability():
    m=DynamicBicycleModel();q=DynamicState(v=10)
    for _ in range(100):q=m.step(q,VehicleCommand(0,0),.01)
    assert q.x==pytest.approx(10,abs=1e-10) and q.y==0 and q.yaw_rate==0
    for speed in [2,5,10,20,30]:assert np.max(m.lateral_eigenvalues(speed).real)<0
    q=DynamicState(v=10)
    for _ in range(100):q=m.step(q,VehicleCommand(.03,0),.01)
    assert q.y > 0 and q.yaw_rate > 0 and abs(q.lateral_v)<2

@pytest.mark.parametrize('bad', [0,-.01,float('nan'),.2])
def test_dynamic_dt_boundaries(bad):
    with pytest.raises(ValueError):DynamicBicycleModel().step(DynamicState(),VehicleCommand(0,0),bad)

def test_friction_limit_and_params():
    with pytest.raises(ValueError):DynamicsParams(mass=0)
    m=DynamicBicycleModel(DynamicsParams(friction=.2));q=DynamicState(v=25)
    for _ in range(100):q=m.step(q,VehicleCommand(.5,0),.01)
    assert all(math.isfinite(v) for v in vars(q).values())
