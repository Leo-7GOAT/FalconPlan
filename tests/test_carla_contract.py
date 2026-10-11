"""Adapter contract test using API-shaped test doubles; not REAL simulation evidence."""
from types import SimpleNamespace as S
import math
import pytest
from hal.carla_adapter import CarlaAdapter
class Vehicle:
    def get_physics_control(self):return S(wheels=[S(position=S(x=1000,y=2140),max_steer_angle=30),S(position=S(x=1000,y=1860),max_steer_angle=0)])
    def get_transform(self):return S(location=S(x=10,y=20),rotation=S(yaw=90))
    def get_velocity(self):return S(x=0,y=3)
class World:
    def get_snapshot(self):return S(timestamp=S(elapsed_seconds=2))
def test_handedness_rear_axle_units_and_fixed_tick():
    a=CarlaAdapter(None,World(),Vehicle())
    s=a.get_state()
    assert s.x==pytest.approx(10) and s.y==pytest.approx(-18.6)
    assert s.psi==pytest.approx(-math.pi/2) and s.v==3 and a.wheelbase==pytest.approx(2.8)
    with pytest.raises(ValueError):a.step(.01)

def test_estimated_feedback_control_has_no_actor_truth_reads(monkeypatch):
    import sys
    from vehicle_model.models import VehicleState,VehicleCommand
    vehicle=Vehicle();vehicle.apply_control=lambda command:None
    a=CarlaAdapter(None,World(),vehicle)
    source=S(state=VehicleState(1,2,.3,4),timestamp=1.5,acceleration=.2)
    a.use_estimated_feedback(source)
    def forbidden():raise AssertionError('GT leakage into estimated control')
    vehicle.get_transform=forbidden;vehicle.get_velocity=forbidden;vehicle.get_acceleration=forbidden
    monkeypatch.setitem(sys.modules,'carla',S(VehicleControl=lambda **kwargs:S(**kwargs)))
    assert a.get_state()==source.state and a.timestamp==1.5
    a.apply_command(VehicleCommand(.1,1))
    assert a.throttle>0
