"""API-shaped sensor boundary tests. This is not REAL CARLA evidence."""
from types import SimpleNamespace as S
import sys
import pytest
from hal.carla_sensors import CarlaSensorEstimator
from vehicle_model.models import VehicleState
class Blueprint:
    def set_attribute(self,*args):pass
class Sensor:
    def listen(self,callback):self.callback=callback
    def stop(self):pass
    def destroy(self):pass
class World:
    def get_blueprint_library(self):return S(find=lambda name:Blueprint())
    def spawn_actor(self,*args,**kwargs):return Sensor()
    def get_map(self):return S(transform_to_geolocation=lambda location:S(longitude=location.x*1e-5,latitude=location.y*1e-5))
def test_real_sensor_boundary_rejects_spawn_derivative_and_projects_georeference(monkeypatch):
    monkeypatch.setitem(sys.modules,'carla',S(Location=lambda x=0,y=0:S(x=x,y=y),Transform=lambda:S()))
    adapter=S(get_state=lambda:VehicleState(0,0,0,0),rear_offset=2)
    e=CarlaSensorEstimator(World(),None,adapter)
    e.imu=S(accelerometer=S(x=1e9),gyroscope=S(z=0))
    e.gnss=S(longitude=2e-5,latitude=0,timestamp=0)
    state=e.update(.05)
    assert e.invalid_imu==1 and e.accepted==1 and state.x==pytest.approx(0,abs=1e-10)
    e.imu=S(accelerometer=S(x=1),gyroscope=S(z=-.1));e.gnss=None
    state=e.update(.05)
    assert state.v==pytest.approx(.05) and state.psi==pytest.approx(.005)
    e.close()
