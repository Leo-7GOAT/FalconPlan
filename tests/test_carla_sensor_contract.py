"""API boundary unit tests; these test doubles are not CARLA simulation evidence."""
from types import SimpleNamespace as S
import sys
import numpy as np
import pytest
from hal.carla_sensors import CarlaSensorEstimator, SensorFaults
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
def make(monkeypatch,**kwargs):
    monkeypatch.setitem(sys.modules,'carla',S(Location=lambda x=0,y=0:S(x=x,y=y),Transform=lambda:S()))
    def forbidden():raise AssertionError('actor GT must not initialize sensor filter')
    return CarlaSensorEstimator(World(),None,S(get_state=forbidden,rear_offset=2),**kwargs)
def emit(e,stamp=0,frame=1,acc=0,gyro=0,longitude=2e-5):
    e.sensors[1].callback(S(timestamp=stamp,frame=frame,accelerometer=S(x=acc),gyroscope=S(z=gyro),compass=np.pi/2))
    e.sensors[0].callback(S(timestamp=stamp,frame=frame,longitude=longitude,latitude=0))
def test_callback_only_initialization_rejects_spawn_imu_and_projects_map(monkeypatch):
    e=make(monkeypatch);emit(e,acc=1e9)
    assert e.update(now=0) is None and e.invalid_imu==1
    emit(e,.05,2);state=e.update(now=.05)
    assert e.ready and state.x==pytest.approx(0,abs=1e-10) and state.v==0
    assert e.initialization['source']=='REAL_GNSS_IMU_COMPASS_STATIONARY_HOLD'
    e.close()
def test_measurement_time_and_duplicate_do_not_double_integrate(monkeypatch):
    e=make(monkeypatch);emit(e);e.update(now=0)
    e.sensors[1].callback(S(timestamp=.05,frame=2,accelerometer=S(x=1),gyroscope=S(z=-.1),compass=np.pi/2))
    state=e.update(now=.05);assert state.v==pytest.approx(.05)
    snapshot=e.filter.mean.copy();e.update(.05,now=.05)
    assert np.array_equal(e.filter.mean,snapshot)
    emit(e,0,1);e.update(now=.05)
    assert e.rejections['out_of_order']==2 and np.array_equal(e.filter.mean,snapshot)
def test_future_and_transport_disorder_never_poison_valid_timestamps(monkeypatch):
    e=make(monkeypatch,faults=SensorFaults(reorder_every=2,future_every=2))
    emit(e);e.update(now=0);emit(e,.05,2);e.update(now=.05)
    assert e.rejections['out_of_order']==2 and e.rejections['future']==2
    emit(e,.1,3);e.update(now=.1)
    assert e.timestamp==pytest.approx(.1) and e.accepted==3
def test_callback_dropout_preserves_old_freshness_for_safety(monkeypatch):
    e=make(monkeypatch,faults=SensorFaults(dropout_windows=((.05,.5),)))
    emit(e);e.update(now=0);emit(e,.1,3);e.update(now=.1,frame=3)
    assert e.timestamp==0 and e.rejections['dropped']==2
    from control.safety import SafetySupervisor
    from vehicle_model.models import VehicleCommand
    assert SafetySupervisor().evaluate(e.state,VehicleCommand(0,1),now=.3,state_time=e.timestamp,command_time=.3,dt=.05).mode.value=='stopped'
def test_nonfinite_packet_rejected_and_stationary_contract_required(monkeypatch):
    e=make(monkeypatch);emit(e,longitude=float('nan'));assert e.update(now=0) is None
    assert e.rejections['nonfinite']==1
    with pytest.raises(ValueError):make(monkeypatch,stationary_start=False)

def test_stationary_speed_prior_prevents_noise_false_reverse_stop(monkeypatch):
    e=make(monkeypatch);emit(e);e.update(now=0);rng=np.random.default_rng(7)
    from control.safety import SafetySupervisor
    from vehicle_model.models import VehicleCommand
    safety=SafetySupervisor()
    for frame in range(2,42):
        stamp=(frame-1)*.05;x=.25*stamp*stamp+rng.normal(0,.1)
        emit(e,stamp,frame,acc=.5,longitude=(2+x)*1e-5);e.update(now=stamp)
        result=safety.evaluate(e.state,VehicleCommand(0,.5),now=stamp,state_time=e.timestamp,command_time=stamp,dt=.05)
        assert result.mode.value=='active'
    assert abs(e.state.v-1)<.3
