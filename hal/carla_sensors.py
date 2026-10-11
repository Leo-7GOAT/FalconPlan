"""Real CARLA GNSS/IMU feedback. Actor pose is never used to initialize this filter.

OpenDRIVE georeference and static axle extrinsics are configuration. The caller
must hold the vehicle stationary for sensor-only GNSS/compass initialization.
"""
from dataclasses import dataclass
import math
import threading
import time
import numpy as np
from vehicle_model.models import VehicleState
from vehicle_model.state_estimator import PlanarEKF

@dataclass(frozen=True)
class SensorFaults:
    dropout_windows: tuple = ()
    reorder_every: int = 0
    future_every: int = 0

@dataclass(frozen=True)
class SensorPacket:
    kind: str
    timestamp: float
    frame: int
    values: tuple

class CarlaSensorEstimator:
    def __init__(self,world,vehicle,adapter,*,noise=False,seed=7,faults=SensorFaults(),stationary_start=True):
        import carla
        if not stationary_start:raise ValueError('Sensor initialization requires an explicit stationary hold')
        self.world=world;self.adapter=adapter;self.sensors=[];self.filter=None
        self.accepted=0;self.rejected=0;self.invalid_imu=0
        self.last_gnss=None;self.last_imu=None;self.acceleration=0.;self.timestamp=float('-inf')
        self.prediction_time=None;self.start_time=None;self.initialization=None
        self.faults=faults;self.lock=threading.Condition();self.pending=[];self.last_packets={};self.raw_frames={};self.raw_counts={'gnss':0,'imu':0}
        self.last_seen={};self.packet_log=[];self.rejections={'out_of_order':0,'future':0,'nonfinite':0,'dropped':0,'imu_gap':0}
        georef=world.get_map();p0=georef.transform_to_geolocation(carla.Location());px=georef.transform_to_geolocation(carla.Location(x=1));py=georef.transform_to_geolocation(carla.Location(y=1))
        self.geo_origin=np.array([p0.longitude,p0.latitude]);self.geo_inverse=np.linalg.inv(np.array([[px.longitude-p0.longitude,py.longitude-p0.longitude],[px.latitude-p0.latitude,py.latitude-p0.latitude]]))
        self.noise_attributes={}
        for name in ('gnss','imu'):
            bp=world.get_blueprint_library().find('sensor.other.'+name);attributes={'sensor_tick':'0.05','noise_seed':str(seed)}
            if name=='gnss':attributes.update(noise_lat_stddev='0.000001' if noise else '0',noise_lon_stddev='0.000001' if noise else '0')
            else:attributes.update(noise_accel_stddev_x='0.05' if noise else '0',noise_gyro_stddev_z='0.005' if noise else '0')
            for key,value in attributes.items():bp.set_attribute(key,value)
            self.noise_attributes[name]=attributes
            sensor=world.spawn_actor(bp,carla.Transform(),attach_to=vehicle);self.sensors.append(sensor)
            sensor.listen(lambda data,name=name:self._receive(name,data))

    @property
    def ready(self):return self.filter is not None
    @property
    def state(self):
        if not self.ready:raise RuntimeError('No sensor-only stationary initialization yet')
        return self.filter.state

    def _receive(self,kind,data):
        values=(data.longitude,data.latitude) if kind=='gnss' else (data.accelerometer.x,-data.gyroscope.z,data.compass)
        packet=SensorPacket(kind,float(data.timestamp),int(data.frame),values)
        with self.lock:
            if self.start_time is None:self.start_time=packet.timestamp
            self.raw_frames[kind]=packet.frame;self.raw_counts[kind]+=1
            elapsed=packet.timestamp-self.start_time
            dropped=any(begin<=elapsed<end for begin,end in self.faults.dropout_windows)
            self.packet_log.append({'source':'REAL_CARLA_CALLBACK','kind':kind,'timestamp':packet.timestamp,'frame':packet.frame,'values':packet.values,'transport_dropped':dropped})
            if dropped:self.rejections['dropped']+=1
            else:
                self.pending.append(packet)
                previous=self.last_packets.get(kind)
                if self.faults.reorder_every and self.raw_counts[kind]%self.faults.reorder_every==0 and previous is not None:self.pending.append(previous)
                if self.faults.future_every and self.raw_counts[kind]%self.faults.future_every==0:self.pending.append(SensorPacket(kind,packet.timestamp+1,packet.frame,packet.values))
            self.last_packets[kind]=packet;self.lock.notify_all()

    def update(self,dt=None,*,now=None,frame=None,timeout=.5):
        # Legacy dt is accepted, but every prediction uses measurement time.
        if now is None:now=self.world.get_snapshot().timestamp.elapsed_seconds
        with self.lock:
            deadline=time.monotonic()+timeout
            while frame is not None and any(self.raw_frames.get(kind,-1)<frame for kind in ('gnss','imu')) and time.monotonic()<deadline:self.lock.wait(max(0,deadline-time.monotonic()))
            packets=self.pending;self.pending=[]
        valid=[]
        for packet in packets:
            if not all(math.isfinite(v) for v in (packet.timestamp,*packet.values)):
                self.rejections['nonfinite']+=1;continue
            if packet.timestamp>now+1e-6:
                self.rejections['future']+=1;continue
            if packet.timestamp<=self.last_seen.get(packet.kind,float('-inf')):
                self.rejections['out_of_order']+=1;continue
            self.last_seen[packet.kind]=packet.timestamp
            if packet.kind=='imu' and (abs(packet.values[0])>20 or abs(packet.values[1])>3):
                self.invalid_imu+=1;continue
            valid.append(packet)
        # Reject transport disorder before ordering independent streams for fusion.
        for packet in sorted(valid,key=lambda p:(p.timestamp,p.kind!='imu')):
            if packet.kind=='imu':
                acceleration,yaw_rate,compass=packet.values
                self.last_imu=packet;self.acceleration=acceleration
                if self.ready:
                    step=packet.timestamp-self.prediction_time
                    if 0<step<=.2+1e-8:self.filter.predict(acceleration,yaw_rate,min(step,.2))
                    elif step>.2:self.rejections['imu_gap']+=1
                    self.filter.update_heading(math.pi/2-compass)
                    self.prediction_time=packet.timestamp
            else:
                self.last_gnss=packet
                if self.ready:
                    xy=self._rear_position(packet,self.state.psi)
                    try:accepted=self.filter.update_position(*xy,packet.timestamp)
                    except ValueError:accepted=False
                    self.accepted+=int(accepted);self.rejected+=int(not accepted)
            if not self.ready and self.last_gnss is not None and self.last_imu is not None and abs(self.last_gnss.timestamp-self.last_imu.timestamp)<=.1:
                psi=math.atan2(math.sin(math.pi/2-self.last_imu.values[2]),math.cos(math.pi/2-self.last_imu.values[2]))
                xy=self._rear_position(self.last_gnss,psi)
                # The explicit parking-brake hold supplies a stationary speed
                # prior, not an unknown +/-1 m/s velocity inferred from noisy GNSS.
                self.filter=PlanarEKF(VehicleState(*xy,psi,0.),initial_velocity_variance=.0001);self.filter.timestamp=self.last_gnss.timestamp
                self.prediction_time=self.last_imu.timestamp;self.accepted+=1
                self.initialization={'source':'REAL_GNSS_IMU_COMPASS_STATIONARY_HOLD','timestamp':self.prediction_time,'x':self.state.x,'y':self.state.y,'yaw':psi,'speed':0.,'initial_velocity_variance':.0001}
        if self.ready:self.timestamp=min(self.prediction_time,self.filter.timestamp)
        return self.state if self.ready else None

    def _rear_position(self,packet,psi):
        xy=self.geo_inverse@(np.array(packet.values)-self.geo_origin)
        return (float(xy[0])-self.adapter.rear_offset*math.cos(psi),-float(xy[1])-self.adapter.rear_offset*math.sin(psi))
    def close(self):
        for sensor in reversed(self.sensors):sensor.stop();sensor.destroy()
