"""Real CARLA GNSS + IMU acquisition and local map calibration for PlanarEKF."""
import math
import numpy as np
from vehicle_model.state_estimator import PlanarEKF
class CarlaSensorEstimator:
    def __init__(self,world,vehicle,adapter):
        import carla
        self.world=world;self.vehicle=vehicle;self.adapter=adapter;self.gnss=None;self.imu=None;self.last_gnss=None;self.sensors=[]
        self.filter=PlanarEKF(adapter.get_state());self.accepted=0;self.rejected=0;self.invalid_imu=0
        # Derive the local GNSS-to-world transform from the loaded OpenDRIVE map.
        georef=world.get_map();p0=georef.transform_to_geolocation(carla.Location());px=georef.transform_to_geolocation(carla.Location(x=1));py=georef.transform_to_geolocation(carla.Location(y=1))
        self.geo_origin=np.array([p0.longitude,p0.latitude]);self.geo_inverse=np.linalg.inv(np.array([[px.longitude-p0.longitude,py.longitude-p0.longitude],[px.latitude-p0.latitude,py.latitude-p0.latitude]]))
        for name in ['gnss','imu']:
            bp=world.get_blueprint_library().find('sensor.other.'+name);bp.set_attribute('sensor_tick','0.05')
            sensor=world.spawn_actor(bp,carla.Transform(),attach_to=vehicle);self.sensors.append(sensor)
            sensor.listen(lambda data,name=name:setattr(self,name,data))
    def update(self,dt):
        if self.imu is not None:
            acceleration=self.imu.accelerometer.x;yaw_rate=-self.imu.gyroscope.z
            # Newly spawned CARLA IMU can emit an uninitialized derivative spike.
            # Reject impossible packets rather than letting the EKF diverge and
            # consequently reject every otherwise-valid GNSS observation.
            if abs(acceleration)<=20 and abs(yaw_rate)<=3:
                self.filter.predict(acceleration,yaw_rate,dt)
            else:self.invalid_imu+=1
        if self.gnss is not None and self.gnss.timestamp != self.last_gnss:
            xy=self.geo_inverse@(np.array([self.gnss.longitude,self.gnss.latitude])-self.geo_origin)
            psi=self.filter.state.psi;xy[0]-=self.adapter.rear_offset*math.cos(psi);xy[1]=-xy[1]-self.adapter.rear_offset*math.sin(psi)
            valid=self.filter.update_position(*xy,self.gnss.timestamp);self.last_gnss=self.gnss.timestamp
            self.accepted += int(valid);self.rejected += int(not valid)
        return self.filter.state
    def close(self):
        for sensor in reversed(self.sensors):sensor.stop();sensor.destroy()
