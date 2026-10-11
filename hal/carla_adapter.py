"""REAL CARLA 0.9.15 VehicleHAL, with explicit handedness conversion.

Core coordinates are x-forward/y-left, CCW yaw, radians and SI; CARLA uses
y-right and clockwise yaw. Ground-truth localization/actors, no perception claims.
"""
import math
from vehicle_model.models import VehicleState,VehicleCommand
from .base import VehicleHAL

class CarlaAdapter(VehicleHAL):
    def __init__(self,client,world,vehicle,dt=.05,control_mode='normalized'):
        self.client=client;self.world=world;self.vehicle=vehicle;self.dt=dt
        physics=vehicle.get_physics_control()
        # CARLA WheelPhysicsControl positions are WORLD-frame centimeters.
        # Project into the actor forward axis before estimating axle offsets.
        pose=vehicle.get_transform();yaw=math.radians(pose.rotation.yaw)
        xs=[(w.position.x/100-pose.location.x)*math.cos(yaw)+(w.position.y/100-pose.location.y)*math.sin(yaw) for w in physics.wheels]
        self.rear_offset=abs(min(xs)) if xs else 1.4
        self.wheelbase=max(xs)-min(xs) if xs else 2.8
        self.max_steer=math.radians(max(w.max_steer_angle for w in physics.wheels))
        self.timestamp=world.get_snapshot().timestamp.elapsed_seconds
        self.last_command=VehicleCommand(0,0)
        if control_mode not in ('ackermann','normalized'):raise ValueError('unknown CARLA control mode')
        self.control_mode=control_mode
        self.target_speed=0.0
        self.throttle=0.0
        self.brake=0.0
    def get_state(self):
        t=self.vehicle.get_transform();v=self.vehicle.get_velocity()
        psi=-math.radians(t.rotation.yaw)
        self.timestamp=self.world.get_snapshot().timestamp.elapsed_seconds
        speed=v.x*math.cos(psi)-v.y*math.sin(psi)
        if abs(speed)<1e-3:speed=0.0  # simulator resting-contact numerical noise
        return VehicleState(t.location.x-self.rear_offset*math.cos(psi),-t.location.y-self.rear_offset*math.sin(psi),psi,speed)
    def apply_command(self,command):
        import carla
        self.last_command=command
        if self.control_mode=='ackermann' and command.a > -5.99:
            # Use CARLA's native speed/acceleration servo, not an uncalibrated
            # acceleration-to-throttle gain. Reference integrates in sim time.
            self.target_speed=max(0,min(12,self.target_speed+command.a*self.dt))
            self.vehicle.apply_ackermann_control(carla.VehicleAckermannControl(steer=-command.delta,speed=self.target_speed,acceleration=command.a,jerk=6.0))
            return
        if command.a<=-5.99:self.target_speed=0
        target_throttle=max(0.,min(1.,.12+command.a/10.0)) if command.a>.05 else 0.
        acceleration=self.vehicle.get_acceleration();pose=self.vehicle.get_transform()
        forward_acc=acceleration.x*math.cos(math.radians(pose.rotation.yaw))+acceleration.y*math.sin(math.radians(pose.rotation.yaw))
        if forward_acc>2.7:target_throttle=min(target_throttle,self.throttle*2.7/forward_acc)
        self.throttle=max(0,min(target_throttle,self.throttle+.25*self.dt)) if command.a>0 else 0
        target_brake=max(0,min(1,-command.a/6))
        self.brake=target_brake if command.a<=-5.99 else max(self.brake-.08*self.dt,min(self.brake+.08*self.dt,target_brake))
        self.vehicle.apply_control(carla.VehicleControl(
            steer=max(-1.,min(1.,-command.delta/self.max_steer)),
            # Nominal inverse acceleration gain, not calibrated hardware data.
            # A conservative throttle mapping prevents full-throttle model mismatch.
            throttle=self.throttle,
            brake=self.brake,manual_gear_shift=True,gear=1))
    def step(self,dt):
        if not math.isfinite(dt) or abs(dt-self.dt)>1e-9:raise ValueError("CARLA synchronous dt must equal configured fixed delta")
        self.world.tick()
        return self.get_state()
    def nearby_actors(self,radius=80):
        state=self.get_state();result=[]
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.id==self.vehicle.id:continue
            t=actor.get_transform();v=actor.get_velocity()
            if math.hypot(t.location.x-state.x,-t.location.y-state.y)<=radius:
                result.append(dict(id=actor.id,x=t.location.x,y=-t.location.y,vx=v.x,vy=-v.y,timestamp=self.timestamp))
        return result
