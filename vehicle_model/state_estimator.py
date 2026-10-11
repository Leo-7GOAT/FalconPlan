"""Planar EKF for timestamped GNSS position / IMU yaw rate and forward acceleration.

Gravity compensation, sensor extrinsics and bias identification must precede real
hardware use. CARLA reports ideal GNSS/IMU unless sensor noise is configured.
"""
import math
import numpy as np
from .models import VehicleState
class PlanarEKF:
    def __init__(self,state: VehicleState,position_variance=.04,process_variance=.1):
        if position_variance<=0 or process_variance<=0:raise ValueError('positive variances required')
        self.mean=np.array([state.x,state.y,state.psi,state.v],dtype=float);self.cov=np.eye(4)
        self.position_variance=position_variance;self.process_variance=process_variance;self.timestamp=None
    def predict(self,acceleration,yaw_rate,dt):
        if not all(map(math.isfinite,(acceleration,yaw_rate,dt))) or not 0<dt<=.2:raise ValueError('invalid IMU/time step')
        x,y,psi,v=self.mean;F=np.eye(4);F[0,2]=-v*math.sin(psi)*dt;F[0,3]=math.cos(psi)*dt;F[1,2]=v*math.cos(psi)*dt;F[1,3]=math.sin(psi)*dt
        self.mean=np.array([x+v*math.cos(psi)*dt,y+v*math.sin(psi)*dt,psi+yaw_rate*dt,max(0,v+acceleration*dt)])
        self.cov=F@self.cov@F.T+np.eye(4)*self.process_variance*dt
        return self.state
    def update_position(self,x,y,timestamp):
        if not all(map(math.isfinite,(x,y,timestamp))):raise ValueError('nonfinite GNSS')
        if self.timestamp is not None and timestamp<=self.timestamp:raise ValueError('duplicate or out of order GNSS')
        H=np.array([[1,0,0,0],[0,1,0,0]],dtype=float);R=np.eye(2)*self.position_variance;innovation=np.array([x,y])-H@self.mean
        S=H@self.cov@H.T+R;K=np.linalg.solve(S,(self.cov@H.T).T).T
        if float(innovation@np.linalg.solve(S,innovation))>25: return False
        self.mean += K@innovation
        # Joseph form preserves PSD under finite precision.
        I=np.eye(4)-K@H;self.cov=I@self.cov@I.T+K@R@K.T;self.timestamp=timestamp
        return True
    @property
    def state(self):return VehicleState(*map(float,self.mean))
