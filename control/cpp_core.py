"""Optional in-process C++17 core; no process launch in the control cycle."""
import ctypes
from pathlib import Path
from vehicle_model.models import VehicleState,VehicleCommand
class CppCore:
    def __init__(self,path):
        self.lib=ctypes.CDLL(str(Path(path).resolve()))
        pointer=ctypes.POINTER(ctypes.c_double)
        self.lib.falcon_stanley.argtypes=[ctypes.c_double]*6+[pointer];self.lib.falcon_stanley.restype=ctypes.c_int
        self.lib.falcon_bicycle.argtypes=[pointer,pointer,ctypes.c_double,pointer];self.lib.falcon_bicycle.restype=ctypes.c_int
        self.lib.falcon_rank_candidates.argtypes=[pointer,ctypes.c_size_t,ctypes.POINTER(ctypes.c_size_t),ctypes.c_size_t,pointer,pointer,ctypes.c_size_t,pointer,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,pointer,ctypes.POINTER(ctypes.c_int),ctypes.POINTER(ctypes.c_size_t)]
        self.lib.falcon_rank_candidates.restype=ctypes.c_int
    def stanley(self,heading,cte,speed,gain=1.5,softening=1,limit=.5):
        result=ctypes.c_double();code=self.lib.falcon_stanley(heading,cte,speed,gain,softening,limit,ctypes.byref(result))
        if code:raise ValueError('C++ Stanley rejected invalid input')
        return result.value
    def bicycle(self,state: VehicleState,command: VehicleCommand,dt):
        q=(ctypes.c_double*4)(state.x,state.y,state.psi,state.v);u=(ctypes.c_double*2)(command.delta,command.a);result=(ctypes.c_double*4)()
        if self.lib.falcon_bicycle(q,u,dt,result):raise ValueError('C++ bicycle rejected invalid input')
        return VehicleState(*result)
    def rank_candidates(self,trajectories,obstacles,params,weights,desired_d,desired_speed):
        """Batch continuous collision filtering, exact legacy cost and stable selection.

        World-projected candidates must retain one jerk sample per world sample.
        Returns (best index or None, per-candidate cost breakdowns, blocked flags).
        Polynomial generation and dynamic/world feasibility remain Python.
        """
        import numpy as np
        from planning.cost import TrajectoryCostBreakdown
        if not trajectories:raise ValueError('no candidate trajectories')
        samples=[];offsets=[0];metadata=[]
        for trajectory in trajectories:
            n=len(trajectory.t)
            arrays=(trajectory.x,trajectory.y,trajectory.d_ddd,trajectory.s_ddd)
            if not n or any(len(a)!=n for a in arrays):raise ValueError('candidate sample lengths mismatch')
            samples.extend(zip(*arrays));offsets.append(len(samples))
            metadata.append((trajectory.target_d,trajectory.target_speed,trajectory.duration))
        packed=np.ascontiguousarray(samples,dtype=np.float64)
        indices=np.ascontiguousarray(offsets,dtype=np.uintp)
        meta=np.ascontiguousarray(metadata,dtype=np.float64)
        circles=np.ascontiguousarray([(o.x,o.y,o.radius) for o in obstacles],dtype=np.float64)
        cost_weights=np.ascontiguousarray([weights.lateral_offset,weights.speed_error,weights.lateral_jerk,weights.longitudinal_jerk,weights.duration],dtype=np.float64)
        costs=np.empty((len(trajectories),6),dtype=np.float64);blocked=np.empty(len(trajectories),dtype=np.int32);best=ctypes.c_size_t()
        ptr=ctypes.POINTER(ctypes.c_double)
        code=self.lib.falcon_rank_candidates(packed.ctypes.data_as(ptr),len(packed),indices.ctypes.data_as(ctypes.POINTER(ctypes.c_size_t)),len(trajectories),meta.ctypes.data_as(ptr),circles.ctypes.data_as(ptr) if len(circles) else None,len(obstacles),cost_weights.ctypes.data_as(ptr),desired_d,desired_speed,params.vehicle_radius,params.safety_margin,costs.ctypes.data_as(ptr),blocked.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),ctypes.byref(best))
        if code:raise ValueError('C++ candidate evaluator rejected invalid input')
        winner=None if best.value==ctypes.c_size_t(-1).value else best.value
        return winner,[TrajectoryCostBreakdown(*map(float,row)) for row in costs],list(map(bool,blocked))
