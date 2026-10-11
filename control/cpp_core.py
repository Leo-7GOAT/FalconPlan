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
    def stanley(self,heading,cte,speed,gain=1.5,softening=1,limit=.5):
        result=ctypes.c_double();code=self.lib.falcon_stanley(heading,cte,speed,gain,softening,limit,ctypes.byref(result))
        if code:raise ValueError('C++ Stanley rejected invalid input')
        return result.value
    def bicycle(self,state: VehicleState,command: VehicleCommand,dt):
        q=(ctypes.c_double*4)(state.x,state.y,state.psi,state.v);u=(ctypes.c_double*2)(command.delta,command.a);result=(ctypes.c_double*4)()
        if self.lib.falcon_bicycle(q,u,dt,result):raise ValueError('C++ bicycle rejected invalid input')
        return VehicleState(*result)
