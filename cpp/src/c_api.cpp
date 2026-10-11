#include "falcon/core.hpp"
#include <exception>
extern "C" {
int falcon_stanley(double heading,double cte,double speed,double gain,double softening,double limit,double* output) noexcept {
 if(!output)return -1;
 try {*output=falcon::stanley(heading,cte,speed,gain,softening,limit);return 0;}catch(const std::exception&){return -2;}
}
int falcon_bicycle(const double* state,const double* command,double dt,double* output) noexcept {
 if(!state||!command||!output)return -1;
 try {auto q=falcon::bicycle_step({state[0],state[1],state[2],state[3]},{command[0],command[1]},dt);output[0]=q.x;output[1]=q.y;output[2]=q.yaw;output[3]=q.v;return 0;}catch(const std::exception&){return -2;}
}
}
