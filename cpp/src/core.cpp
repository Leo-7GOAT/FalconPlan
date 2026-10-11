#include "falcon/core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace falcon {
namespace { void valid(double v) { if(!std::isfinite(v)) throw std::invalid_argument("nonfinite input"); }
void positive(double v) { valid(v);if(v<=0)throw std::invalid_argument("positive input required"); }
State derivative(State q,Command u,double L) { return {q.v*std::cos(q.yaw),q.v*std::sin(q.yaw),q.v/L*std::tan(u.steering),u.acceleration}; }
State sum(State q,State k,double h) { return {q.x+h*k.x,q.y+h*k.y,q.yaw+h*k.yaw,q.v+h*k.v}; }
}
double stanley(double heading,double cte,double speed,double gain,double softening,double max_steer) {
valid(heading);valid(cte);valid(speed);positive(gain);positive(softening);positive(max_steer);
if(speed<0)throw std::invalid_argument("negative speed");
return std::clamp(std::atan2(std::sin(heading),std::cos(heading))+std::atan2(gain*cte,speed+softening),-max_steer,max_steer);
}
State bicycle_step(State q,Command u,double dt,Params p) {
valid(q.x);valid(q.y);valid(q.yaw);valid(q.v);valid(u.steering);valid(u.acceleration);positive(dt);positive(p.wheelbase);
positive(p.max_steer);positive(p.max_accel);positive(p.max_decel);positive(p.max_speed);
if(p.max_steer>=1.5707963267948966)throw std::invalid_argument("steering singularity");
u.steering=std::clamp(u.steering,-p.max_steer,p.max_steer);u.acceleration=std::clamp(u.acceleration,-p.max_decel,p.max_accel);
auto k1=derivative(q,u,p.wheelbase);auto k2=derivative(sum(q,k1,dt/2),u,p.wheelbase);auto k3=derivative(sum(q,k2,dt/2),u,p.wheelbase);auto k4=derivative(sum(q,k3,dt),u,p.wheelbase);
auto z=sum(sum(sum(sum(q,k1,dt/6),k2,dt/3),k3,dt/3),k4,dt/6);z.v=std::clamp(z.v,0.,p.max_speed);return z;
}
std::size_t nearest_point(State q,const std::vector<PathPoint>& path,double L) {
positive(L);if(path.empty())throw std::invalid_argument("empty trajectory");
valid(q.x);valid(q.y);valid(q.yaw);
double x=q.x+L*std::cos(q.yaw),y=q.y+L*std::sin(q.yaw),best=std::numeric_limits<double>::infinity();std::size_t index=0;
for(std::size_t i=0;i<path.size();++i){valid(path[i].x);valid(path[i].y);double d=std::pow(path[i].x-x,2)+std::pow(path[i].y-y,2);if(d<best){best=d;index=i;}}
return index;
}
Longitudinal constant_jerk(Longitudinal q,double j,double dt){valid(q.s);valid(q.v);valid(q.a);valid(j);positive(dt);return {q.s+q.v*dt+.5*q.a*dt*dt+j*dt*dt*dt/6,q.v+q.a*dt+.5*j*dt*dt,q.a+j*dt};}
PID::PID(double kp,double ki,double kd,double low,double high):kp_(kp),ki_(ki),kd_(kd),low_(low),high_(high){valid(kp);valid(ki);valid(kd);valid(low);valid(high);if(low>=high)throw std::invalid_argument("invalid PID limits");}
void PID::reset(){integral_=0;previous_=0;first_=true;}
double PID::update(double target,double measurement,double dt,double ff){valid(target);valid(measurement);valid(ff);positive(dt);double e=target-measurement,d=first_?0:(e-previous_)/dt,candidate=integral_+e*dt,u=ff+kp_*e+ki_*candidate+kd_*d;
if(!((u>high_&&e>0)||(u<low_&&e<0)))integral_=candidate;
previous_=e;first_=false;return std::clamp(ff+kp_*e+ki_*integral_+kd_*d,low_,high_);}
}
