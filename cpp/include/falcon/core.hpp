#pragma once
#include <vector>
namespace falcon {
struct State { double x,y,yaw,v; };
struct Command { double steering,acceleration; };
struct Params { double wheelbase=2.8,max_steer=.5,max_accel=3,max_decel=6,max_speed=30; };
struct PathPoint { double x,y,yaw,speed; };
double stanley(double heading,double cte,double speed,double gain=1,double softening=1,double max_steer=.5);
State bicycle_step(State state,Command command,double dt,Params params={});
std::size_t nearest_point(State state,const std::vector<PathPoint>& path,double wheelbase);
struct Longitudinal { double s,v,a; };
Longitudinal constant_jerk(Longitudinal state,double jerk,double dt);
class PID {
 public:
  PID(double kp,double ki,double kd,double low,double high);
  double update(double target,double measurement,double dt,double feedforward=0);
  void reset();
 private:
  double kp_,ki_,kd_,low_,high_,integral_=0,previous_=0; bool first_=true;
};
}
