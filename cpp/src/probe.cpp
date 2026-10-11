#include "falcon/core.hpp"
#include <iostream>
#include <iomanip>
int main(){double x,y,yaw,v,delta,a,dt;std::cout<<std::setprecision(17);while(std::cin>>x>>y>>yaw>>v>>delta>>a>>dt){auto q=falcon::bicycle_step({x,y,yaw,v},{delta,a},dt);auto s=falcon::stanley(yaw,y,v);std::cout<<q.x<<' '<<q.y<<' '<<q.yaw<<' '<<q.v<<' '<<s<<'\n';}}
