#include "falcon/planning.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace falcon {
namespace {
void finite(double value) {
 if(!std::isfinite(value)) throw std::invalid_argument("non-finite planning input");
}
void nonnegative(double value) {
 finite(value);if(value<0)throw std::invalid_argument("negative planning parameter");
}
double segment_distance(double px,double py,const CandidateSample& a,const CandidateSample& b) {
 const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
 finite(dx);finite(dy);finite(l2);
 if(l2<=1e-12){const double value=std::hypot(px-a.x,py-a.y);finite(value);return value;}
 const double projection=((px-a.x)*dx+(py-a.y)*dy)/l2;finite(projection);
 const double lambda=std::clamp(projection,0.0,1.0);
 const double result=std::hypot(px-(a.x+lambda*dx),py-(a.y+lambda*dy));finite(result);return result;
}
}
CandidateEvaluation evaluate_candidate(const Candidate& c,const std::vector<Circle>& obstacles,
 const std::array<double,5>& weights,double desired_d,double desired_speed,double radius,double margin) {
 if(c.samples.empty())throw std::invalid_argument("empty candidate");
 finite(c.target_d);finite(desired_d);nonnegative(c.target_speed);nonnegative(desired_speed);
 finite(c.duration);if(c.duration<=0)throw std::invalid_argument("duration must be positive");
 nonnegative(radius);nonnegative(margin);
 for(double w:weights)nonnegative(w);
 double lj=0,sj=0;
 for(const auto& p:c.samples) {
  finite(p.x);finite(p.y);finite(p.lateral_jerk);finite(p.longitudinal_jerk);
  lj+=p.lateral_jerk*p.lateral_jerk;sj+=p.longitudinal_jerk*p.longitudinal_jerk;
 }
 const double d=c.target_d-desired_d,v=c.target_speed-desired_speed;
 CandidateEvaluation result{{d*d,v*v,lj/c.samples.size(),sj/c.samples.size(),c.duration,0},false};
 for(std::size_t k=0;k<5;++k)result.costs[5]+=weights[k]*result.costs[k];
 for(double value:result.costs)finite(value); // Overflow is an invalid numeric result.
 for(const auto& o:obstacles) {
  finite(o.x);finite(o.y);nonnegative(o.radius);
  const double threshold=radius+margin+o.radius;finite(threshold);
  if(c.samples.size()==1) {
   const double distance=std::hypot(c.samples[0].x-o.x,c.samples[0].y-o.y);finite(distance);
   result.collision |= distance<=threshold;
  } else for(std::size_t k=1;k<c.samples.size();++k) {
   if(segment_distance(o.x,o.y,c.samples[k-1],c.samples[k])<=threshold)result.collision=true;
  }
 }
 return result;
}
}
