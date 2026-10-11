#include "falcon/core.hpp"
#include "falcon/planning.hpp"
#include "falcon/c_api.h"
#include <algorithm>
#include <exception>
#include <limits>
#include <stdexcept>
extern "C" {
int falcon_stanley(double heading,double cte,double speed,double gain,double softening,double limit,double* output) noexcept {
 if(!output)return -1;
 try {*output=falcon::stanley(heading,cte,speed,gain,softening,limit);return 0;}catch(const std::exception&){return -2;}
}
int falcon_bicycle(const double* state,const double* command,double dt,double* output) noexcept {
 if(!state||!command||!output)return -1;
 try {auto q=falcon::bicycle_step({state[0],state[1],state[2],state[3]},{command[0],command[1]},dt);output[0]=q.x;output[1]=q.y;output[2]=q.yaw;output[3]=q.v;return 0;}catch(const std::exception&){return -2;}
}
int falcon_rank_candidates(const double* samples,std::size_t sample_count,
 const std::size_t* offsets,std::size_t candidate_count,const double* metadata,
 const double* obstacles,std::size_t obstacle_count,const double* weights,
 double desired_d,double desired_speed,double radius,double margin,
 double* costs,int* collisions,std::size_t* best_index) noexcept {
 if(!samples||!offsets||!metadata||!weights||!costs||!collisions||!best_index||(!obstacles&&obstacle_count))return -1;
 try {
  if(!sample_count||sample_count>1000000||!candidate_count||candidate_count>10000||obstacle_count>10000)
   throw std::invalid_argument("invalid planning count");
  if(offsets[0]!=0||offsets[candidate_count]!=sample_count)throw std::invalid_argument("invalid offsets");
  std::vector<falcon::Circle> circles;circles.reserve(obstacle_count);
  for(std::size_t i=0;i<obstacle_count;++i)circles.push_back({obstacles[3*i],obstacles[3*i+1],obstacles[3*i+2]});
  std::array<double,5> w;std::copy(weights,weights+5,w.begin());
  std::vector<falcon::CandidateEvaluation> evaluated;evaluated.reserve(candidate_count);
  std::size_t best=std::numeric_limits<std::size_t>::max();double minimum=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<candidate_count;++i) {
   const auto begin=offsets[i],end=offsets[i+1];
   if(begin>=end||end>sample_count)throw std::invalid_argument("empty or invalid candidate offsets");
   falcon::Candidate c{{},metadata[3*i],metadata[3*i+1],metadata[3*i+2]};c.samples.reserve(end-begin);
   for(std::size_t k=begin;k<end;++k)c.samples.push_back({samples[4*k],samples[4*k+1],samples[4*k+2],samples[4*k+3]});
   const auto result=falcon::evaluate_candidate(c,circles,w,desired_d,desired_speed,radius,margin);
   evaluated.push_back(result);
   if(!result.collision&&result.costs[5]<minimum){minimum=result.costs[5];best=i;}
  }
  for(std::size_t i=0;i<candidate_count;++i){std::copy(evaluated[i].costs.begin(),evaluated[i].costs.end(),costs+6*i);collisions[i]=evaluated[i].collision?1:0;}
  *best_index=best;return 0;
 }catch(const std::exception&){return -2;}
}
}
