#pragma once
#include <array>
#include <vector>
namespace falcon {
struct CandidateSample { double x,y,lateral_jerk,longitudinal_jerk; };
struct Circle { double x,y,radius; };
struct Candidate {
 std::vector<CandidateSample> samples;
 double target_d,target_speed,duration;
};
struct CandidateEvaluation { std::array<double,6> costs; bool collision; };
CandidateEvaluation evaluate_candidate(const Candidate& candidate,
    const std::vector<Circle>& obstacles,const std::array<double,5>& weights,
    double desired_d,double desired_speed,double vehicle_radius,double margin);
}
