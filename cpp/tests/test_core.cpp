#include "falcon/core.hpp"
#include <gtest/gtest.h>
#include <limits>
extern "C" int falcon_stanley(double,double,double,double,double,double,double*) noexcept;
extern "C" int falcon_bicycle(const double*,const double*,double,double*) noexcept;
TEST(CAbi,NullAndInvalidBoundaries) {
 double result=0;
 EXPECT_EQ(falcon_stanley(0,0,1,1,1,.5,nullptr),-1);
 EXPECT_EQ(falcon_stanley(0,0,-1,1,1,.5,&result),-2);
 EXPECT_EQ(falcon_stanley(0,std::numeric_limits<double>::quiet_NaN(),1,1,1,.5,&result),-2);
 double state[]{0,0,0,1},command[]{0,0},output[4]{};
 EXPECT_EQ(falcon_bicycle(nullptr,command,.1,output),-1);
 EXPECT_EQ(falcon_bicycle(state,nullptr,.1,output),-1);
 EXPECT_EQ(falcon_bicycle(state,command,.1,nullptr),-1);
 EXPECT_EQ(falcon_bicycle(state,command,0,output),-2);
 EXPECT_EQ(falcon_bicycle(state,command,.1,output),0);
 EXPECT_NEAR(output[0],.1,1e-12);
}
TEST(Stanley,SignsLimitsAndFailure){EXPECT_GT(falcon::stanley(0,1,10),0);
EXPECT_DOUBLE_EQ(falcon::stanley(0,100,0),.5);
EXPECT_THROW(falcon::stanley(0,0,-1),std::invalid_argument);
EXPECT_THROW(falcon::stanley(0,std::numeric_limits<double>::quiet_NaN(),1),std::invalid_argument);}
TEST(Bicycle,AnalyticStraightAndLimits){auto q=falcon::bicycle_step({0,0,0,10},{0,0},.1);
EXPECT_NEAR(q.x,1,1e-12);
EXPECT_DOUBLE_EQ(q.y,0);q=falcon::bicycle_step({0,0,0,0},{0,-6},.1);
EXPECT_DOUBLE_EQ(q.v,0);
EXPECT_THROW(falcon::bicycle_step(q,{0,0},0),std::invalid_argument);}
TEST(Jerk,Analytic){auto q=falcon::constant_jerk({0,2,1},3,1);
EXPECT_DOUBLE_EQ(q.s,3);
EXPECT_DOUBLE_EQ(q.v,4.5);
EXPECT_DOUBLE_EQ(q.a,4);
EXPECT_THROW(falcon::constant_jerk(q,0,-1),std::invalid_argument);}
TEST(PID,AntiWindupAndReset){falcon::PID p(1,1,0,-3,3);for(int i=0;i<100;++i)EXPECT_DOUBLE_EQ(p.update(100,0,.1),3);
EXPECT_DOUBLE_EQ(p.update(0,0,.1),0);p.reset();
EXPECT_DOUBLE_EQ(p.update(1,0,.1),1.1);}
TEST(Path,SearchAndFailure){std::vector<falcon::PathPoint> p{{0,0,0,1},{3,0,0,1},{6,0,0,1}};
EXPECT_EQ(falcon::nearest_point({0,0,0,1},p,2.8),1);
EXPECT_THROW(falcon::nearest_point({0,0,0,1},{},2.8),std::invalid_argument);}

