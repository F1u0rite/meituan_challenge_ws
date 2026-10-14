#include <gtest/gtest.h>
#include "mtc_aubo_fake_integration_cpp/aubo_fjt_motion_port.hpp"
#include "mtc_aubo_fake_integration_cpp/fsm_injection.hpp"
using namespace mtc::aubo_fake;
class AuboCommand : public testing::Test {
protected:
  mtc::trajectory::Config cfg;mtc::moveit_planner::RawTrajectory raw;
  void SetUp() override {
    for(const auto & name:cfg.joints)cfg.limits[name]={-1,1,1,10};
    raw.joint_names=cfg.joints;
    for(int i=0;i<3;++i) {mtc::moveit_planner::RawTrajectoryPoint p;p.time_from_start_s=i;
      p.positions.assign(6,i*.01);p.velocities.assign(6,.01);p.accelerations.assign(6,0);raw.points.push_back(p);}
  }
  auto command(double factor=1) {return command_from_plan(raw,cfg,"world","explicit-test",factor);}
};
TEST_F(AuboCommand, CompleteSixJointTrajectoryPreserved) {auto c=command();ASSERT_TRUE(c.trajectory);EXPECT_EQ(c.trajectory->points.size(),3u);EXPECT_EQ(c.trajectory->joint_names,cfg.joints);EXPECT_EQ(c.trajectory->points.back().positions_rad,raw.points.back().positions);EXPECT_EQ(c.trajectory->stamp_s,0);}
TEST_F(AuboCommand, NamesAreMappedNotAssumed) {std::reverse(raw.joint_names.begin(),raw.joint_names.end());for(auto & p:raw.points)for(size_t j=0;j<6;++j)p.positions[j]=.01*j;auto c=command();EXPECT_NEAR(c.trajectory->points.front().positions_rad.front(),.05,1e-9);}
TEST_F(AuboCommand, ExplicitSlowdownPreservesSourceAndScalesDerivatives) {const auto before=raw.points;auto c=command(3);EXPECT_EQ(raw.points[2].time_from_start_s,2);EXPECT_EQ(c.trajectory->points[2].time_from_start_s,6);EXPECT_DOUBLE_EQ(c.trajectory->points[1].velocities_rad_s[0],.01/3);EXPECT_EQ(c.trajectory->points[1].positions_rad,before[1].positions);}
TEST_F(AuboCommand, EmptyTrajectoryRejected) {raw.points.clear();EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, EndpointOnlyRejected) {raw.points.resize(1);EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, RepeatedJointRejected) {raw.joint_names[0]=raw.joint_names[1];EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, NonzeroFirstTimeRejected) {raw.points[0].time_from_start_s=.1;EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, NonIncreasingTimeRejected) {raw.points[2].time_from_start_s=1;EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, NanRejected) {raw.points[1].positions[0]=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, SplineLimitsCannotBeBypassed) {raw.points[1].positions[0]=2;EXPECT_THROW(command(),std::invalid_argument);}
TEST_F(AuboCommand, AccelerationScalingPreserved) {for(auto & p:raw.points)p.accelerations.assign(6,.02);auto c=command(2);EXPECT_DOUBLE_EQ(c.trajectory->points[0].accelerations_rad_s2[0],.005);}
TEST_F(AuboCommand, MissingFrameOrIdentityRejected) {
  EXPECT_THROW(command_from_plan(raw,cfg,"","id"),std::invalid_argument);
  EXPECT_THROW(command_from_plan(raw,cfg,"world",""),std::invalid_argument);
}
TEST_F(AuboCommand, UnsafeRetimingRejected) {
  EXPECT_THROW(command(.5),std::invalid_argument);
  EXPECT_THROW(command(5),std::invalid_argument);
}
TEST_F(AuboCommand, ExpiredOrUnverifiedPlanTimeRejected) {EXPECT_TRUE(plan_is_fresh(100,100.1));EXPECT_FALSE(plan_is_fresh(90,100));EXPECT_FALSE(plan_is_fresh(101,100));EXPECT_FALSE(plan_is_fresh(std::numeric_limits<double>::quiet_NaN(),100));}
