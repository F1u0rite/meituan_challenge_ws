#include <gtest/gtest.h>
#include <limits>
#include "mtc_phase2c_integration_cpp/dispatch_adapter.hpp"
using namespace mtc::integration;
class Dispatch : public ::testing::Test {
protected:
  mtc::trajectory::Config cfg;
  mtc::core::MotionCommand source;
  mtc::trajectory::JointSample state;
  void SetUp() override {
    for(auto & j:cfg.joints) {cfg.limits[j]={-0.5,0.5,0.2,0.5};}
    source.simulation_only=true; source.timeout_ms=5000;
    mtc::core::TimedTrajectory t;t.joint_names=cfg.joints;t.header_frame_id="base_link";t.stamp_s=10;
    for(int i=0;i<3;++i) {
      mtc::core::TrajectoryPoint p;p.time_from_start_s=i;p.positions_rad.assign(6,i*0.01);
      p.velocities_rad_s.assign(6,0);p.accelerations_rad_s2.assign(6,0);t.points.push_back(p);
    }
    source.trajectory=t;state.positions.assign(6,0);state.velocities.assign(6,0);state.stamp_ns=100000000000LL;
  }
  DispatchResult run(DispatchPolicy policy=DispatchPolicy::IMMEDIATE,double steady=10.1,double scheduled=0) {
    DispatchAdapter a(cfg,"session");return a.prepare({source,GenerationClock::STEADY,10},policy,state,steady,100,scheduled);
  }
};
TEST_F(Dispatch, ExplicitImmediateKeepsOriginalGenerationStamp) {auto r=run();ASSERT_TRUE(r.command);EXPECT_EQ(r.command->trajectory->stamp_s,0);EXPECT_EQ(source.trajectory->stamp_s,10);}
TEST_F(Dispatch, ExpiredPlanRejected) {EXPECT_FALSE(run(DispatchPolicy::IMMEDIATE,13).command);}
TEST_F(Dispatch, SteadyClockNotInterpretedAsRosTime) {EXPECT_TRUE(run().command);}
TEST_F(Dispatch, NoImplicitImmediateAuthorization) {EXPECT_FALSE(run(DispatchPolicy::UNSPECIFIED).command);}
TEST_F(Dispatch, ValidRosScheduleRetainsTime) {auto r=run(DispatchPolicy::ROS_SCHEDULED,10.1,100.1);ASSERT_TRUE(r.command);EXPECT_EQ(r.command->trajectory->stamp_s,100.1);}
TEST_F(Dispatch, PastOrDistantRosScheduleRejected) {EXPECT_FALSE(run(DispatchPolicy::ROS_SCHEDULED,10.1,99).command);EXPECT_FALSE(run(DispatchPolicy::ROS_SCHEDULED,10.1,101).command);}
TEST_F(Dispatch, AllWaypointsAndDerivativesPreserved) {auto r=run();ASSERT_TRUE(r.command);EXPECT_EQ(r.command->trajectory->joint_names,source.trajectory->joint_names);for(size_t i=0;i<3;++i) {auto & a=r.command->trajectory->points[i];auto & b=source.trajectory->points[i];EXPECT_EQ(a.positions_rad,b.positions_rad);EXPECT_EQ(a.velocities_rad_s,b.velocities_rad_s);EXPECT_EQ(a.accelerations_rad_s2,b.accelerations_rad_s2);EXPECT_EQ(a.time_from_start_s,b.time_from_start_s);}}
TEST_F(Dispatch, DispatchIdentitiesUnique) {DispatchAdapter a(cfg,"session");auto r=a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100);auto s=a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100);ASSERT_TRUE(r.command);ASSERT_TRUE(s.command);EXPECT_NE(r.command->command_id,s.command->command_id);EXPECT_NE(r.command->trajectory->trajectory_id,s.command->trajectory->trajectory_id);}
TEST_F(Dispatch, UnknownClockRejected) {DispatchAdapter a(cfg,"session");EXPECT_FALSE(a.prepare({source,GenerationClock::UNKNOWN,10},DispatchPolicy::IMMEDIATE,state,10.1,100).command);}
TEST_F(Dispatch, StartMismatchRejected) {state.positions[0]=0.1;EXPECT_FALSE(run().command);}
TEST_F(Dispatch, ContactRejected) {source.requires_contact=true;EXPECT_FALSE(run().command);}
TEST_F(Dispatch, NonFiniteGenerationRejected) {DispatchAdapter a(cfg,"session");EXPECT_FALSE(a.prepare({source,GenerationClock::STEADY,std::numeric_limits<double>::quiet_NaN()},DispatchPolicy::IMMEDIATE,state,10,100).command);}
TEST_F(Dispatch, StaleCurrentStateRejected) {state.stamp_ns=99000000000LL;EXPECT_FALSE(run().command);}
TEST_F(Dispatch, SplineSafetyCheckRetained) {source.trajectory->points[1].positions_rad[0]=1;EXPECT_FALSE(run().command);}
TEST_F(Dispatch, ExplicitUniformSlowdownPreservesOriginalAndSpatialPath) {
  DispatchAdapter a(cfg,"session");auto r=a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100,0,2);
  ASSERT_TRUE(r.command);EXPECT_EQ(source.trajectory->duration_s(),2);EXPECT_EQ(r.command->trajectory->duration_s(),4);
  for(size_t i=0;i<3;++i) {EXPECT_EQ(r.command->trajectory->points[i].positions_rad,source.trajectory->points[i].positions_rad);}
  EXPECT_EQ(r.command->timeout_ms,7000u);
}
TEST_F(Dispatch, ExplicitRetimingTransformsDerivatives) {
  source.trajectory->points[1].velocities_rad_s.assign(6,0.02);source.trajectory->points[1].accelerations_rad_s2.assign(6,0.01);
  DispatchAdapter a(cfg,"session");auto r=a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100,0,2);
  ASSERT_TRUE(r.command);EXPECT_EQ(r.command->trajectory->points[1].velocities_rad_s[0],0.01);EXPECT_EQ(r.command->trajectory->points[1].accelerations_rad_s2[0],0.0025);
}
TEST_F(Dispatch, InvalidRetimingRejected) {
  DispatchAdapter a(cfg,"session");EXPECT_FALSE(a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100,0,0.5).command);
  EXPECT_FALSE(a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100,0,5).command);
}
TEST_F(Dispatch, RetimingCannotBypassPositionOrSplineLimits) {
  source.trajectory->points[1].positions_rad[0]=1;
  DispatchAdapter a(cfg,"session");EXPECT_FALSE(a.prepare({source,GenerationClock::STEADY,10},DispatchPolicy::IMMEDIATE,state,10.1,100,0,3).command);
}
TEST_F(Dispatch, UnrecognizedPolicyCannotBecomeImplicitImmediate) {
  EXPECT_FALSE(run(static_cast<DispatchPolicy>(42)).command);
}
TEST_F(Dispatch, UnrecognizedClockCannotBecomeRosTime) {
  DispatchAdapter a(cfg,"session");EXPECT_FALSE(a.prepare({source,static_cast<GenerationClock>(42),10},DispatchPolicy::IMMEDIATE,state,10.1,100).command);
}
