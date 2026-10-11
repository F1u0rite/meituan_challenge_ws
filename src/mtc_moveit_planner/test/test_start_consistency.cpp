// Copyright 2026 meituan_challenge_ws contributors

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "mtc_moveit_planner/start_consistency.hpp"

namespace
{

using mtc::moveit_planner::check_trajectory_start;

mtc::core::TimedTrajectory trajectory()
{
  mtc::core::TimedTrajectory t;
  t.joint_names = {"shoulder_joint", "upperArm_joint", "foreArm_joint"};
  t.header_frame_id = "world";
  t.points = {{0.0, {0.1, 0.2, 0.3}, {}, {}}, {1.0, {0.2, 0.3, 0.4}, {}, {}}};
  return t;
}

TEST(StartConsistency, MatchesByNameNotIndex)
{
  const auto check = check_trajectory_start(
    trajectory(), {"foreArm_joint", "shoulder_joint", "upperArm_joint", "wrist1_joint"},
    {0.305, 0.1, 0.2, 9.0});
  EXPECT_TRUE(check.ok) << check.message;
  EXPECT_NEAR(check.max_deviation_rad, 0.005, 1e-12);
  EXPECT_EQ(check.worst_joint, "foreArm_joint");
}

TEST(StartConsistency, DeviationAboveToleranceIsRejected)
{
  const auto check = check_trajectory_start(
    trajectory(), {"shoulder_joint", "upperArm_joint", "foreArm_joint"}, {0.1, 0.2, 0.33});
  EXPECT_FALSE(check.ok);
  EXPECT_EQ(check.worst_joint, "foreArm_joint");
  EXPECT_NE(check.message.find("> 0.01"), std::string::npos) << check.message;
}

TEST(StartConsistency, MissingJointNanAndEmptyTrajectoryAreRejected)
{
  EXPECT_FALSE(check_trajectory_start(
      trajectory(), {"shoulder_joint", "upperArm_joint"}, {0.1, 0.2}).ok);
  EXPECT_FALSE(check_trajectory_start(
      trajectory(), {"shoulder_joint", "upperArm_joint", "foreArm_joint"},
      {0.1, std::numeric_limits<double>::quiet_NaN(), 0.3}).ok);
  mtc::core::TimedTrajectory empty;
  EXPECT_FALSE(check_trajectory_start(empty, {"shoulder_joint"}, {0.0}).ok);
  EXPECT_FALSE(check_trajectory_start(
      trajectory(), {"shoulder_joint", "upperArm_joint", "foreArm_joint"}, {0.1, 0.2, 0.3},
      -1.0).ok);
}

}  // namespace
