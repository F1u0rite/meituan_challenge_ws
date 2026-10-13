#include <gtest/gtest.h>
#include "mtc_phase2c_integration_cpp/planning_only_manager.hpp"
TEST(PlanningOnly, NoExecutionHandlesOrControllers) {
  mtc::integration::PlanningOnlyManager manager;
  EXPECT_FALSE(manager.getControllerHandle("joint_trajectory_controller"));
  std::vector<std::string> names={"stale"};manager.getControllersList(names);EXPECT_TRUE(names.empty());
  names={"stale"};manager.getActiveControllers(names);EXPECT_TRUE(names.empty());
  names={"stale"};manager.getControllerJoints("jtc",names);EXPECT_TRUE(names.empty());
  EXPECT_FALSE(manager.getControllerState("jtc").active_);
  EXPECT_FALSE(manager.switchControllers({"jtc"},{}));
}
