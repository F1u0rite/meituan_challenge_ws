// Copyright 2026 meituan_challenge_ws contributors
//
// PlanningBackend 的 MoveIt 2 实现（ROS 外壳，依赖 move_group 正在运行）。
//
//   * FREE_SPACE：MoveGroupInterface::plan()（OMPL，默认 RRTConnect）到 ee_link 目标位姿；
//   * CARTESIAN ：MoveGroupInterface::computeCartesianPath()，要求完整（fraction >= 阈值）；
//   * 两种结果都用 TOTG 按 velocity/acceleration scaling 重新时间参数化，
//     再按原语的末端线速度/角速度上限做**均匀时间拉伸**（路径不变）；
//   * 只规划、不执行：本类不调用 execute()/move()，不连接控制器 Action。

#ifndef MTC_MOVEIT_PLANNER__MOVE_GROUP_BACKEND_HPP_
#define MTC_MOVEIT_PLANNER__MOVE_GROUP_BACKEND_HPP_

#include <memory>
#include <optional>
#include <string>

#include "mtc_moveit_planner/planning_backend.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace mtc
{
namespace moveit_planner
{

struct MoveGroupBackendOptions
{
  std::string planning_group = "manipulator";
  std::string end_effector_link = "ee_link";
  std::string planning_pipeline = "ompl";
  std::string planner_id = "RRTConnectkConfigDefault";
  double tf_timeout_s = 0.5;
  double state_timeout_s = 5.0;
  double speed_sample_dt_s = 0.01;   ///< 末端速度检查的采样间隔
};

class MoveGroupBackend : public PlanningBackend
{
public:
  MoveGroupBackend(std::shared_ptr<rclcpp::Node> node, MoveGroupBackendOptions options);
  ~MoveGroupBackend() override;

  std::string planning_frame() const override;
  std::string end_effector_link() const override;
  std::optional<JointStateSnapshot> current_state() override;
  std::optional<mtc::core::Pose6D> end_effector_pose(const JointStateSnapshot & state) override;
  std::optional<mtc::core::Pose6D> transform_to_planning_frame(
    const mtc::core::Pose6D & pose) override;
  BackendResult plan(const BackendRequest & request) override;

  /// SRDF 命名状态（如 "home"）对应的关节值；不存在返回 nullopt。
  std::optional<JointStateSnapshot> named_state(const std::string & name);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__MOVE_GROUP_BACKEND_HPP_
