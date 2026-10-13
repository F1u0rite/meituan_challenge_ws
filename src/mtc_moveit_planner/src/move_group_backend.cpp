// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/move_group_backend.hpp"
#include "mtc_moveit_planner/scene_ids.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>

#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif
#if __has_include(<moveit/robot_state/robot_state.hpp>)
#include <moveit/robot_state/robot_state.hpp>
#else
#include <moveit/robot_state/robot_state.h>
#endif
#if __has_include(<moveit/robot_state/conversions.hpp>)
#include <moveit/robot_state/conversions.hpp>
#else
#include <moveit/robot_state/conversions.h>
#endif
#if __has_include(<moveit/robot_trajectory/robot_trajectory.hpp>)
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#else
#include <moveit/robot_trajectory/robot_trajectory.h>
#endif
#if __has_include(<moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>)
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#else
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#endif
#if __has_include(<moveit/utils/moveit_error_code.hpp>)
#include <moveit/utils/moveit_error_code.hpp>
#else
#include <moveit/utils/moveit_error_code.h>
#endif
#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#endif
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mtc_moveit_planner/pose_math.hpp"

namespace mtc
{
namespace moveit_planner
{

namespace
{

mtc::core::Pose6D to_pose6d(const Eigen::Isometry3d & transform, const std::string & frame)
{
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  mtc::core::Pose6D pose;
  pose.position = mtc::core::Vector3{
    transform.translation().x(), transform.translation().y(), transform.translation().z()};
  pose.orientation = mtc::core::Quaternion{q.x(), q.y(), q.z(), q.w()};
  pose.frame_id = frame;
  pose.stamp_s = 0.0;
  pose.valid = true;
  return pose;
}

Eigen::Isometry3d to_eigen(const Transform & t)
{
  const auto q = normalized(t.rotation);
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  out.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return out;
}

geometry_msgs::msg::Pose to_msg(const mtc::core::Pose6D & pose)
{
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.position.x;
  msg.position.y = pose.position.y;
  msg.position.z = pose.position.z;
  const auto q = normalized(pose.orientation);
  msg.orientation.x = q.x;
  msg.orientation.y = q.y;
  msg.orientation.z = q.z;
  msg.orientation.w = q.w;
  return msg;
}

}  // namespace

struct MoveGroupBackend::Impl
{
  std::shared_ptr<rclcpp::Node> node;
  MoveGroupBackendOptions options;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  moveit::core::RobotModelConstPtr model;
  const moveit::core::JointModelGroup * group = nullptr;
  std::string planning_frame;
  std::vector<std::string> joint_names;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client;

  struct Timing
  {
    double factor = 1.0;        ///< 为满足末端线/角速度上限的均匀时间拉伸系数
    double max_linear = 0.0;    ///< 拉伸前采样到的末端最大线速度
    double max_angular = 0.0;
    double duration = 0.0;      ///< 拉伸后的时长
  };

  /// 已经 TOTG 的轨迹：按固定间隔采样末端线/角速度（含路点间插值），算出拉伸系数。
  Timing time_scale(
    const robot_trajectory::RobotTrajectory & trajectory, const BackendRequest & request,
    const moveit::core::RobotState & start_state, double sample_dt_s) const
  {
    Timing timing;
    const double duration = trajectory.getDuration();
    const double dt = std::max(1e-3, sample_dt_s);
    auto sample = std::make_shared<moveit::core::RobotState>(start_state);
    std::optional<mtc::core::Pose6D> previous;
    for (double t = 0.0; ; t = std::min(duration, t + dt)) {
      trajectory.getStateAtDurationFromStart(t, sample);
      sample->update();
      const auto pose = to_pose6d(
        sample->getGlobalLinkTransform(options.end_effector_link), planning_frame);
      if (previous) {
        timing.max_linear =
          std::max(timing.max_linear, distance_between(pose.position, previous->position) / dt);
        timing.max_angular = std::max(
          timing.max_angular, angle_between(pose.orientation, previous->orientation) / dt);
      }
      previous = pose;
      if (t >= duration) {
        break;
      }
    }
    if (request.max_linear_speed_m_s > 0.0) {
      timing.factor = std::max(timing.factor, timing.max_linear / request.max_linear_speed_m_s);
    }
    if (request.max_angular_speed_rad_s > 0.0) {
      timing.factor =
        std::max(timing.factor, timing.max_angular / request.max_angular_speed_rad_s);
    }
    if (timing.factor > 1.0) {
      timing.factor *= 1.02;  // 采样误差余量
    }
    timing.duration = duration * timing.factor;
    return timing;
  }

  /// 关节空间直线插值（不做碰撞校验）在 TOTG + 末端限速下的时长，用于给 IK 候选排序。
  /// 只比较相对大小，因此用粗步长（0.1 rad）与粗采样（0.05 s）；最终轨迹仍按细粒度计算。
  double estimate_interpolated_duration(
    const moveit::core::RobotState & start_state, const std::vector<double> & goal,
    const BackendRequest & request) const
  {
    constexpr double max_step_rad = 0.1;
    constexpr double sample_dt_s = 0.05;
    std::vector<double> from;
    start_state.copyJointGroupPositions(group, from);
    double span = 0.0;
    for (size_t i = 0; i < from.size(); ++i) {
      span = std::max(span, std::fabs(goal[i] - from[i]));
    }
    const int steps = std::max(2, static_cast<int>(std::ceil(span / max_step_rad)));
    robot_trajectory::RobotTrajectory trajectory(model, group);
    moveit::core::RobotState state(start_state);
    std::vector<double> q(from.size());
    for (int k = 0; k <= steps; ++k) {
      const double s = static_cast<double>(k) / steps;
      for (size_t i = 0; i < q.size(); ++i) {
        q[i] = from[i] + (goal[i] - from[i]) * s;
      }
      state.setJointGroupPositions(group, q);
      state.update();
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(trajectory, request.velocity_scaling, request.acceleration_scaling)) {
      return std::numeric_limits<double>::infinity();
    }
    return time_scale(trajectory, request, start_state, sample_dt_s).duration;
  }

  /// 关节空间直线插值（起点 → goal），逐点经 /check_state_validity 校验（含附着载荷与路径约束）。
  /// 全部有效则写入 out 并返回 true；why 记录第一处无效原因。
  bool joint_interpolated_path(
    const moveit::core::RobotState & start_state, const std::vector<double> & goal,
    const moveit_msgs::msg::Constraints & constraints, double max_step_rad,
    moveit_msgs::msg::RobotTrajectory & out, std::string & why) const
  {
    std::vector<double> from;
    start_state.copyJointGroupPositions(group, from);
    if (from.size() != goal.size()) {
      why = "joint dimension mismatch";
      return false;
    }
    double span = 0.0;
    for (size_t i = 0; i < from.size(); ++i) {
      span = std::max(span, std::fabs(goal[i] - from[i]));
    }
    const int steps = std::max(2, static_cast<int>(std::ceil(span / max_step_rad)));
    if (!validity_client->wait_for_service(std::chrono::seconds(2))) {
      why = "/check_state_validity unavailable";
      return false;
    }
    robot_trajectory::RobotTrajectory trajectory(model, group);
    moveit::core::RobotState state(start_state);
    std::vector<double> q(from.size());
    for (int k = 0; k <= steps; ++k) {
      const double s = static_cast<double>(k) / steps;
      for (size_t i = 0; i < q.size(); ++i) {
        q[i] = from[i] + (goal[i] - from[i]) * s;
      }
      state.setJointGroupPositions(group, q);
      state.update();
      auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      moveit::core::robotStateToRobotStateMsg(state, request->robot_state, true);
      request->group_name = group->getName();
      request->constraints = constraints;
      auto future = validity_client->async_send_request(request);
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        why = "/check_state_validity timed out";
        return false;
      }
      const auto response = future.get();
      if (!response->valid) {
        why = "joint-interpolated state " + std::to_string(k) + "/" + std::to_string(steps) +
          " invalid (" + std::to_string(response->contacts.size()) + " contact(s))";
        return false;
      }
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    trajectory.getRobotTrajectoryMsg(out);
    return true;
  }
  std::vector<std::string> scene_ids;   ///< 本后端写入的静态障碍物

  moveit::core::RobotStatePtr state_from(const JointStateSnapshot & snapshot) const
  {
    if (!snapshot.shape_is_valid()) {
      return nullptr;
    }
    auto state = std::make_shared<moveit::core::RobotState>(model);
    state->setToDefaultValues();
    for (size_t i = 0; i < snapshot.names.size(); ++i) {
      if (!model->hasJointModel(snapshot.names[i]) || !std::isfinite(snapshot.positions[i])) {
        return nullptr;
      }
      state->setVariablePosition(snapshot.names[i], snapshot.positions[i]);
    }
    state->update();
    return state;
  }

  JointStateSnapshot snapshot_from(const moveit::core::RobotState & state) const
  {
    JointStateSnapshot snapshot;
    snapshot.names = joint_names;
    state.copyJointGroupPositions(group, snapshot.positions);
    return snapshot;
  }
};

MoveGroupBackend::MoveGroupBackend(
  std::shared_ptr<rclcpp::Node> node, MoveGroupBackendOptions options)
: impl_(std::make_unique<Impl>())
{
  impl_->node = std::move(node);
  impl_->options = std::move(options);
  impl_->tf_buffer = std::make_shared<tf2_ros::Buffer>(impl_->node->get_clock());
  impl_->tf_listener = std::make_shared<tf2_ros::TransformListener>(*impl_->tf_buffer);
  impl_->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    impl_->node, impl_->options.planning_group);
  impl_->model = impl_->move_group->getRobotModel();
  impl_->group = impl_->model->getJointModelGroup(impl_->options.planning_group);
  if (impl_->group == nullptr) {
    throw std::runtime_error("planning group not found: " + impl_->options.planning_group);
  }
  if (!impl_->model->hasLinkModel(impl_->options.end_effector_link)) {
    throw std::runtime_error("end-effector link not found: " + impl_->options.end_effector_link);
  }
  impl_->planning_frame = impl_->move_group->getPlanningFrame();
  impl_->joint_names = impl_->group->getActiveJointModelNames();
  impl_->move_group->setPlanningPipelineId(impl_->options.planning_pipeline);
  impl_->move_group->setPlannerId(impl_->options.planner_id);
  impl_->move_group->setEndEffectorLink(impl_->options.end_effector_link);
  impl_->move_group->setNumPlanningAttempts(1);
  impl_->validity_client = impl_->node->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
}

MoveGroupBackend::~MoveGroupBackend() = default;

std::string MoveGroupBackend::planning_frame() const {return impl_->planning_frame;}

std::string MoveGroupBackend::end_effector_link() const {return impl_->options.end_effector_link;}

std::optional<JointStateSnapshot> MoveGroupBackend::current_state()
{
  const auto state = impl_->move_group->getCurrentState(impl_->options.state_timeout_s);
  if (!state) {
    return std::nullopt;
  }
  return impl_->snapshot_from(*state);
}

std::optional<JointStateSnapshot> MoveGroupBackend::named_state(const std::string & name)
{
  auto state = std::make_shared<moveit::core::RobotState>(impl_->model);
  state->setToDefaultValues();
  if (!state->setToDefaultValues(impl_->group, name)) {
    return std::nullopt;
  }
  state->update();
  return impl_->snapshot_from(*state);
}

std::optional<mtc::core::Pose6D> MoveGroupBackend::end_effector_pose(
  const JointStateSnapshot & snapshot)
{
  const auto state = impl_->state_from(snapshot);
  if (!state) {
    return std::nullopt;
  }
  return to_pose6d(
    state->getGlobalLinkTransform(impl_->options.end_effector_link), impl_->planning_frame);
}

std::optional<mtc::core::Pose6D> MoveGroupBackend::transform_to_planning_frame(
  const mtc::core::Pose6D & pose)
{
  if (pose.frame_id.empty()) {
    return std::nullopt;
  }
  if (pose.frame_id == impl_->planning_frame) {
    return pose;
  }
  geometry_msgs::msg::PoseStamped in;
  in.header.frame_id = pose.frame_id;
  // Pose6D::stamp_s 属于 FSM 时基而非 ROS 时间，这里取最新可用变换（静态 TF 不受影响）。
  in.header.stamp = rclcpp::Time(0, 0, impl_->node->get_clock()->get_clock_type());
  in.pose = to_msg(pose);
  try {
    const auto out = impl_->tf_buffer->transform(
      in, impl_->planning_frame, tf2::durationFromSec(impl_->options.tf_timeout_s));
    mtc::core::Pose6D result = pose;
    result.frame_id = impl_->planning_frame;
    result.position = mtc::core::Vector3{
      out.pose.position.x, out.pose.position.y, out.pose.position.z};
    result.orientation = mtc::core::Quaternion{
      out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z,
      out.pose.orientation.w};
    return result;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

BackendResult MoveGroupBackend::plan(const BackendRequest & request)
{
  BackendResult result;
  const auto start = impl_->state_from(request.start);
  if (!start) {
    result.reason = "INVALID_START";
    result.message = "start state has unknown joints or non-finite values";
    return result;
  }
  if (request.payload) {
    // 只附着在本次请求的起点状态上：MoveIt 以起点的 attached bodies 做碰撞检查，全局场景不变。
    const auto & p = *request.payload;
    std::vector<shapes::ShapeConstPtr> shapes{
      std::make_shared<shapes::Box>(p.size_m.x, p.size_m.y, p.size_m.z)};
    EigenSTL::vector_Isometry3d shape_poses{Eigen::Isometry3d::Identity()};
    start->attachBody(p.id, to_eigen(p.ee_to_payload), shapes, shape_poses,
      std::set<std::string>(p.touch_links.begin(), p.touch_links.end()),
      impl_->options.end_effector_link);
    start->update();
  }
  auto goal = transform_to_planning_frame(request.ee_goal);
  if (!goal) {
    result.reason = "TF_UNAVAILABLE";
    result.message = "cannot transform goal from '" + request.ee_goal.frame_id + "'";
    return result;
  }

  auto & mg = *impl_->move_group;
  mg.setStartState(*start);
  mg.setMaxVelocityScalingFactor(request.velocity_scaling);
  mg.setMaxAccelerationScalingFactor(request.acceleration_scaling);
  mg.setPlanningTime(request.planning_time_s);

  // 控制器限位比 URDF 窄的关节：以路径约束下发（起点与目标也必须满足）。
  moveit_msgs::msg::Constraints path_constraints;
  path_constraints.name = "mtc_controller_joint_limits";
  for (const auto & bound : request.joint_bounds) {
    const auto * joint = impl_->model->getJointModel(bound.first);
    if (joint == nullptr || joint->getVariableCount() != 1) {
      continue;
    }
    const auto & urdf = joint->getVariableBounds()[0];
    if (urdf.position_bounded_ && bound.second.first <= urdf.min_position_ &&
      bound.second.second >= urdf.max_position_)
    {
      continue;  // 不比 URDF 更窄，无需约束
    }
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = bound.first;
    jc.position = 0.5 * (bound.second.first + bound.second.second);
    jc.tolerance_above = bound.second.second - jc.position;
    jc.tolerance_below = jc.position - bound.second.first;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);
  }
  if (!path_constraints.joint_constraints.empty()) {
    mg.setPathConstraints(path_constraints);
  } else {
    mg.clearPathConstraints();
  }

  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = impl_->planning_frame;
  goal_msg.pose = to_msg(*goal);

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const auto t0 = std::chrono::steady_clock::now();
  bool planned = false;
  if (request.mode == MotionMode::FREE_SPACE && request.prefer_straight_line) {
    // 先试末端直线：只有完整（fraction=1）、无碰撞、满足关节约束时才采用。
    const std::vector<geometry_msgs::msg::Pose> waypoints{goal_msg.pose};
    moveit_msgs::msg::RobotTrajectory straight;
    const double fraction = mg.computeCartesianPath(
      waypoints, request.cartesian_step_m, straight, path_constraints, request.avoid_collisions);
    // 逐点 IK 可能在相邻路点间翻转构型：任一关节单步 > 0.2 rad（步长 5 mm 时不应出现）即放弃直线。
    double max_step = 0.0;
    const auto & pts = straight.joint_trajectory.points;
    for (size_t i = 1; i < pts.size(); ++i) {
      for (size_t j = 0; j < pts[i].positions.size() && j < pts[i - 1].positions.size(); ++j) {
        max_step = std::max(max_step, std::fabs(pts[i].positions[j] - pts[i - 1].positions[j]));
      }
    }
    if (fraction >= 1.0 - 1e-9 && max_step <= 0.2) {
      trajectory_msg = straight;
      result.cartesian_fraction = fraction;
      result.method = "cartesian_preferred";
      planned = true;
    } else {
      result.message = "straight line rejected (fraction=" + std::to_string(fraction) +
        ", max_joint_step=" + std::to_string(max_step) + " rad); ";
    }
    mg.setStartState(*start);
  }
  if (planned) {
    mg.clearPathConstraints();
    result.planning_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  } else if (request.mode == MotionMode::FREE_SPACE) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    // 选关节目标：KDL 在种子不收敛时会随机重启，单次 setFromIK 可能落到另一组解（V1 实测
    // 同一请求 39.6 s / 123 s 来回跳）。因此收集多组候选，按"实际关心的量"排序：
    //   1) 位姿延拓：起点末端位姿 → 目标位姿分 ik_continuation_steps 步，每步以上一步解为种子；
    //   2) 固定种子网格：起点 + 腕关节 ±90° 组合，共 27 个种子，顺序固定；
    // 候选按"关节插值在 TOTG + 末端限速下的预计时长"从短到长排序（关节跨度最小 ≠ 时长最短：
    // V1 中跨度 3.45 rad 的解要 123 s，2.79 rad 的解 39.6 s），再逐个做逐点碰撞校验。
    // 求解时即要求满足控制器限位（路径约束中的关节约束）。
    // 回调收到的是候选 IK 解（按 group 变量顺序），此时 state 尚未写入该解。
    const auto within_bounds = [&path_constraints](moveit::core::RobotState *,
        const moveit::core::JointModelGroup * group, const double * values) {
        const auto & names = group->getVariableNames();
        for (const auto & jc : path_constraints.joint_constraints) {
          const auto it = std::find(names.begin(), names.end(), jc.joint_name);
          if (it == names.end()) {
            continue;
          }
          const double v = values[it - names.begin()];
          if (v > jc.position + jc.tolerance_above || v < jc.position - jc.tolerance_below) {
            return false;
          }
        }
        return true;
      };
    std::vector<double> from;
    start->copyJointGroupPositions(impl_->group, from);
    struct Candidate
    {
      std::vector<double> q;
      double span;
      double est_duration;
      std::string source;
    };
    std::vector<Candidate> candidates;
    const auto t_ik = std::chrono::steady_clock::now();
    double est_s = 0.0;
    const auto add_candidate = [&](const moveit::core::RobotState & state, const std::string & src) {
        std::vector<double> q;
        state.copyJointGroupPositions(impl_->group, q);
        for (const auto & c : candidates) {
          double d = 0.0;
          for (size_t i = 0; i < q.size(); ++i) {
            d = std::max(d, std::fabs(q[i] - c.q[i]));
          }
          if (d < 1e-3) {
            return;
          }
        }
        double span = 0.0;
        for (size_t i = 0; i < q.size(); ++i) {
          span = std::max(span, std::fabs(q[i] - from[i]));
        }
        const auto t_est = std::chrono::steady_clock::now();
        candidates.push_back(Candidate{q, span,
            impl_->estimate_interpolated_duration(*start, q, request), src});
        est_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - t_est).count();
      };
    {
      const auto & gp = goal_msg.pose;
      Eigen::Isometry3d to_pose = Eigen::Isometry3d::Identity();
      to_pose.translation() = Eigen::Vector3d(gp.position.x, gp.position.y, gp.position.z);
      to_pose.linear() = Eigen::Quaterniond(
        gp.orientation.w, gp.orientation.x, gp.orientation.y, gp.orientation.z)
        .normalized().toRotationMatrix();
      const Eigen::Isometry3d from_pose =
        start->getGlobalLinkTransform(impl_->options.end_effector_link);
      const Eigen::Quaterniond q0(from_pose.rotation());
      const Eigen::Quaterniond q1(to_pose.rotation());
      const int n = std::max(1, impl_->options.ik_continuation_steps);
      moveit::core::RobotState walker(*start);
      bool tracked = true;
      for (int k = 1; k <= n && tracked; ++k) {
        const double s = static_cast<double>(k) / n;
        Eigen::Isometry3d waypoint = Eigen::Isometry3d::Identity();
        waypoint.translation() = (1.0 - s) * from_pose.translation() + s * to_pose.translation();
        waypoint.linear() = q0.slerp(s, q1).toRotationMatrix();
        tracked = walker.setFromIK(impl_->group, waypoint, impl_->options.end_effector_link,
            impl_->options.ik_timeout_s, within_bounds);
      }
      if (tracked) {
        add_candidate(walker, "continuation");
      }
    }
    // 固定种子网格：起点 + 三个腕关节各取 {0, -π/2, +π/2} 偏移（共 27 个，顺序固定）。
    {
      const auto & names = impl_->group->getVariableNames();
      std::vector<size_t> wrists;
      for (const char * w : {"wrist1_joint", "wrist2_joint", "wrist3_joint"}) {
        const auto it = std::find(names.begin(), names.end(), w);
        if (it != names.end()) {
          wrists.push_back(static_cast<size_t>(it - names.begin()));
        }
      }
      const double offsets[] = {0.0, -M_PI / 2.0, M_PI / 2.0};
      const size_t combos = static_cast<size_t>(std::pow(3.0, wrists.size()));
      for (size_t c = 0; c < combos; ++c) {
        std::vector<double> seed = from;
        size_t code_digits = c;
        for (const size_t w : wrists) {
          seed[w] += offsets[code_digits % 3];
          code_digits /= 3;
        }
        moveit::core::RobotState seeded(*start);
        seeded.setJointGroupPositions(impl_->group, seed);
        seeded.enforceBounds(impl_->group);
        seeded.update();
        if (seeded.setFromIK(impl_->group, goal_msg.pose, impl_->options.end_effector_link,
            impl_->options.ik_seed_timeout_s, within_bounds))
        {
          add_candidate(seeded, c == 0 ? "seed_start" : "seed_grid");
        }
      }
    }
    // 按"关节插值在 TOTG + 末端限速下的预计时长"排序（这才是看门狗关心的量），平局按跨度。
    std::stable_sort(candidates.begin(), candidates.end(),
      [](const Candidate & l, const Candidate & r) {
        return l.est_duration != r.est_duration ? l.est_duration < r.est_duration : l.span < r.span;
      });
    if (!candidates.empty()) {
      std::ostringstream os;
      os << std::setprecision(4) << "ik candidates=" << candidates.size() << " est_s/span_rad=[";
      for (size_t i = 0; i < candidates.size() && i < 6; ++i) {
        os << (i ? ", " : "") << candidates[i].est_duration << "/" << candidates[i].span << "(" <<
          candidates[i].source << ")";
      }
      os << (candidates.size() > 6 ? ", ...]; " : "]; ");
      os << "ik+rank " << std::chrono::duration<double>(std::chrono::steady_clock::now() - t_ik).count() <<
        " s (rank " << est_s << " s); ";
      result.message += os.str();
      // 确定性优先：按跨度从小到大，关节空间直线插值，逐点校验碰撞/约束；都不行再交给 OMPL。
      for (const auto & candidate : candidates) {
        std::string why;
        if (impl_->joint_interpolated_path(
            *start, candidate.q, path_constraints, 0.02, plan.trajectory, why))
        {
          code = moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
          result.method = "joint_interpolated_ik";
          std::ostringstream chosen;
          chosen << std::setprecision(4) << "chosen est " << candidate.est_duration << " s, span " <<
            candidate.span << " rad (" << candidate.source << "); ";
          result.message += chosen.str();
          break;
        }
        result.message += "joint interpolation rejected (" + why + "); ";
      }
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        mg.setStartState(*start);
        mg.setJointValueTarget(impl_->joint_names, candidates.front().q);
        code = mg.plan(plan);
        result.method = "ompl_seeded_ik";
        if (code != moveit::core::MoveItErrorCode::SUCCESS) {
          result.message += "OMPL to seeded-IK joint goal returned " +
            moveit::core::errorCodeToString(code) + "; ";
        }
      }
    } else {
      result.message += "seeded IK found no solution within controller limits; ";
    }
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      result.method = "ompl";
      mg.setStartState(*start);
      mg.clearPoseTargets();
      mg.setPoseTarget(goal_msg, impl_->options.end_effector_link);
      code = mg.plan(plan);
    }
    mg.clearPoseTargets();
    mg.clearPathConstraints();
    result.planning_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      result.reason = "NO_PLAN";
      result.message += "MoveIt plan() returned " + moveit::core::errorCodeToString(code);
      return result;
    }
    trajectory_msg = plan.trajectory;
  } else {
    result.method = "cartesian";
    const std::vector<geometry_msgs::msg::Pose> waypoints{goal_msg.pose};
    const double fraction = mg.computeCartesianPath(
      waypoints, request.cartesian_step_m, trajectory_msg, path_constraints,
      request.avoid_collisions);
    mg.clearPathConstraints();
    result.planning_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    result.cartesian_fraction = fraction;
    if (!(fraction >= request.min_cartesian_fraction - 1e-9)) {
      result.reason = "CARTESIAN_INCOMPLETE";
      result.message = "computeCartesianPath achieved fraction " + std::to_string(fraction) +
        " < required " + std::to_string(request.min_cartesian_fraction);
      return result;
    }
  }

  robot_trajectory::RobotTrajectory trajectory(impl_->model, impl_->options.planning_group);
  trajectory.setRobotTrajectoryMsg(*start, trajectory_msg);
  if (trajectory.getWayPointCount() < 2) {
    result.reason = "DEGENERATE_TRAJECTORY";
    result.message = "planner returned fewer than 2 waypoints";
    return result;
  }
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(trajectory, request.velocity_scaling, request.acceleration_scaling)) {
    result.reason = "TIME_PARAMETERIZATION_FAILED";
    result.message = "TOTG could not time-parameterize the path";
    return result;
  }

  const auto timing = impl_->time_scale(trajectory, request, *start, impl_->options.speed_sample_dt_s);
  const double factor = timing.factor;
  const double max_linear = timing.max_linear;
  const double max_angular = timing.max_angular;
  result.time_scale_applied = factor;
  result.max_linear_speed_m_s = max_linear / factor;
  result.max_angular_speed_rad_s = max_angular / factor;

  moveit_msgs::msg::RobotTrajectory out;
  trajectory.getRobotTrajectoryMsg(out);
  const auto & jt = out.joint_trajectory;
  result.trajectory.joint_names = jt.joint_names;
  for (const auto & point : jt.points) {
    RawTrajectoryPoint raw;
    raw.time_from_start_s = rclcpp::Duration(point.time_from_start).seconds() * factor;
    raw.positions = point.positions;
    raw.velocities = point.velocities;
    raw.accelerations = point.accelerations;
    for (auto & v : raw.velocities) {
      v /= factor;
    }
    for (auto & a : raw.accelerations) {
      a /= factor * factor;
    }
    result.trajectory.points.push_back(std::move(raw));
  }
  result.end_state = impl_->snapshot_from(trajectory.getLastWayPoint());
  result.ok = true;
  result.message += "ok";
  return result;
}

bool MoveGroupBackend::apply_scene(const std::vector<SceneBox> & boxes)
{
  // ID 隔离：只增删带本模块前缀的对象，绝不碰其他模块写入的碰撞体（见 scene_ids.hpp）。
  std::vector<std::string> local_ids;
  for (const auto & box : boxes) {
    local_ids.push_back(box.id);
  }
  const std::string id_error = validate_local_scene_ids(local_ids);
  if (!id_error.empty()) {
    RCLCPP_ERROR(impl_->node->get_logger(), "apply_scene: %s", id_error.c_str());
    return false;
  }
  if (!impl_->scene) {
    impl_->scene = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::vector<std::string> ids;
  for (const auto & box : boxes) {
    if (!box.pose.is_usable() || !(box.size_m.x > 0.0) || !(box.size_m.y > 0.0) ||
      !(box.size_m.z > 0.0))
    {
      return false;
    }
    const auto pose = transform_to_planning_frame(box.pose);
    if (!pose) {
      return false;
    }
    moveit_msgs::msg::CollisionObject object;
    object.id = scoped_scene_id(box.id);
    object.header.frame_id = impl_->planning_frame;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box.size_m.x, box.size_m.y, box.size_m.z};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(to_msg(*pose));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    objects.push_back(object);
    ids.push_back(object.id);
  }
  // 需要移除的：场景里本模块的遗留对象（含进程重启前写入的）+ 本实例上次写入但这次不要的。
  std::vector<std::string> known = impl_->scene->getKnownObjectNames();
  known.insert(known.end(), impl_->scene_ids.begin(), impl_->scene_ids.end());
  for (const auto & id : stale_owned_scene_ids(known, ids)) {
    moveit_msgs::msg::CollisionObject remove;
    remove.id = id;
    remove.header.frame_id = impl_->planning_frame;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    objects.push_back(remove);
  }
  if (objects.empty()) {
    impl_->scene_ids.clear();
    return true;
  }
  if (!impl_->scene->applyCollisionObjects(objects)) {
    return false;
  }
  impl_->scene_ids = ids;
  return true;
}

}  // namespace moveit_planner
}  // namespace mtc
