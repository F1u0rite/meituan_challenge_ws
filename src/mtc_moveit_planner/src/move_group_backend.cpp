// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/move_group_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
    // 以起点为种子求 IK：得到离起点最近的关节解，再规划到该关节目标（路径短、可重复）。
    // 求解时即要求满足控制器限位（路径约束中的关节约束）。
    auto seeded = std::make_shared<moveit::core::RobotState>(*start);
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
    if (seeded->setFromIK(impl_->group, goal_msg.pose, impl_->options.end_effector_link,
        impl_->options.ik_timeout_s, within_bounds))
    {
      std::vector<double> joint_goal;
      seeded->copyJointGroupPositions(impl_->group, joint_goal);
      mg.setStartState(*start);
      mg.setJointValueTarget(impl_->joint_names, joint_goal);
      code = mg.plan(plan);
      result.method = "ompl_seeded_ik";
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        result.message += "OMPL to seeded-IK joint goal returned " +
          moveit::core::errorCodeToString(code) + "; ";
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

  // 末端线速度 / 角速度：按固定间隔采样整条轨迹（含路点之间的插值）。
  const double duration = trajectory.getDuration();
  const double dt = std::max(1e-3, impl_->options.speed_sample_dt_s);
  auto sample = std::make_shared<moveit::core::RobotState>(*start);
  double max_linear = 0.0;
  double max_angular = 0.0;
  std::optional<mtc::core::Pose6D> previous;
  for (double t = 0.0; ; t = std::min(duration, t + dt)) {
    trajectory.getStateAtDurationFromStart(t, sample);
    sample->update();
    const auto pose = to_pose6d(
      sample->getGlobalLinkTransform(impl_->options.end_effector_link), impl_->planning_frame);
    if (previous) {
      max_linear = std::max(max_linear, distance_between(pose.position, previous->position) / dt);
      max_angular = std::max(
        max_angular, angle_between(pose.orientation, previous->orientation) / dt);
    }
    previous = pose;
    if (t >= duration) {
      break;
    }
  }
  double factor = 1.0;
  if (request.max_linear_speed_m_s > 0.0) {
    factor = std::max(factor, max_linear / request.max_linear_speed_m_s);
  }
  if (request.max_angular_speed_rad_s > 0.0) {
    factor = std::max(factor, max_angular / request.max_angular_speed_rad_s);
  }
  if (factor > 1.0) {
    factor *= 1.02;  // 采样误差余量
  }
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
  if (!impl_->scene) {
    impl_->scene = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  }
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto & id : impl_->scene_ids) {
    moveit_msgs::msg::CollisionObject remove;
    remove.id = id;
    remove.header.frame_id = impl_->planning_frame;
    remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    objects.push_back(remove);
  }
  std::vector<std::string> ids;
  for (const auto & box : boxes) {
    if (box.id.empty() || !box.pose.is_usable() || !(box.size_m.x > 0.0) ||
      !(box.size_m.y > 0.0) || !(box.size_m.z > 0.0))
    {
      return false;
    }
    const auto pose = transform_to_planning_frame(box.pose);
    if (!pose) {
      return false;
    }
    moveit_msgs::msg::CollisionObject object;
    object.id = box.id;
    object.header.frame_id = impl_->planning_frame;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box.size_m.x, box.size_m.y, box.size_m.z};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(to_msg(*pose));
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    objects.push_back(object);
    ids.push_back(box.id);
  }
  if (objects.empty()) {
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
