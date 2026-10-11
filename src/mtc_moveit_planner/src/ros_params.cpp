// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/ros_params.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace mtc
{
namespace moveit_planner
{

namespace
{

CalibrationStatus status_param(
  rclcpp::Node & node, const std::string & name, CalibrationStatus fallback)
{
  const std::string text = node.get_parameter_or<std::string>(name, to_string(fallback));
  const auto status = calibration_status_from_string(text);
  if (!status) {
    throw std::runtime_error("parameter " + name + " must be MEASURED/ESTIMATED/PLACEHOLDER/UNKNOWN");
  }
  return *status;
}

std::vector<double> vector_param(
  rclcpp::Node & node, const std::string & name, const std::vector<double> & fallback,
  size_t expected_size)
{
  const auto values = node.get_parameter_or<std::vector<double>>(name, fallback);
  if (values.size() != expected_size) {
    throw std::runtime_error(
      "parameter " + name + " must have " + std::to_string(expected_size) + " elements");
  }
  return values;
}

}  // namespace

PlannerConfig load_planner_config(rclcpp::Node & node)
{
  PlannerConfig config = default_planner_config();
  config.velocity_scaling = node.get_parameter_or("planner.velocity_scaling", config.velocity_scaling);
  config.acceleration_scaling =
    node.get_parameter_or("planner.acceleration_scaling", config.acceleration_scaling);
  config.planning_time_s = node.get_parameter_or("planner.planning_time_s", config.planning_time_s);
  config.cartesian_step_m = node.get_parameter_or("planner.cartesian_step_m", config.cartesian_step_m);
  config.insertion_axis = static_cast<int>(
    node.get_parameter_or<int64_t>("planner.insertion_axis", config.insertion_axis));
  config.allow_uncalibrated_geometry = node.get_parameter_or(
    "planner.allow_uncalibrated_geometry", config.allow_uncalibrated_geometry);
  config.force_simulation_only =
    node.get_parameter_or("planner.force_simulation_only", config.force_simulation_only);

  const auto tcp_xyz = vector_param(node, "tcp.xyz", {0.0, 0.0, 0.0}, 3);
  const auto tcp_q = vector_param(node, "tcp.quat_xyzw", {0.0, 0.0, 0.0, 1.0}, 4);
  config.ee_to_tcp.translation = mtc::core::Vector3{tcp_xyz[0], tcp_xyz[1], tcp_xyz[2]};
  config.ee_to_tcp.rotation = normalized(mtc::core::Quaternion{tcp_q[0], tcp_q[1], tcp_q[2], tcp_q[3]});
  config.tcp_status = status_param(node, "tcp.status", config.tcp_status);

  std::vector<std::string> slot_names;
  for (const auto & entry : config.slots) {
    slot_names.push_back(entry.first);
  }
  slot_names = node.get_parameter_or<std::vector<std::string>>("slots.names", slot_names);
  for (const auto & name : slot_names) {
    SlotSpec slot = config.slots.count(name) ? config.slots[name] : SlotSpec{};
    const std::vector<double> fallback{slot.pose.position.x, slot.pose.position.y, slot.pose.position.z};
    const auto xyz = vector_param(node, "slots." + name + ".xyz", fallback, 3);
    slot.pose.position = mtc::core::Vector3{xyz[0], xyz[1], xyz[2]};
    slot.pose.frame_id = node.get_parameter_or<std::string>(
      "slots." + name + ".frame_id", slot.pose.frame_id.empty() ? "base_link" : slot.pose.frame_id);
    slot.pose.orientation = mtc::core::Quaternion{0.0, 0.0, 0.0, 1.0};
    slot.pose.valid = true;
    slot.status = status_param(node, "slots." + name + ".status", slot.status);
    config.slots[name] = slot;
  }
  return config;
}

MoveGroupBackendOptions load_backend_options(rclcpp::Node & node)
{
  MoveGroupBackendOptions options;
  options.planning_group = node.get_parameter_or("backend.planning_group", options.planning_group);
  options.end_effector_link =
    node.get_parameter_or("backend.end_effector_link", options.end_effector_link);
  options.planning_pipeline =
    node.get_parameter_or("backend.planning_pipeline", options.planning_pipeline);
  options.planner_id = node.get_parameter_or("backend.planner_id", options.planner_id);
  options.tf_timeout_s = node.get_parameter_or("backend.tf_timeout_s", options.tf_timeout_s);
  options.state_timeout_s = node.get_parameter_or("backend.state_timeout_s", options.state_timeout_s);
  return options;
}

}  // namespace moveit_planner
}  // namespace mtc
