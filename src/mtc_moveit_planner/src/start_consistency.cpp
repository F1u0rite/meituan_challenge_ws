// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_moveit_planner/start_consistency.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace mtc
{
namespace moveit_planner
{

StartCheck check_trajectory_start(
  const mtc::core::TimedTrajectory & trajectory,
  const std::vector<std::string> & actual_names,
  const std::vector<double> & actual_positions,
  double tolerance_rad)
{
  StartCheck check;
  if (trajectory.points.empty() ||
    trajectory.points.front().positions_rad.size() != trajectory.joint_names.size())
  {
    check.message = "trajectory has no valid first point";
    return check;
  }
  if (actual_names.size() != actual_positions.size() || !std::isfinite(tolerance_rad) ||
    tolerance_rad < 0.0)
  {
    check.message = "invalid actual state or tolerance";
    return check;
  }
  const auto & first = trajectory.points.front().positions_rad;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    size_t index = actual_names.size();
    for (size_t j = 0; j < actual_names.size(); ++j) {
      if (actual_names[j] == trajectory.joint_names[i]) {
        index = j;
        break;
      }
    }
    if (index == actual_names.size()) {
      check.message = "actual state is missing joint " + trajectory.joint_names[i];
      check.worst_joint = trajectory.joint_names[i];
      return check;
    }
    const double deviation = std::fabs(first[i] - actual_positions[index]);
    if (!std::isfinite(deviation)) {
      check.message = "non-finite value for joint " + trajectory.joint_names[i];
      check.worst_joint = trajectory.joint_names[i];
      return check;
    }
    if (deviation > check.max_deviation_rad || check.worst_joint.empty()) {
      check.max_deviation_rad = deviation;
      check.worst_joint = trajectory.joint_names[i];
    }
  }
  check.ok = check.max_deviation_rad <= tolerance_rad;
  std::ostringstream os;
  os << std::setprecision(6) << "max start deviation " << check.max_deviation_rad << " rad at " <<
    check.worst_joint << (check.ok ? " <= " : " > ") << tolerance_rad << " rad";
  check.message = os.str();
  return check;
}

}  // namespace moveit_planner
}  // namespace mtc
