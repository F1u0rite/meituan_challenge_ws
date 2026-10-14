#pragma once
#include <mtc_trajectory_execution_cpp/fjt_motion_port.hpp>
#include <mtc_moveit_planner/trajectory_conversion.hpp>

namespace mtc::aubo_fake {
// Source-compatible MotionPort injection into PR14's FSM host. Exactly one
// underlying FJT client; Gazebo Runner and its identity checks are not modified.
class AuboFjtMotionPort final : public mtc::manipulation::MotionPort {
public:
  AuboFjtMotionPort(rclcpp::Node::SharedPtr node, mtc::trajectory::Config config,
    const std::shared_ptr<mtc::trajectory::ControlledFakeStart> & credential,
    std::string planning_frame);
  std::optional<mtc::core::MotionResult> execute_trajectory(const mtc::core::MotionCommand & command) override;
  std::optional<mtc::core::MotionResult> execute_joint_move(const mtc::core::MotionCommand & command) override;
  mtc::core::StopResult request_stop(const std::string & reason) override;
  mtc::core::StopResult cancel_command(const std::string & id, const std::string & reason);
  mtc::trajectory::ExecutionSnapshot snapshot() const;
private:
  std::string planning_frame_;
  mtc::trajectory::FjtMotionPort port_;
};
// Explicit immediate dispatch copy from this run's freshly planned RawTrajectory.
// No imported PR14 trajectories or initial joint values are accepted by the probe.
mtc::core::MotionCommand command_from_plan(const mtc::moveit_planner::RawTrajectory & raw,
  const mtc::trajectory::Config & config, const std::string & frame,
  const std::string & command_id, double uniform_slowdown = 1.0);
bool plan_is_fresh(double generated_monotonic_s, double now_monotonic_s, double maximum_age_s=1.0);
}
