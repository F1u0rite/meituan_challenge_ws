#pragma once
#include <mtc_trajectory_execution_cpp/trajectory_validator.hpp>
namespace mtc::integration {
enum class GenerationClock {STEADY, ROS, UNKNOWN};
enum class DispatchPolicy {UNSPECIFIED, IMMEDIATE, ROS_SCHEDULED};
struct PlanningRecord {
  const mtc::core::MotionCommand original;
  const GenerationClock clock;
  const double generated_s;
};
struct DispatchResult {
  std::optional<mtc::core::MotionCommand> command;
  std::string error;
};
class DispatchAdapter {
public:
  DispatchAdapter(mtc::trajectory::Config config, std::string session, double max_plan_age=2.0);
  DispatchResult prepare(const PlanningRecord & plan, DispatchPolicy policy,
    const mtc::trajectory::JointSample & current, double steady_now_s,
    double ros_now_s, double scheduled_ros_s=0.0, double uniform_time_factor=1.0);
private:
  mtc::trajectory::Config config_;
  std::string session_;
  double max_age_;
  uint64_t sequence_=0;
};
}  // namespace mtc::integration
