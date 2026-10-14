#include "mtc_aubo_fake_integration_cpp/aubo_fjt_motion_port.hpp"
#include <cmath>
#include <stdexcept>
namespace mtc::aubo_fake {
bool plan_is_fresh(double generated,double now,double maximum) {
  return std::isfinite(generated)&&std::isfinite(now)&&std::isfinite(maximum)&&generated>0&&maximum>0&&maximum<=1&&now>=generated&&now-generated<=maximum;
}
static mtc::core::MotionResult reject(const std::string & reason) {
  mtc::core::MotionResult r;r.exec_state=mtc::core::ExecState::REJECTED;
  r.error_code=mtc::core::ErrorCodes::EXECUTION_REJECTED;r.message=reason;return r;
}
AuboFjtMotionPort::AuboFjtMotionPort(rclcpp::Node::SharedPtr node,mtc::trajectory::Config config,
  const std::shared_ptr<mtc::trajectory::ControlledFakeStart> & credential,std::string frame)
: planning_frame_(std::move(frame)),port_(std::move(node),std::move(config),credential) {
  if (!credential || planning_frame_.empty()) {throw std::invalid_argument("controlled Fake credential and planning frame required");}
}
std::optional<mtc::core::MotionResult> AuboFjtMotionPort::execute_trajectory(const mtc::core::MotionCommand & c) {
  if (!c.trajectory || c.trajectory->header_frame_id!=planning_frame_) {return reject("planning frame mismatch or missing complete trajectory");}
  return port_.execute_trajectory(c);
}
std::optional<mtc::core::MotionResult> AuboFjtMotionPort::execute_joint_move(const mtc::core::MotionCommand & c) {return port_.execute_joint_move(c);}
mtc::core::StopResult AuboFjtMotionPort::request_stop(const std::string & r) {return port_.request_stop(r);}
mtc::core::StopResult AuboFjtMotionPort::cancel_command(const std::string & id,const std::string & r) {return port_.cancel_command(id,r);}
mtc::trajectory::ExecutionSnapshot AuboFjtMotionPort::snapshot() const {return port_.snapshot();}
mtc::core::MotionCommand command_from_plan(const mtc::moveit_planner::RawTrajectory & raw,
  const mtc::trajectory::Config & cfg,const std::string & frame,const std::string & id,double slowdown) {
  cfg.validate();
  if (id.empty() || !std::isfinite(slowdown) || slowdown<1 || slowdown>4) {throw std::invalid_argument("unique identity and explicit bounded slowdown required");}
  auto converted=mtc::moveit_planner::to_timed_trajectory(raw,cfg.joints,frame,0.0);
  if (!converted.ok || converted.trajectory.points.size()<2 || converted.trajectory.points.front().time_from_start_s!=0) {
    throw std::invalid_argument("complete planned trajectory with t=0 start required: "+converted.error);
  }
  if (!mtc::moveit_planner::stretch_time(converted.trajectory,slowdown)) {throw std::invalid_argument("retiming rejected");}
  mtc::core::MotionCommand command;command.command_id=id;command.primitive_name="aubo_fake_noncontact_probe";
  command.simulation_only=true;command.requires_contact=false;command.joint_names=cfg.joints;
  command.velocity_scaling=0.1;command.acceleration_scaling=0.05;
  command.trajectory=std::move(converted.trajectory);command.trajectory->trajectory_id=id+"/trajectory";
  const double timeout=std::ceil((command.trajectory->duration_s()+3)*1000);
  if (!std::isfinite(timeout) || timeout>cfg.max_timeout_ms) {throw std::invalid_argument("planned timeout exceeds execution capability");}
  command.timeout_ms=static_cast<uint32_t>(timeout);
  const auto validation=mtc::trajectory::convert(command,cfg);
  if (!validation.trajectory) {throw std::invalid_argument(validation.error);}
  return command;
}
}
