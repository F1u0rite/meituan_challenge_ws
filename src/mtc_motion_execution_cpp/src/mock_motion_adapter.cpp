// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"

#include <algorithm>
#include <cctype>
#include <utility>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace motion
{

namespace
{

mtc::core::MotionResult make_result(
  bool success, bool arrived, mtc::core::ExecState state, uint16_t code,
  const std::string & message)
{
  mtc::core::MotionResult result;
  result.success = success;
  result.arrived = arrived;
  result.exec_state = state;
  result.error_code = code;
  result.message = message;
  return result;
}

}  // namespace

MockMotionAdapter::MockMotionAdapter(
  MockMotionAdapterConfig config, mtc::core::ClockPtr clock, ValidationConfig validation)
: config_(std::move(config)),
  clock_(clock ? std::move(clock) : std::make_shared<mtc::core::SteadyClock>()),
  validation_(std::move(validation))
{
  capabilities_.backend_name = config_.backend_name;
  capabilities_.joint_goal_supported = config_.joint_goal_supported;
  capabilities_.timed_trajectory_supported = config_.timed_trajectory_supported;
  capabilities_.cartesian_motion_supported = config_.cartesian_motion_supported;
  capabilities_.cancel_supported = config_.cancel_supported;
  capabilities_.detail = config_.detail;

  position_.assign(validation_.expected_joint_names.size(), 0.0);
  velocity_.assign(validation_.expected_joint_names.size(), 0.0);
}

mtc::core::MotionCapabilities MockMotionAdapter::capabilities() const
{
  return capabilities_;
}

mtc::core::JointFeedback MockMotionAdapter::joint_feedback() const
{
  mtc::core::JointFeedback feedback;
  feedback.position_rad = position_;
  feedback.velocity_rad_s = velocity_;
  feedback.stamp_s = clock_->now();
  return feedback;
}

void MockMotionAdapter::advance_time(double seconds)
{
  if (seconds <= 0.0) {
    return;
  }
  // 仅当注入的是 ManualClock 时才推进；SteadyClock 下不睡眠，
  // 以免单元测试因模拟运动而变慢。
  if (auto * manual = dynamic_cast<mtc::core::ManualClock *>(clock_.get())) {
    manual->advance(seconds);
  }
}

mtc::core::MotionResult MockMotionAdapter::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  call_log_.push_back(command);

  const ValidationOutcome validation =
    validate_joint_move(command, validation_, capabilities_);
  if (!validation.ok) {
    last_validation_ = validation;
    return make_result(
      false, false, mtc::core::ExecState::REJECTED, validation.error_code, validation.message);
  }

  const auto acquired = guard_.acquire(command.command_id);
  if (!acquired.first) {
    return make_result(
      false, false, mtc::core::ExecState::REJECTED,
      mtc::core::ErrorCodes::EXECUTION_REJECTED, acquired.second);
  }

  active_command_ = command.command_id;
  cancel_requested_ = false;
  stop_confirmed_ = false;

  if (config_.fail_at == "status_unknown" || config_.status_unknown) {
    // 反馈断流：状态不确定，必须锁定动作，禁止自动重发。
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false, mtc::core::ExecState::STOP_UNKNOWN,
      mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN,
      "反馈断流，运动状态不确定；已锁定新的运动请求，禁止自动重发");
    result.status_unknown = true;
    result.final_position_rad = position_;
    return result;
  }

  const double budget_s = static_cast<double>(command.timeout_ms) / 1000.0;
  const double simulated_s = std::min(config_.travel_time_s, budget_s);

  if (config_.fail_at == "timeout" || config_.timeout || config_.travel_time_s > budget_s) {
    advance_time(budget_s);
    if (cancel_requested_) {
      guard_.release(command.command_id);
      active_command_.reset();
      mtc::core::MotionResult result = make_result(
        false, false,
        stop_confirmed_ ? mtc::core::ExecState::STOPPED : mtc::core::ExecState::STOP_UNKNOWN,
        stop_confirmed_ ? mtc::core::ErrorCodes::OK : mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
        stop_confirmed_ ? "已取消并确认停稳" : "已取消，但后端未确认停稳");
      result.stop_confirmed = stop_confirmed_;
      result.status_unknown = !stop_confirmed_;
      result.final_position_rad = position_;
      return result;
    }
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false, mtc::core::ExecState::TIMEOUT, mtc::core::ErrorCodes::MOTION_TIMEOUT,
      "运动超时（预算 " + std::to_string(command.timeout_ms) + " ms）");
    result.timed_out = true;
    result.status_unknown = true;
    result.final_position_rad = position_;
    return result;
  }

  // 逐步"运动"：中途允许被取消。
  const int steps = std::max(1, static_cast<int>(simulated_s / 0.05));
  for (int i = 0; i < steps; ++i) {
    if (cancel_requested_) {
      break;
    }
    advance_time(simulated_s / steps);
    velocity_.clear();
    for (size_t j = 0; j < position_.size() && j < command.target_rad.size(); ++j) {
      velocity_.push_back(
        (command.target_rad[j] - position_[j]) / std::max(simulated_s / steps, 1e-9));
    }
  }

  if (cancel_requested_) {
    guard_.release(command.command_id);
    active_command_.reset();
    mtc::core::MotionResult result = make_result(
      false, false,
      stop_confirmed_ ? mtc::core::ExecState::STOPPED : mtc::core::ExecState::STOP_UNKNOWN,
      stop_confirmed_ ? mtc::core::ErrorCodes::OK : mtc::core::ErrorCodes::CANCEL_NOT_CONFIRMED,
      stop_confirmed_ ? "已取消并确认停稳" : "已取消，但后端未确认停稳");
    result.stop_confirmed = stop_confirmed_;
    result.status_unknown = !stop_confirmed_;
    result.final_position_rad = position_;
    return result;
  }

  position_ = command.target_rad;
  velocity_.assign(position_.size(), 0.0);
  guard_.release(command.command_id);
  active_command_.reset();

  mtc::core::MotionResult result = make_result(
    true, true, mtc::core::ExecState::SETTLED, mtc::core::ErrorCodes::OK, "已到位并停稳");
  result.stop_confirmed = true;
  result.final_position_rad = position_;
  return result;
}

mtc::core::StopResult MockMotionAdapter::request_stop(const std::string & reason)
{
  mtc::core::StopResult result;

  if (!active_command_.has_value()) {
    stop_confirmed_ = true;
    result.request_delivered = true;
    result.stop_confirmed = true;
    result.message = "无在途命令，已确认停稳（原因：" + reason + "）";
    return result;
  }

  cancel_requested_ = true;
  stop_confirmed_ = config_.cancel_confirms;
  result.request_delivered = true;
  result.stop_confirmed = stop_confirmed_;

  if (stop_confirmed_) {
    velocity_.assign(velocity_.size(), 0.0);
    guard_.release(*active_command_);
    result.message = "停止请求已送达并确认停稳（原因：" + reason + "）";
    return result;
  }
  result.message = "停止请求已送达，但尚未确认停稳（原因：" + reason + "）";
  return result;
}

std::optional<bool> MockMotionAdapter::explicit_stop_confirmation() const
{
  return stop_confirmed_;
}

bool MockMotionAdapter::force_begin_in_flight(const std::string & command_id)
{
  const auto acquired = guard_.acquire(command_id);
  if (!acquired.first) {
    return false;
  }
  active_command_ = command_id;
  cancel_requested_ = false;
  stop_confirmed_ = false;
  return true;
}

void MockMotionAdapter::force_end_in_flight()
{
  guard_.force_release();
  active_command_.reset();
  cancel_requested_ = false;
}

mtc::core::MotionResult MockMotionAdapter::execute_trajectory(
  const mtc::core::TimedTrajectory & trajectory)
{
  // 记录调用以便测试断言"轨迹只被当作一条轨迹提交一次"，
  // 而不是被拆成多次单点调用。
  trajectory_call_log_.push_back(trajectory);

  const ValidationOutcome validation =
    validate_timed_trajectory(trajectory, validation_, capabilities_);
  if (!validation.ok) {
    return make_result(
      false, false, mtc::core::ExecState::REJECTED, validation.error_code, validation.message);
  }
  return make_result(
    true, true, mtc::core::ExecState::SETTLED, mtc::core::ErrorCodes::OK,
    "轨迹已在 Mock 后端执行完毕（" + std::to_string(trajectory.points.size()) + " 个路点）");
}

// ---------------------------------------------------------------------------
// DisabledMotionAdapter
// ---------------------------------------------------------------------------

DisabledMotionAdapter::DisabledMotionAdapter(std::string reason)
: reason_(std::move(reason))
{
}

mtc::core::MotionCapabilities DisabledMotionAdapter::capabilities() const
{
  mtc::core::MotionCapabilities capabilities;
  capabilities.backend_name = "disabled";
  capabilities.joint_goal_supported = false;
  capabilities.timed_trajectory_supported = false;
  capabilities.cartesian_motion_supported = false;
  capabilities.cancel_supported = false;
  capabilities.detail = reason_;
  return capabilities;
}

mtc::core::MotionResult DisabledMotionAdapter::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  return make_result(
    false, false, mtc::core::ExecState::REJECTED, mtc::core::ErrorCodes::EXECUTION_REJECTED,
    reason_ + "（command_id=" + command.command_id + "）");
}

mtc::core::StopResult DisabledMotionAdapter::request_stop(const std::string & reason)
{
  mtc::core::StopResult result;
  result.request_delivered = false;
  result.stop_confirmed = false;
  result.message = reason_ + "；未送达任何真实停止请求（原因：" + reason + "）";
  return result;
}

mtc::core::JointFeedback DisabledMotionAdapter::joint_feedback() const
{
  return mtc::core::JointFeedback{};
}

// ---------------------------------------------------------------------------
// 工厂
// ---------------------------------------------------------------------------

MotionAdapterPtr make_adapter(
  const std::string & name,
  MockMotionAdapterConfig config,
  mtc::core::ClockPtr clock,
  ValidationConfig validation)
{
  std::string normalized = name;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (normalized == "mock") {
    return std::make_shared<MockMotionAdapter>(std::move(config), std::move(clock),
             std::move(validation));
  }
  return std::make_shared<DisabledMotionAdapter>(
    "不支持或未启用的运动后端 \"" + name + "\"；仅 mock 可用于离线运行");
}

}  // namespace motion
}  // namespace mtc
