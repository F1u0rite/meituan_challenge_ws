// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_motion_execution_cpp/executor_core.hpp"

#include <exception>
#include <utility>

#include "mtc_core_cpp/error_codes.hpp"

namespace mtc
{
namespace motion
{

void HandshakeRecord::mark(const std::string & step, const std::string & detail)
{
  steps.push_back(detail.empty() ? step : (step + ":" + detail));
}

MotionExecutorCore::MotionExecutorCore(MotionAdapterPtr adapter, ValidationConfig config)
: adapter_(std::move(adapter)),
  config_(std::move(config))
{
}

mtc::core::MotionCapabilities MotionExecutorCore::capabilities() const
{
  return adapter_ ? adapter_->capabilities() : mtc::core::MotionCapabilities{};
}

ExecutorSnapshot MotionExecutorCore::snapshot() const
{
  ExecutorSnapshot snapshot;
  snapshot.exec_state = exec_state_;
  snapshot.active_command_id = active_command_id_;
  snapshot.last_error_code = last_error_code_;
  snapshot.motion_state_unknown = motion_state_unknown_;
  snapshot.stop_confirmed = stop_confirmed_;
  return snapshot;
}

const HandshakeRecord * MotionExecutorCore::handshake(const std::string & command_id) const
{
  const auto found = handshakes_.find(command_id);
  return (found != handshakes_.end()) ? &found->second : nullptr;
}

mtc::core::MotionResult MotionExecutorCore::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  HandshakeRecord & record = handshakes_[command.command_id];
  record.command_id = command.command_id;
  record.mark("PREPARE");

  // 状态未知时优先锁定：未确认状态前一律拒绝新动作。
  if (motion_state_unknown_) {
    exec_state_ = mtc::core::ExecState::STOP_UNKNOWN;
    last_error_code_ = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    record.mark("PREPARE_REJECTED", "motion_state_unknown");

    mtc::core::MotionResult result;
    result.success = false;
    result.arrived = false;
    result.stop_confirmed = false;
    result.exec_state = mtc::core::ExecState::STOP_UNKNOWN;
    result.error_code = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    result.status_unknown = true;
    result.message =
      "上一次运动状态未知且未确认，已锁住新的运动请求；禁止自动重发";
    return result;
  }

  // ACCEPT：唯一执行权由适配器 guard 与 core 双重把关。
  if (active_command_id_.has_value()) {
    exec_state_ = mtc::core::ExecState::REJECTED;
    last_error_code_ = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    record.mark("ACCEPT_REJECTED", "in_flight=" + *active_command_id_);

    mtc::core::MotionResult result;
    result.success = false;
    result.arrived = false;
    result.exec_state = mtc::core::ExecState::REJECTED;
    result.error_code = mtc::core::ErrorCodes::EXECUTION_REJECTED;
    result.message = "已有在途命令 " + *active_command_id_ +
      "，拒绝并发运动请求 " + command.command_id;
    return result;
  }
  active_command_id_ = command.command_id;
  record.mark("ACCEPT");

  // VALIDATE：能力与目标校验，能力不足直接拒绝而不是变形执行。
  const ValidationOutcome validation =
    validate_joint_move(command, config_, adapter_->capabilities());
  if (!validation.ok) {
    active_command_id_.reset();
    exec_state_ = mtc::core::ExecState::REJECTED;
    last_error_code_ = validation.error_code;
    record.mark("VALIDATE_REJECTED", validation.message);

    mtc::core::MotionResult result;
    result.success = false;
    result.arrived = false;
    result.exec_state = mtc::core::ExecState::REJECTED;
    result.error_code = validation.error_code;
    result.message = validation.message;
    return result;
  }
  record.mark("VALIDATE");

  // SEND + MONITOR + CONFIRM：交给适配器实现，核心只解释结果。
  exec_state_ = mtc::core::ExecState::MOVING;
  record.mark("SEND");
  record.mark("MONITOR_REQUEST", mtc::core::to_string(mtc::core::ExecState::MOVING));

  mtc::core::MotionResult result;
  try {
    result = adapter_->execute_joint_move(command);
  } catch (const std::exception & exc) {
    // 后端异常一律按状态未知处理，禁止重发
    active_command_id_.reset();
    exec_state_ = mtc::core::ExecState::STOP_UNKNOWN;
    last_error_code_ = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    motion_state_unknown_ = true;
    stop_confirmed_ = false;
    record.mark("MONITOR_EXCEPTION", exc.what());

    mtc::core::MotionResult failure;
    failure.success = false;
    failure.arrived = false;
    failure.stop_confirmed = false;
    failure.exec_state = mtc::core::ExecState::STOP_UNKNOWN;
    failure.error_code = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    failure.status_unknown = true;
    failure.message = std::string("后端执行异常，运动状态不确定：") + exc.what() +
      "；禁止自动重发";
    return failure;
  } catch (...) {
    active_command_id_.reset();
    exec_state_ = mtc::core::ExecState::STOP_UNKNOWN;
    last_error_code_ = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    motion_state_unknown_ = true;
    stop_confirmed_ = false;
    record.mark("MONITOR_EXCEPTION", "unknown");

    mtc::core::MotionResult failure;
    failure.success = false;
    failure.arrived = false;
    failure.stop_confirmed = false;
    failure.exec_state = mtc::core::ExecState::STOP_UNKNOWN;
    failure.error_code = mtc::core::ErrorCodes::MOTION_STATUS_UNKNOWN;
    failure.status_unknown = true;
    failure.message = "后端执行异常，运动状态不确定；禁止自动重发";
    return failure;
  }

  exec_state_ = result.exec_state;
  last_error_code_ = result.error_code;
  motion_state_unknown_ = result.status_unknown;
  stop_confirmed_ = result.stop_confirmed;
  active_command_id_.reset();
  record.mark("MONITOR", mtc::core::to_string(result.exec_state));
  if (result.success) {
    record.mark("CONFIRM", "settled");
  } else {
    record.mark("STOP_OR_FAULT", result.message);
  }
  return result;
}

mtc::core::StopResult MotionExecutorCore::request_stop(
  const std::string & reason,
  std::optional<std::string> in_flight_command_id)
{
  if (in_flight_command_id.has_value()) {
    active_command_id_ = in_flight_command_id;
  }

  const mtc::core::StopResult adapter_result = adapter_->request_stop(reason);
  const mtc::core::MotionCapabilities capabilities = adapter_->capabilities();

  mtc::core::StopResult result;
  result.request_delivered = adapter_result.request_delivered;
  result.message = adapter_result.message;

  if (!capabilities.cancel_supported) {
    // 后端不支持取消时，不能伪造 stop_confirmed。
    stop_confirmed_ = false;
    motion_state_unknown_ = true;
    exec_state_ = mtc::core::ExecState::STOP_UNKNOWN;
    result.request_delivered = false;
    result.stop_confirmed = false;
    result.message = "后端 " + capabilities.backend_name +
      " 未声明 cancel_supported：" + adapter_result.message;
    return result;
  }

  // 送达 != 停稳。显式标志优先；无标志可用时退回保守判断。
  if (motion_state_unknown_) {
    // 运动状态未知时，任何停止请求都不能被宣告为"已确认停稳"。
    stop_confirmed_ = false;
  } else {
    const std::optional<bool> explicit_flag = adapter_->explicit_stop_confirmation();
    if (explicit_flag.has_value()) {
      stop_confirmed_ = result.request_delivered && *explicit_flag;
    } else if (!active_command_id_.has_value()) {
      stop_confirmed_ = result.request_delivered;
    } else {
      stop_confirmed_ = false;
    }
  }

  exec_state_ = stop_confirmed_ ? mtc::core::ExecState::STOPPED : mtc::core::ExecState::CANCELLING;
  result.stop_confirmed = stop_confirmed_;
  return result;
}

void MotionExecutorCore::clear_unknown_after_manual_confirmation(const std::string & note)
{
  motion_state_unknown_ = false;
  exec_state_ = mtc::core::ExecState::IDLE;
  stop_confirmed_ = true;
  last_error_code_ = mtc::core::ErrorCodes::OK;

  HandshakeRecord & record = handshakes_["manual"];
  record.command_id = "manual";
  record.mark("MANUAL_CONFIRMATION", note);
}

}  // namespace motion
}  // namespace mtc
