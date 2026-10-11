// Copyright 2026 meituan_challenge_ws contributors

#include "mtc_manipulation_cpp/motion_adapter_port.hpp"

#include <utility>

namespace mtc
{
namespace manipulation
{

MotionAdapterPort::MotionAdapterPort(mtc::motion::MotionAdapterPtr adapter)
: adapter_(std::move(adapter))
{
}

std::optional<mtc::core::MotionResult> MotionAdapterPort::execute_joint_move(
  const mtc::core::MotionCommand & command)
{
  // 同步契约：适配器返回即为最终结论，不做任何"代为成功"的加工。
  return adapter_->execute_joint_move(command);
}

mtc::core::StopResult MotionAdapterPort::request_stop(const std::string & reason)
{
  // stop_confirmed 原样透传：桥接层绝不代替后端确认停稳。
  return adapter_->request_stop(reason);
}

}  // namespace manipulation
}  // namespace mtc
