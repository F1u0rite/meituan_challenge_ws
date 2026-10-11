// Copyright 2026 meituan_challenge_ws contributors
//
// MotionAdapter -> PickPlace MotionPort 桥接。
//
// 存在理由：PickPlace FSM 的 MotionPort 是"返回 nullopt 表示在途"的异步契约，
// 而 mtc_motion_execution_cpp 的 MotionAdapter 是同步单次调用契约。
// 本桥接把两者对齐，使同一套运动适配器既服务于运动执行层，
// 也服务于抓放状态机，避免两份重复的运动后端实现。
//
// 第二阶段将在此基础上把 execute_joint_move 绑定到标准 ROS 2 Action
// （ExecuteJointMove / FollowJointTrajectory）的 goal/cancel/result。

#ifndef MTC_MANIPULATION_CPP__MOTION_ADAPTER_PORT_HPP_
#define MTC_MANIPULATION_CPP__MOTION_ADAPTER_PORT_HPP_

#include <string>

#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_motion_execution_cpp/motion_adapter.hpp"

namespace mtc
{
namespace manipulation
{

/// 把 mtc::motion::MotionAdapter 适配为 PickPlace 的 MotionPort。
///
/// 语义映射：
///   * 适配器返回结果 -> 立即返回该结果（非 nullopt），即"同步完成"；
///   * 若需要模拟"在途未完成"，应使用专门的 Fake 端口而非本桥接；
///   * request_stop 直接转发，stop_confirmed 语义原样保留（绝不代为确认）。
class MotionAdapterPort : public MotionPort
{
public:
  explicit MotionAdapterPort(mtc::motion::MotionAdapterPtr adapter);

  std::optional<mtc::core::MotionResult> execute_joint_move(
    const mtc::core::MotionCommand & command) override;

  mtc::core::StopResult request_stop(const std::string & reason) override;

  const mtc::motion::MotionAdapterPtr & adapter() const {return adapter_;}

private:
  mtc::motion::MotionAdapterPtr adapter_;
};

}  // namespace manipulation
}  // namespace mtc

#endif  // MTC_MANIPULATION_CPP__MOTION_ADAPTER_PORT_HPP_
