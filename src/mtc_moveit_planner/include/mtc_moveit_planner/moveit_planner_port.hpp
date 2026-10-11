// Copyright 2026 meituan_challenge_ws contributors
//
// PlannerPort 的 MoveIt 2 实现（mtc::manipulation::PlannerPort）。
//
// 契约（README_FSM.md §Port 责任 + PickPlaceFsm::accept_plan）：
//   * 为 PlanTarget.primitives 的**每一个**原语返回一条 MotionCommand，按 primitive_name 对应；
//   * 每条命令携带**完整** TimedTrajectory（target_rad 为空），一次提交，不拆点；
//   * trajectory.joint_names 与 FSM 关节顺序逐元素一致；header_frame_id = planning frame；
//   * velocity/acceleration_scaling ∈ (0, 0.2]；timeout_ms >= 轨迹时长；
//   * 任一原语失败 => 整体 ok=false、error_code=PLANNING_FAILED(200)、commands 为空，
//     message 以 "[原因] 原语: 详情" 开头；不静默重试，不返回默认目标。
//
// 异步语义：plan() 首次调用启动后台规划并返回 nullopt；FSM 下一次 step()
// 以同一 PlanTarget 再调用时，若已完成则返回结果。同一目标不会重复启动规划。
//
// 起点衔接：同一 PlanTarget 内后一原语的起点 = 前一原语轨迹终点。
// 若两原语之间存在 ToolPort 动作（如 hook_engage），按 PrimitiveSpec::gap_before
// 假设其结束位置并在报告中注明——执行侧必须在发送前核对起点偏差。

#ifndef MTC_MOVEIT_PLANNER__MOVEIT_PLANNER_PORT_HPP_
#define MTC_MOVEIT_PLANNER__MOVEIT_PLANNER_PORT_HPP_

#include <future>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "mtc_core_cpp/clock.hpp"
#include "mtc_core_cpp/results.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_moveit_planner/planner_config.hpp"
#include "mtc_moveit_planner/planning_backend.hpp"

namespace mtc
{
namespace moveit_planner
{

/// 每个原语的规划记录（证据/日志用，不参与 FSM 契约）。
struct PrimitivePlanReport
{
  std::string primitive_name;
  std::string mode;
  std::string method;             ///< 后端实际使用的方法（ompl / cartesian / cartesian_preferred）
  std::string goal_kind;
  std::string start_assumption;   ///< "current_state" / "previous_primitive_end" / "simulated ENGAGE_INSERT ..."
  bool ok = false;
  std::string reason;
  std::string message;
  mtc::core::Pose6D ee_goal;      ///< planning frame 中 ee_link 目标
  double planning_time_s = 0.0;
  double cartesian_fraction = 0.0;
  double time_scale_applied = 1.0;
  double max_linear_speed_m_s = 0.0;
  double max_angular_speed_rad_s = 0.0;
  size_t point_count = 0;
  double duration_s = 0.0;
  uint32_t timeout_ms = 0;
  double state_watchdog_s = 0.0;
  bool contact_intent_only = false;
};

struct PlanReport
{
  std::string phase;
  std::string planning_frame;
  std::string end_effector_link;
  bool simulation_only = true;
  std::vector<std::string> uncalibrated_inputs;  ///< 例如 "TCP=PLACEHOLDER"、"slot T0=PLACEHOLDER"
  std::vector<PrimitivePlanReport> primitives;
  JointStateSnapshot start_state;
};

class MoveItPlannerPort : public mtc::manipulation::PlannerPort
{
public:
  MoveItPlannerPort(
    PlanningBackendPtr backend,
    PlannerConfig config,
    mtc::core::ClockPtr clock,
    bool asynchronous = true);

  ~MoveItPlannerPort() override;

  MoveItPlannerPort(const MoveItPlannerPort &) = delete;
  MoveItPlannerPort & operator=(const MoveItPlannerPort &) = delete;

  std::optional<mtc::core::PlanResult> plan(
    const mtc::manipulation::PlanTarget & target) override;

  /// 同步规划。start_override 为空时从后端当前状态开始（工具/离线脚本用于串接阶段）。
  mtc::core::PlanResult plan_blocking(
    const mtc::manipulation::PlanTarget & target,
    const std::optional<JointStateSnapshot> & start_override = std::nullopt);

  /// 最近一次完成的规划报告。
  PlanReport last_report() const;

  const PlannerConfig & config() const {return config_;}

  /// 由 PlanTarget 内容生成去重键（阶段/对象/工位/原语/提环位姿）。
  static std::string make_key(const mtc::manipulation::PlanTarget & target);

private:
  PlanningBackendPtr backend_;
  PlannerConfig config_;
  mtc::core::ClockPtr clock_;
  bool asynchronous_;

  mutable std::mutex mutex_;
  std::mutex backend_mutex_;
  std::map<std::string, std::future<mtc::core::PlanResult>> jobs_;
  PlanReport last_report_;
};

}  // namespace moveit_planner
}  // namespace mtc

#endif  // MTC_MOVEIT_PLANNER__MOVEIT_PLANNER_PORT_HPP_
